//! A simple Driver for the Waveshare 7.5" E-Ink Display (V2) via SPI
//!
//! # References
//!
//! - [Datasheet](https://www.waveshare.com/wiki/7.5inch_e-Paper_HAT)
//! - [Waveshare C driver](https://github.com/waveshare/e-Paper/blob/702def0/RaspberryPi%26JetsonNano/c/lib/e-Paper/EPD_7in5_V2.c)
//! - [Waveshare Python driver](https://github.com/waveshare/e-Paper/blob/702def0/RaspberryPi%26JetsonNano/python/lib/waveshare_epd/epd7in5_V2.py)
//!
//! Important note for V2:
//! Revision V2 has been released on 2019.11, the resolution is upgraded to 800×480, from 640×384 of V1.
//! The hardware and interface of V2 are compatible with V1, however, the related software should be updated.

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::SpiDevice,
};

use crate::color::Color;
use crate::interface::DisplayInterface;
pub use crate::interface::BusyTimeoutError;
use crate::traits::{InternalWiAdditions, RefreshLut, WaveshareDisplay};

pub(crate) mod command;
use self::command::Command;
use crate::buffer_len;

/// Full size buffer for use with the 7in5 v2 EPD
#[cfg(feature = "graphics")]
pub type Display7in5 = crate::graphics::Display<
    WIDTH,
    HEIGHT,
    false,
    { buffer_len(WIDTH as usize, HEIGHT as usize) },
    Color,
>;

/// Width of the display
pub const WIDTH: u32 = 800;
/// Height of the display
pub const HEIGHT: u32 = 480;
/// Default Background Color
pub const DEFAULT_BACKGROUND_COLOR: Color = Color::White;
const IS_BUSY_LOW: bool = true;
const SINGLE_BYTE_WRITE: bool = false;

/// Epd7in5 (V2) driver
///
pub struct Epd7in5<SPI, BUSY, DC, RST, DELAY> {
    /// Connection Interface
    interface: DisplayInterface<SPI, BUSY, DC, RST, DELAY, SINGLE_BYTE_WRITE>,
    /// Background Color
    color: Color,
}

impl<SPI, BUSY, DC, RST, DELAY> InternalWiAdditions<SPI, BUSY, DC, RST, DELAY>
    for Epd7in5<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    fn init(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        // Reset the device
        self.interface.reset(delay, 10_000, 2_000);

        // V2 procedure as described here:
        // https://github.com/waveshare/e-Paper/blob/master/RaspberryPi%26JetsonNano/python/lib/waveshare_epd/epd7in5bc_V2.py
        // and as per specs:
        // https://www.waveshare.com/w/upload/6/60/7.5inch_e-Paper_V2_Specification.pdf

        self.cmd_with_data(spi, Command::PowerSetting, &[0x07, 0x07, 0x3f, 0x3f])?;
        self.cmd_with_data(spi, Command::BoosterSoftStart, &[0x17, 0x17, 0x28, 0x17])?;
        self.command(spi, Command::PowerOn)?;
        delay.delay_ms(100);
        self.wait_until_idle(spi, delay)?;
        self.cmd_with_data(spi, Command::PanelSetting, &[0x1F])?;
        self.cmd_with_data(spi, Command::TconResolution, &[0x03, 0x20, 0x01, 0xE0])?;
        self.cmd_with_data(spi, Command::DualSpi, &[0x00])?;
        // CDI byte1 0x20 selects BDV=10, driving the panel's VCOM border (the
        // strip outside the 800x480 array) black. The Waveshare default 0x10
        // (BDV=01) drives it white, which shows as a ~1px white gutter wherever
        // the bezel exposes the border edge.
        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0x20, 0x07])?;
        self.cmd_with_data(spi, Command::TconSetting, &[0x22])?;
        Ok(())
    }
}

impl<SPI, BUSY, DC, RST, DELAY> WaveshareDisplay<SPI, BUSY, DC, RST, DELAY>
    for Epd7in5<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    type DisplayColor = Color;
    fn new(
        spi: &mut SPI,
        busy: BUSY,
        dc: DC,
        rst: RST,
        delay: &mut DELAY,
        delay_us: Option<u32>,
    ) -> Result<Self, SPI::Error> {
        let interface = DisplayInterface::new(busy, dc, rst, delay_us);
        let color = DEFAULT_BACKGROUND_COLOR;

        let mut epd = Epd7in5 { interface, color };

        epd.init(spi, delay)?;

        Ok(epd)
    }

    fn wake_up(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.init(spi, delay)
    }

    fn sleep(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay)?;
        self.command(spi, Command::PowerOff)?;
        self.wait_until_idle(spi, delay)?;
        self.cmd_with_data(spi, Command::DeepSleep, &[0xA5])?;
        Ok(())
    }

    fn update_frame(
        &mut self,
        spi: &mut SPI,
        buffer: &[u8],
        delay: &mut DELAY,
    ) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay)?;
        // Waveshare's reference C demo (EPD_7in5_V2.c::EPD_7IN5_V2_Display)
        // sends the framebuffer to DTM1 (0x10) raw and to DTM2 (0x13) bitwise-
        // inverted. The user-facing convention is bit 1 = white; the panel's
        // DTM2 register expects bit 0 = white (datasheet §22, KW mode with
        // NEW/OLD, DDX=00). Writing both DTM1 and DTM2 with opposite polarity
        // forces a full LUTKW/LUTWK transition for every pixel, producing
        // strong contrast. Without this, every framebuffer renders inverted.
        self.cmd_with_data(spi, Command::DataStartTransmission1, buffer)?;
        self.command(spi, Command::DataStartTransmission2)?;
        self.interface.data_inverted(spi, buffer)?;
        Ok(())
    }

    fn update_partial_frame(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        buffer: &[u8],
        x: u32,
        y: u32,
        width: u32,
        height: u32,
    ) -> Result<(), SPI::Error> {
        // UC8179 partial-window addressing is byte-aligned on the x axis (each byte =
        // 8 horizontal pixels). The reference C demo enforces this by construction;
        // we panic so callers don't silently scramble pixels with off-by-bit windows.
        assert!(x % 8 == 0, "epd7in5_v2: partial x must be multiple of 8");
        assert!(width % 8 == 0, "epd7in5_v2: partial width must be multiple of 8");
        let row_bytes = (width / 8) as usize;
        assert_eq!(
            buffer.len(),
            row_bytes * height as usize,
            "epd7in5_v2: partial buffer size mismatch",
        );

        self.wait_until_idle(spi, delay)?;

        // CDI = 0xA9 selects the differential (KW) waveform path used during partial
        // refresh; the init value (0x20) drives the full LUT and would ghost badly.
        // Restored at the end so subsequent full refreshes are unaffected.
        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0xA9, 0x07])?;

        self.command(spi, Command::PartialIn)?;

        let x_end = x + width - 1;
        let y_end = y + height - 1;
        self.cmd_with_data(
            spi,
            Command::PartialWindow,
            &[
                (x >> 8) as u8,
                (x & 0xFF) as u8,
                (x_end >> 8) as u8,
                (x_end & 0xFF) as u8,
                (y >> 8) as u8,
                (y & 0xFF) as u8,
                (y_end >> 8) as u8,
                (y_end & 0xFF) as u8,
                0x01,
            ],
        )?;

        // Waveshare reference writes the framebuffer raw to DTM2 (0x13) for partial
        // refresh — opposite polarity from the full-refresh DTM2 path which inverts.
        // The CDI=0xA9 LUT compensates, so raw bytes here render correctly.
        self.cmd_with_data(spi, Command::DataStartTransmission2, buffer)?;

        self.command(spi, Command::DisplayRefresh)?;
        self.wait_until_idle(spi, delay)?;

        self.command(spi, Command::PartialOut)?;

        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0x20, 0x07])?;

        Ok(())
    }

    fn display_frame(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay)?;
        self.command(spi, Command::DisplayRefresh)?;
        Ok(())
    }

    fn update_and_display_frame(
        &mut self,
        spi: &mut SPI,
        buffer: &[u8],
        delay: &mut DELAY,
    ) -> Result<(), SPI::Error> {
        self.update_frame(spi, buffer, delay)?;
        self.command(spi, Command::DisplayRefresh)?;
        Ok(())
    }

    fn clear_frame(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay)?;
        self.send_resolution(spi)?;

        // Match Waveshare's `EPD_7IN5_V2_Clear` (DTM1=0xFF, DTM2=0x00) so
        // every pixel transitions black->white via LUTKW. See `update_frame`
        // for the polarity rationale.
        self.command(spi, Command::DataStartTransmission1)?;
        self.interface.data_x_times(spi, 0xFF, WIDTH / 8 * HEIGHT)?;

        self.command(spi, Command::DataStartTransmission2)?;
        self.interface.data_x_times(spi, 0x00, WIDTH / 8 * HEIGHT)?;

        self.command(spi, Command::DisplayRefresh)?;
        Ok(())
    }

    fn set_background_color(&mut self, color: Color) {
        self.color = color;
    }

    fn background_color(&self) -> &Color {
        &self.color
    }

    fn width(&self) -> u32 {
        WIDTH
    }

    fn height(&self) -> u32 {
        HEIGHT
    }

    fn set_lut(
        &mut self,
        _spi: &mut SPI,
        _delay: &mut DELAY,
        _refresh_rate: Option<RefreshLut>,
    ) -> Result<(), SPI::Error> {
        unimplemented!();
    }

    fn wait_until_idle(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.interface
            .wait_until_idle_with_cmd(spi, delay, IS_BUSY_LOW, Command::GetStatus)
    }
}

impl<SPI, BUSY, DC, RST, DELAY> Epd7in5<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    fn command(&mut self, spi: &mut SPI, command: Command) -> Result<(), SPI::Error> {
        self.interface.cmd(spi, command)
    }

    fn send_data(&mut self, spi: &mut SPI, data: &[u8]) -> Result<(), SPI::Error> {
        self.interface.data(spi, data)
    }

    fn cmd_with_data(
        &mut self,
        spi: &mut SPI,
        command: Command,
        data: &[u8],
    ) -> Result<(), SPI::Error> {
        self.interface.cmd_with_data(spi, command, data)
    }

    fn send_resolution(&mut self, spi: &mut SPI) -> Result<(), SPI::Error> {
        let w = self.width();
        let h = self.height();

        self.command(spi, Command::TconResolution)?;
        self.send_data(spi, &[(w >> 8) as u8])?;
        self.send_data(spi, &[w as u8])?;
        self.send_data(spi, &[(h >> 8) as u8])?;
        self.send_data(spi, &[h as u8])
    }
}

// ─── Timeout-aware variants ────────────────────────────────────────────────
//
// V2-specific inherent methods that wrap each operation containing an internal
// BUSY wait with a per-call cap. The trait-level API in `WaveshareDisplay`
// stays unchanged so existing call sites compile unchanged; new wake-flow code
// (e.g. M4 in adhan_clock) uses these methods exclusively to surface a real
// timeout instead of hanging on a stuck panel.
impl<SPI, BUSY, DC, RST, DELAY> Epd7in5<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    /// Construct + run init with a BUSY-wait cap. Mirrors `WaveshareDisplay::new`
    /// + `init`, replacing the post-PowerOn unbounded wait with a bounded one.
    pub fn new_with_timeout(
        spi: &mut SPI,
        busy: BUSY,
        dc: DC,
        rst: RST,
        delay: &mut DELAY,
        delay_us: Option<u32>,
        timeout_us: u32,
    ) -> Result<Self, BusyTimeoutError<SPI::Error>> {
        let interface = DisplayInterface::new(busy, dc, rst, delay_us);
        let color = DEFAULT_BACKGROUND_COLOR;

        let mut epd = Epd7in5 { interface, color };
        epd.init_with_timeout(spi, delay, timeout_us)?;

        Ok(epd)
    }

    /// Bounded poll on BUSY using V2's `0x71 GetStatus` probe between polls.
    pub fn wait_until_idle_with_timeout(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        timeout_us: u32,
    ) -> Result<(), BusyTimeoutError<SPI::Error>> {
        self.interface.wait_until_idle_with_cmd_timeout(
            spi,
            delay,
            IS_BUSY_LOW,
            Command::GetStatus,
            timeout_us,
        )
    }

    /// `update_and_display_frame` with a BUSY-wait cap on the pre-flight idle
    /// check. The DisplayRefresh that follows is fire-and-forget; the caller
    /// is expected to call `wait_until_idle_with_timeout` afterward.
    pub fn update_and_display_frame_with_timeout(
        &mut self,
        spi: &mut SPI,
        buffer: &[u8],
        delay: &mut DELAY,
        timeout_us: u32,
    ) -> Result<(), BusyTimeoutError<SPI::Error>> {
        self.wait_until_idle_with_timeout(spi, delay, timeout_us)?;
        self.update_frame_no_wait(spi, buffer)?;
        self.command(spi, Command::DisplayRefresh)?;
        Ok(())
    }

    /// `update_partial_frame` with a BUSY-wait cap on the pre-flight idle
    /// check. Mirrors the trait method's window/byte-alignment asserts.
    pub fn update_partial_frame_with_timeout(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        buffer: &[u8],
        x: u32,
        y: u32,
        width: u32,
        height: u32,
        timeout_us: u32,
    ) -> Result<(), BusyTimeoutError<SPI::Error>> {
        assert!(x % 8 == 0, "epd7in5_v2: partial x must be multiple of 8");
        assert!(width % 8 == 0, "epd7in5_v2: partial width must be multiple of 8");
        let row_bytes = (width / 8) as usize;
        assert_eq!(
            buffer.len(),
            row_bytes * height as usize,
            "epd7in5_v2: partial buffer size mismatch",
        );

        self.wait_until_idle_with_timeout(spi, delay, timeout_us)?;

        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0xA9, 0x07])?;
        self.command(spi, Command::PartialIn)?;

        let x_end = x + width - 1;
        let y_end = y + height - 1;
        self.cmd_with_data(
            spi,
            Command::PartialWindow,
            &[
                (x >> 8) as u8,
                (x & 0xFF) as u8,
                (x_end >> 8) as u8,
                (x_end & 0xFF) as u8,
                (y >> 8) as u8,
                (y & 0xFF) as u8,
                (y_end >> 8) as u8,
                (y_end & 0xFF) as u8,
                0x01,
            ],
        )?;

        self.cmd_with_data(spi, Command::DataStartTransmission2, buffer)?;

        self.command(spi, Command::DisplayRefresh)?;
        self.wait_until_idle_with_timeout(spi, delay, timeout_us)?;

        self.command(spi, Command::PartialOut)?;
        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0x20, 0x07])?;

        Ok(())
    }

    /// Windowed partial refresh that re-establishes both DTM1 (old) and DTM2
    /// (new) before triggering. Required after the panel's `0x07 DeepSleep`
    /// because UC8179 RAM is not retained — the controller's stored DTM1 is
    /// gone and the differential `CDI=0xA9` waveform needs both registers
    /// populated to compute per-pixel transitions correctly.
    ///
    /// `old_buffer` is the framebuffer that's currently on glass in the
    /// window; `new_buffer` is the target. Both must be `(width / 8) * height`
    /// bytes. Both are sent raw (matching the polarity the previous full
    /// refresh would have left in DTM1 — see `update_frame` for the polarity
    /// rationale: full refresh writes DTM1=raw, DTM2=inverted under
    /// `CDI=0x10`; partial under `CDI=0xA9` reads the differential, so the
    /// DTM1 value the panel last saw was `raw`).
    pub fn update_partial_frame_dual_with_timeout(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        old_buffer: &[u8],
        new_buffer: &[u8],
        x: u32,
        y: u32,
        width: u32,
        height: u32,
        timeout_us: u32,
    ) -> Result<(), BusyTimeoutError<SPI::Error>> {
        assert!(x % 8 == 0, "epd7in5_v2: partial x must be multiple of 8");
        assert!(width % 8 == 0, "epd7in5_v2: partial width must be multiple of 8");
        let row_bytes = (width / 8) as usize;
        assert_eq!(
            old_buffer.len(),
            row_bytes * height as usize,
            "epd7in5_v2: partial old_buffer size mismatch",
        );
        assert_eq!(
            new_buffer.len(),
            row_bytes * height as usize,
            "epd7in5_v2: partial new_buffer size mismatch",
        );

        self.wait_until_idle_with_timeout(spi, delay, timeout_us)?;

        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0xA9, 0x07])?;
        self.command(spi, Command::PartialIn)?;

        let x_end = x + width - 1;
        let y_end = y + height - 1;
        self.cmd_with_data(
            spi,
            Command::PartialWindow,
            &[
                (x >> 8) as u8,
                (x & 0xFF) as u8,
                (x_end >> 8) as u8,
                (x_end & 0xFF) as u8,
                (y >> 8) as u8,
                (y & 0xFF) as u8,
                (y_end >> 8) as u8,
                (y_end & 0xFF) as u8,
                0x01,
            ],
        )?;

        self.cmd_with_data(spi, Command::DataStartTransmission1, old_buffer)?;
        self.cmd_with_data(spi, Command::DataStartTransmission2, new_buffer)?;

        self.command(spi, Command::DisplayRefresh)?;
        self.wait_until_idle_with_timeout(spi, delay, timeout_us)?;

        self.command(spi, Command::PartialOut)?;
        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0x20, 0x07])?;

        Ok(())
    }

    /// `sleep` with a BUSY-wait cap on the post-PowerOff idle check. On
    /// timeout, the `0x07 DeepSleep` command is skipped and the error
    /// returned; the caller is expected to drop PWR to force a clean cold
    /// reset on the next wake.
    pub fn sleep_with_timeout(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        timeout_us: u32,
    ) -> Result<(), BusyTimeoutError<SPI::Error>> {
        self.wait_until_idle_with_timeout(spi, delay, timeout_us)?;
        self.command(spi, Command::PowerOff)?;
        self.wait_until_idle_with_timeout(spi, delay, timeout_us)?;
        self.cmd_with_data(spi, Command::DeepSleep, &[0xA5])?;
        Ok(())
    }

    /// `wake_up` with a BUSY-wait cap. Re-runs init via `init_with_timeout`.
    pub fn wake_up_with_timeout(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        timeout_us: u32,
    ) -> Result<(), BusyTimeoutError<SPI::Error>> {
        self.init_with_timeout(spi, delay, timeout_us)
    }

    fn init_with_timeout(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        timeout_us: u32,
    ) -> Result<(), BusyTimeoutError<SPI::Error>> {
        self.interface.reset(delay, 10_000, 2_000);

        self.cmd_with_data(spi, Command::PowerSetting, &[0x07, 0x07, 0x3f, 0x3f])?;
        self.cmd_with_data(spi, Command::BoosterSoftStart, &[0x17, 0x17, 0x28, 0x17])?;
        self.command(spi, Command::PowerOn)?;
        delay.delay_ms(100);
        self.wait_until_idle_with_timeout(spi, delay, timeout_us)?;
        self.cmd_with_data(spi, Command::PanelSetting, &[0x1F])?;
        self.cmd_with_data(spi, Command::TconResolution, &[0x03, 0x20, 0x01, 0xE0])?;
        self.cmd_with_data(spi, Command::DualSpi, &[0x00])?;
        // CDI byte1 0x20 selects BDV=10, driving the panel's VCOM border (the
        // strip outside the 800x480 array) black. The Waveshare default 0x10
        // (BDV=01) drives it white, which shows as a ~1px white gutter wherever
        // the bezel exposes the border edge.
        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0x20, 0x07])?;
        self.cmd_with_data(spi, Command::TconSetting, &[0x22])?;
        Ok(())
    }

    fn update_frame_no_wait(&mut self, spi: &mut SPI, buffer: &[u8]) -> Result<(), SPI::Error> {
        self.cmd_with_data(spi, Command::DataStartTransmission1, buffer)?;
        self.command(spi, Command::DataStartTransmission2)?;
        self.interface.data_inverted(spi, buffer)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn epd_size() {
        assert_eq!(WIDTH, 800);
        assert_eq!(HEIGHT, 480);
        assert_eq!(DEFAULT_BACKGROUND_COLOR, Color::White);
    }
}
