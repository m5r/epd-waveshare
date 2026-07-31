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
use embedded_hal_async::{delay::DelayNs as AsyncDelayNs, spi::SpiDevice as AsyncSpiDevice};

use crate::color::Color;
pub use crate::interface::BusyTimeoutError;
use crate::interface::DisplayInterface;
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

/// Default SPI write chunk for the async cancellable framebuffer transfers. The
/// cancel hook is checked once per chunk, so smaller chunks = finer cancel
/// granularity at the cost of more SPI transactions. 4 KB ≈ one Linux SPI
/// transfer cap and divides the 48 000-byte 800×480 framebuffer into 12 bands.
const ASYNC_WRITE_CHUNK: usize = 4096;

/// Outcome of an async, cooperatively-cancellable refresh operation.
///
/// `Completed` means the operation ran to its normal end. `Cancelled` means the
/// caller's `should_cancel` hook returned true at a known command boundary, so
/// the method stopped issuing SPI and returned early.
///
/// Cancellation is COOPERATIVE: the method observes `should_cancel` at each
/// boundary (every BUSY poll, between framebuffer chunks, and immediately before
/// `DisplayRefresh`) and returns `Cancelled` there. Dropping the future
/// mid-`.await` is NOT the intended cancellation mechanism — it would strand the
/// UC8179 mid-transaction. Callers that observe `Cancelled` from a method that
/// had already issued `DisplayRefresh` (i.e. `wait_until_idle_*`) must treat the
/// panel as mid-refresh and hard-reset it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefreshOutcome {
    /// The operation ran to completion.
    Completed,
    /// The caller's cancel hook fired; the method stopped at a known boundary.
    Cancelled,
}

/// Waveform source for the fast partial-refresh mode.
///
/// The plain init (`new_*` / `wake_up_*`) leaves PSR REG=0, so EVERY refresh —
/// including the windowed "partial" — runs the OTP full waveform (~4 s on the
/// 7.5" V2). The panel family's 0.3 s partial spec requires one of these two
/// alternate waveform sources, selected at init by
/// [`Epd7in5::new_fast_partial_with_timeout_async`]:
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FastPartialWaveform {
    /// Short LUTs loaded into the controller registers (PSR REG=1). Ported from
    /// GxEPD2's `GxEPD2_750_GDEY075T7` "experimental partial screen update LUTs
    /// with balanced charge". Panel-batch independent, but the frame counts are
    /// GxEPD2's empirical tuning, not a Good Display release.
    RegisterLut,
    /// The OTP fast-partial waveform that GDEY075T7-era glass ships at forced
    /// temperature index 0x6E (CCSET TSFIX + TSSET). No register LUTs — the
    /// factory waveform — but only works on panel batches whose OTP actually
    /// contains it. GxEPD2 default for current batches.
    OtpForcedTemperature,
}

/// Register-LUT length the UC8179 expects for commands 0x20–0x25. GxEPD2 sends
/// 6 meaningful bytes and zero-pads the rest; the pad is part of the register.
const FAST_PARTIAL_LUT_LEN: usize = 42;

/// One fast-partial LUT: a single 6-byte group `[levels, T1, T2, T3, T4,
/// repeat]`, zero-padded. Frame counts are GxEPD2's: T1=30 charge-balance
/// pre-phase, T2=5 extension, T3=30 color-change phase, T4=5 extension,
/// repeated once. `levels` packs four 2-bit drive levels for the four phases.
const fn fast_partial_lut(levels: u8) -> [u8; FAST_PARTIAL_LUT_LEN] {
    let mut lut = [0u8; FAST_PARTIAL_LUT_LEN];
    lut[0] = levels;
    lut[1] = 30;
    lut[2] = 5;
    lut[3] = 30;
    lut[4] = 5;
    lut[5] = 1;
    lut
}

/// LUTC (0x20), VCOM: no drive.
const FAST_PARTIAL_LUT_VCOM: [u8; FAST_PARTIAL_LUT_LEN] = fast_partial_lut(0x00);
/// LUTWW (0x21), white→white: no drive.
const FAST_PARTIAL_LUT_WHITE_TO_WHITE: [u8; FAST_PARTIAL_LUT_LEN] = fast_partial_lut(0x00);
/// LUTKW (0x22), black→white: `01 01 10 10` — GxEPD2's "more white" variant.
const FAST_PARTIAL_LUT_BLACK_TO_WHITE: [u8; FAST_PARTIAL_LUT_LEN] = fast_partial_lut(0x5A);
/// LUTWK (0x23), white→black: `10 00 01 00`.
const FAST_PARTIAL_LUT_WHITE_TO_BLACK: [u8; FAST_PARTIAL_LUT_LEN] = fast_partial_lut(0x84);
/// LUTKK (0x24), black→black: no drive.
const FAST_PARTIAL_LUT_BLACK_TO_BLACK: [u8; FAST_PARTIAL_LUT_LEN] = fast_partial_lut(0x00);
/// LUTBD (0x25), border: no drive — the border keeps whatever the last full
/// refresh put there (this fork drives it black via CDI BDV, see `init`).
const FAST_PARTIAL_LUT_BORDER: [u8; FAST_PARTIAL_LUT_LEN] = fast_partial_lut(0x00);

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
        assert!(
            width % 8 == 0,
            "epd7in5_v2: partial width must be multiple of 8"
        );
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
        assert!(
            width % 8 == 0,
            "epd7in5_v2: partial width must be multiple of 8"
        );
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
        assert!(
            width % 8 == 0,
            "epd7in5_v2: partial width must be multiple of 8"
        );
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

// ─── Async, cooperatively-cancellable variants ─────────────────────────────
//
// Parallel to the blocking `*_with_timeout` family above, but driven by
// `embedded_hal_async`: SPI command/data writes and the inter-poll delay
// `.await`, so a single-core embassy cooperative executor runs other tasks
// while a refresh is in flight (SPI writes) or while BUSY is asserted. The
// blocking methods above are untouched; both paths coexist during the firmware
// transition.
//
// Cancellation is COOPERATIVE. The three long-running methods take a
// `should_cancel: &mut dyn FnMut() -> bool` (object-safe; the firmware can pass
// a closure that polls an `AtomicBool`/signal). The hook is checked at every
// BUSY poll, between framebuffer SPI chunks, and immediately before the
// `DisplayRefresh` (0x12) trigger. When it fires, the method STOPS — issues no
// further SPI — and returns `Ok(RefreshOutcome::Cancelled)`. Dropping the future
// mid-`.await` is NOT the intended cancel mechanism: it would strand the UC8179
// mid-transaction. See `RefreshOutcome` for the full contract.
impl<SPI, BUSY, DC, RST, DELAY> Epd7in5<SPI, BUSY, DC, RST, DELAY>
where
    SPI: AsyncSpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: AsyncDelayNs,
{
    /// Async construct + init with a BUSY-wait cap. Mirrors
    /// [`new_with_timeout`](Self::new_with_timeout). Plain async, NO cancel hook:
    /// init is short and must run to completion to leave the panel in a usable
    /// state.
    pub async fn new_with_timeout_async(
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
        epd.init_with_timeout_async(spi, delay, timeout_us).await?;

        Ok(epd)
    }

    /// Async bounded BUSY-poll WITH a cancel hook. Probes V2's `0x71 GetStatus`
    /// between polls and awaits the inter-poll delay so the executor runs other
    /// tasks during the wait.
    ///
    /// Because the caller of a refresh has already issued `0x12 DisplayRefresh`
    /// before awaiting idle, a `Cancelled` here means the panel is mid-refresh —
    /// that is expected; the caller is responsible for a hard reset. This method
    /// just reports `Cancelled` and issues no further SPI.
    pub async fn wait_until_idle_with_timeout_async(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        timeout_us: u32,
        should_cancel: &mut dyn FnMut() -> bool,
    ) -> Result<RefreshOutcome, BusyTimeoutError<SPI::Error>> {
        let cancelled = self
            .interface
            .wait_until_idle_with_cmd_timeout_async(
                spi,
                delay,
                IS_BUSY_LOW,
                Command::GetStatus,
                timeout_us,
                should_cancel,
            )
            .await?;
        Ok(if cancelled {
            RefreshOutcome::Cancelled
        } else {
            RefreshOutcome::Completed
        })
    }

    /// Async `update_and_display_frame` WITH a cancel hook. Drains to idle, then
    /// streams the framebuffer to DTM1 (raw) and DTM2 (inverted) in
    /// `ASYNC_WRITE_CHUNK`-byte bands, checking `should_cancel` after the
    /// preflight idle wait, at each DTM phase boundary, between bands, and once
    /// more immediately before issuing `DisplayRefresh` (0x12). The
    /// DisplayRefresh is fire-and-forget; the caller awaits idle separately via
    /// [`wait_until_idle_with_timeout_async`](Self::wait_until_idle_with_timeout_async).
    ///
    /// On cancel, returns `RefreshOutcome::Cancelled` having issued no
    /// `DisplayRefresh`, so the panel is left idle (no refresh started).
    pub async fn update_and_display_frame_with_timeout_async(
        &mut self,
        spi: &mut SPI,
        buffer: &[u8],
        delay: &mut DELAY,
        timeout_us: u32,
        should_cancel: &mut dyn FnMut() -> bool,
    ) -> Result<RefreshOutcome, BusyTimeoutError<SPI::Error>> {
        if self
            .wait_until_idle_with_timeout_async(spi, delay, timeout_us, should_cancel)
            .await?
            == RefreshOutcome::Cancelled
        {
            return Ok(RefreshOutcome::Cancelled);
        }
        // A cancel latched on entry (panel already idle, so the preflight wait
        // returned immediately without polling the hook) is caught here before
        // streaming the whole DTM1 band sequence.
        if should_cancel() {
            return Ok(RefreshOutcome::Cancelled);
        }

        // DTM1 = raw framebuffer, chunked with a cancel check per band.
        self.interface
            .cmd_async(spi, Command::DataStartTransmission1)
            .await?;
        for band in buffer.chunks(ASYNC_WRITE_CHUNK) {
            if should_cancel() {
                return Ok(RefreshOutcome::Cancelled);
            }
            self.interface.data_async(spi, band).await?;
        }

        // A cancel latched during the final DTM1 band is caught here before
        // streaming the whole DTM2 band sequence.
        if should_cancel() {
            return Ok(RefreshOutcome::Cancelled);
        }
        // DTM2 = bitwise-inverted framebuffer (see `update_frame` polarity note),
        // chunked with the same per-band cancel check.
        self.interface
            .cmd_async(spi, Command::DataStartTransmission2)
            .await?;
        for band in buffer.chunks(ASYNC_WRITE_CHUNK) {
            if should_cancel() {
                return Ok(RefreshOutcome::Cancelled);
            }
            self.interface.data_inverted_async(spi, band).await?;
        }

        if should_cancel() {
            return Ok(RefreshOutcome::Cancelled);
        }
        self.interface
            .cmd_async(spi, Command::DisplayRefresh)
            .await?;
        Ok(RefreshOutcome::Completed)
    }

    /// Async dual-buffer windowed partial refresh WITH a cancel hook. Mirrors
    /// [`update_partial_frame_dual_with_timeout`](Self::update_partial_frame_dual_with_timeout):
    /// re-establishes DTM1 (`old_buffer`) and DTM2 (`new_buffer`) under the
    /// differential `CDI=0xA9` waveform, both sent raw. `should_cancel` is
    /// checked at the pre-flight idle wait, on entry before switching to
    /// PartialIn mode, between framebuffer bands, immediately before
    /// `DisplayRefresh`, and at the post-refresh idle wait.
    ///
    /// On cancel BEFORE `DisplayRefresh`, returns `Cancelled` with no refresh
    /// started (panel left idle, though already in PartialIn mode — caller
    /// hard-resets). On cancel DURING the post-refresh wait, the panel is
    /// mid-refresh and the caller must hard-reset.
    #[allow(clippy::too_many_arguments)]
    pub async fn update_partial_frame_dual_with_timeout_async(
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
        should_cancel: &mut dyn FnMut() -> bool,
    ) -> Result<RefreshOutcome, BusyTimeoutError<SPI::Error>> {
        assert!(x % 8 == 0, "epd7in5_v2: partial x must be multiple of 8");
        assert!(
            width % 8 == 0,
            "epd7in5_v2: partial width must be multiple of 8"
        );
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

        if self
            .wait_until_idle_with_timeout_async(spi, delay, timeout_us, should_cancel)
            .await?
            == RefreshOutcome::Cancelled
        {
            return Ok(RefreshOutcome::Cancelled);
        }
        // A cancel latched on entry is caught here before the panel is switched
        // into PartialIn mode (which would otherwise require a hard reset).
        if should_cancel() {
            return Ok(RefreshOutcome::Cancelled);
        }

        self.interface
            .cmd_with_data_async(spi, Command::VcomAndDataIntervalSetting, &[0xA9, 0x07])
            .await?;
        self.interface.cmd_async(spi, Command::PartialIn).await?;

        let x_end = x + width - 1;
        let y_end = y + height - 1;
        self.interface
            .cmd_with_data_async(
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
            )
            .await?;

        // DTM1 = old (raw), chunked with a per-band cancel check.
        self.interface
            .cmd_async(spi, Command::DataStartTransmission1)
            .await?;
        for band in old_buffer.chunks(ASYNC_WRITE_CHUNK) {
            if should_cancel() {
                return Ok(RefreshOutcome::Cancelled);
            }
            self.interface.data_async(spi, band).await?;
        }

        // DTM2 = new (raw), chunked with a per-band cancel check.
        self.interface
            .cmd_async(spi, Command::DataStartTransmission2)
            .await?;
        for band in new_buffer.chunks(ASYNC_WRITE_CHUNK) {
            if should_cancel() {
                return Ok(RefreshOutcome::Cancelled);
            }
            self.interface.data_async(spi, band).await?;
        }

        if should_cancel() {
            return Ok(RefreshOutcome::Cancelled);
        }
        self.interface
            .cmd_async(spi, Command::DisplayRefresh)
            .await?;

        if self
            .wait_until_idle_with_timeout_async(spi, delay, timeout_us, should_cancel)
            .await?
            == RefreshOutcome::Cancelled
        {
            return Ok(RefreshOutcome::Cancelled);
        }

        self.interface.cmd_async(spi, Command::PartialOut).await?;
        self.interface
            .cmd_with_data_async(spi, Command::VcomAndDataIntervalSetting, &[0x20, 0x07])
            .await?;

        Ok(RefreshOutcome::Completed)
    }

    /// Async `sleep` with a BUSY-wait cap. Mirrors
    /// [`sleep_with_timeout`](Self::sleep_with_timeout). Plain async, NO cancel
    /// hook: the power-down sequence must run to completion.
    pub async fn sleep_with_timeout_async(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        timeout_us: u32,
    ) -> Result<(), BusyTimeoutError<SPI::Error>> {
        // No-op cancel hook: this path is non-cancellable by contract.
        let mut never_cancel = || false;
        self.interface
            .wait_until_idle_with_cmd_timeout_async(
                spi,
                delay,
                IS_BUSY_LOW,
                Command::GetStatus,
                timeout_us,
                &mut never_cancel,
            )
            .await?;
        self.interface.cmd_async(spi, Command::PowerOff).await?;
        self.interface
            .wait_until_idle_with_cmd_timeout_async(
                spi,
                delay,
                IS_BUSY_LOW,
                Command::GetStatus,
                timeout_us,
                &mut never_cancel,
            )
            .await?;
        self.interface
            .cmd_with_data_async(spi, Command::DeepSleep, &[0xA5])
            .await?;
        Ok(())
    }

    /// Async init with a BUSY-wait cap. Plain async, NO cancel hook — mirrors
    /// the blocking [`init_with_timeout`](Self::init_with_timeout).
    async fn init_with_timeout_async(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        timeout_us: u32,
    ) -> Result<(), BusyTimeoutError<SPI::Error>> {
        self.interface.reset_async(delay, 10_000, 2_000).await;

        self.interface
            .cmd_with_data_async(spi, Command::PowerSetting, &[0x07, 0x07, 0x3f, 0x3f])
            .await?;
        self.interface
            .cmd_with_data_async(spi, Command::BoosterSoftStart, &[0x17, 0x17, 0x28, 0x17])
            .await?;
        self.interface.cmd_async(spi, Command::PowerOn).await?;
        delay.delay_ms(100).await;

        let mut never_cancel = || false;
        self.interface
            .wait_until_idle_with_cmd_timeout_async(
                spi,
                delay,
                IS_BUSY_LOW,
                Command::GetStatus,
                timeout_us,
                &mut never_cancel,
            )
            .await?;

        self.interface
            .cmd_with_data_async(spi, Command::PanelSetting, &[0x1F])
            .await?;
        self.interface
            .cmd_with_data_async(spi, Command::TconResolution, &[0x03, 0x20, 0x01, 0xE0])
            .await?;
        self.interface
            .cmd_with_data_async(spi, Command::DualSpi, &[0x00])
            .await?;
        self.interface
            .cmd_with_data_async(spi, Command::VcomAndDataIntervalSetting, &[0x20, 0x07])
            .await?;
        self.interface
            .cmd_with_data_async(spi, Command::TconSetting, &[0x22])
            .await?;
        Ok(())
    }

    // ─── Fast partial-refresh mode ─────────────────────────────────────────
    //
    // Init + refresh for the panel family's 0.3 s-class partial waveform. The
    // init sequence is a faithful port of GxEPD2_750_GDEY075T7 (`_InitDisplay`
    // + `_Init_Part`), including its redundant PSR/CDI double-writes in the
    // RegisterLut branch and its config-then-PowerOn order (the plain init
    // above powers on first, per the Waveshare demo). Async-only: the firmware
    // has no blocking callers.

    /// Construct + init in fast-partial mode with a BUSY-wait cap. The session
    /// this creates is for windowed partial refreshes via
    /// [`update_fast_partial_frame_dual_with_timeout_async`](Self::update_fast_partial_frame_dual_with_timeout_async);
    /// to run a FULL refresh, first switch back with
    /// [`reinit_full_with_timeout_async`](Self::reinit_full_with_timeout_async)
    /// — a full-frame refresh under the short partial LUTs would not clean the
    /// panel properly.
    #[allow(clippy::too_many_arguments)]
    pub async fn new_fast_partial_with_timeout_async(
        spi: &mut SPI,
        busy: BUSY,
        dc: DC,
        rst: RST,
        delay: &mut DELAY,
        delay_us: Option<u32>,
        waveform: FastPartialWaveform,
        timeout_us: u32,
    ) -> Result<Self, BusyTimeoutError<SPI::Error>> {
        let interface = DisplayInterface::new(busy, dc, rst, delay_us);
        let color = DEFAULT_BACKGROUND_COLOR;

        let mut epd = Epd7in5 { interface, color };
        epd.init_fast_partial_with_timeout_async(spi, delay, waveform, timeout_us)
            .await?;

        Ok(epd)
    }

    /// Re-run the plain full-waveform init (reset + OTP LUT + this fork's CDI
    /// border setting) on an already-constructed instance. Switches a
    /// fast-partial session back to full-refresh mode; also usable as a
    /// mid-session recovery re-init.
    pub async fn reinit_full_with_timeout_async(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        timeout_us: u32,
    ) -> Result<(), BusyTimeoutError<SPI::Error>> {
        self.init_with_timeout_async(spi, delay, timeout_us).await
    }

    /// Dual-buffer windowed partial refresh for a session initialized via
    /// [`new_fast_partial_with_timeout_async`](Self::new_fast_partial_with_timeout_async).
    /// Identical wire flow to
    /// [`update_partial_frame_dual_with_timeout_async`](Self::update_partial_frame_dual_with_timeout_async)
    /// EXCEPT it never touches CDI: the fast-partial init already selected the
    /// waveform + data-polarity mode, and rewriting CDI per refresh (the
    /// OTP-path method swaps 0xA9/0x20) would clobber it. Both buffers are sent
    /// raw; same cancellation contract as the rest of the async family.
    #[allow(clippy::too_many_arguments)]
    pub async fn update_fast_partial_frame_dual_with_timeout_async(
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
        should_cancel: &mut dyn FnMut() -> bool,
    ) -> Result<RefreshOutcome, BusyTimeoutError<SPI::Error>> {
        assert!(x % 8 == 0, "epd7in5_v2: partial x must be multiple of 8");
        assert!(
            width % 8 == 0,
            "epd7in5_v2: partial width must be multiple of 8"
        );
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

        if self
            .wait_until_idle_with_timeout_async(spi, delay, timeout_us, should_cancel)
            .await?
            == RefreshOutcome::Cancelled
        {
            return Ok(RefreshOutcome::Cancelled);
        }
        // A cancel latched on entry is caught here before the panel is switched
        // into PartialIn mode (which would otherwise require a hard reset).
        if should_cancel() {
            return Ok(RefreshOutcome::Cancelled);
        }

        self.interface.cmd_async(spi, Command::PartialIn).await?;

        let x_end = x + width - 1;
        let y_end = y + height - 1;
        self.interface
            .cmd_with_data_async(
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
            )
            .await?;

        // DTM1 = old (raw), chunked with a per-band cancel check.
        self.interface
            .cmd_async(spi, Command::DataStartTransmission1)
            .await?;
        for band in old_buffer.chunks(ASYNC_WRITE_CHUNK) {
            if should_cancel() {
                return Ok(RefreshOutcome::Cancelled);
            }
            self.interface.data_async(spi, band).await?;
        }

        // DTM2 = new (raw), chunked with a per-band cancel check.
        self.interface
            .cmd_async(spi, Command::DataStartTransmission2)
            .await?;
        for band in new_buffer.chunks(ASYNC_WRITE_CHUNK) {
            if should_cancel() {
                return Ok(RefreshOutcome::Cancelled);
            }
            self.interface.data_async(spi, band).await?;
        }

        if should_cancel() {
            return Ok(RefreshOutcome::Cancelled);
        }
        self.interface
            .cmd_async(spi, Command::DisplayRefresh)
            .await?;

        if self
            .wait_until_idle_with_timeout_async(spi, delay, timeout_us, should_cancel)
            .await?
            == RefreshOutcome::Cancelled
        {
            return Ok(RefreshOutcome::Cancelled);
        }

        self.interface.cmd_async(spi, Command::PartialOut).await?;

        Ok(RefreshOutcome::Completed)
    }

    /// Fast-partial init: GxEPD2_750_GDEY075T7 `_InitDisplay` + `_Init_Part`,
    /// ported command-for-command (including the RegisterLut branch's PSR and
    /// CDI double-writes), then `_PowerOn`. Plain async, NO cancel hook — init
    /// must run to completion to leave the panel usable.
    async fn init_fast_partial_with_timeout_async(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        waveform: FastPartialWaveform,
        timeout_us: u32,
    ) -> Result<(), BusyTimeoutError<SPI::Error>> {
        self.interface.reset_async(delay, 10_000, 2_000).await;

        // _InitDisplay
        self.interface
            .cmd_with_data_async(spi, Command::PanelSetting, &[0x1F])
            .await?;
        // 5-byte PowerSetting (the plain init sends 4): the fifth byte is
        // VDHR=4.2V, per GxEPD2 "same POWER SETTING as from OTP".
        self.interface
            .cmd_with_data_async(spi, Command::PowerSetting, &[0x07, 0x07, 0x3F, 0x3F, 0x09])
            .await?;
        self.interface
            .cmd_with_data_async(spi, Command::BoosterSoftStart, &[0x17, 0x17, 0x28, 0x17])
            .await?;
        self.interface
            .cmd_with_data_async(spi, Command::TconResolution, &[0x03, 0x20, 0x01, 0xE0])
            .await?;
        self.interface
            .cmd_with_data_async(spi, Command::DualSpi, &[0x00])
            .await?;
        // CDI 0x29: LUTKW border + N2OCP (copy-new-to-old) + DDX=01. NOTE: not
        // this fork's black-border 0x20 — bench the border visually; the
        // partial path's border LUT is no-drive so the border should hold the
        // last full refresh's state either way.
        self.interface
            .cmd_with_data_async(spi, Command::VcomAndDataIntervalSetting, &[0x29, 0x07])
            .await?;
        self.interface
            .cmd_with_data_async(spi, Command::TconSetting, &[0x22])
            .await?;
        self.interface
            .cmd_with_data_async(spi, Command::PowerSaving, &[0x22])
            .await?;

        // _Init_Part
        match waveform {
            FastPartialWaveform::OtpForcedTemperature => {
                self.interface
                    .cmd_with_data_async(spi, Command::CascadeSetting, &[0x02])
                    .await?;
                self.interface
                    .cmd_with_data_async(spi, Command::ForceTemperature, &[0x6E])
                    .await?;
            }
            FastPartialWaveform::RegisterLut => {
                self.interface
                    .cmd_with_data_async(spi, Command::PanelSetting, &[0x3F])
                    .await?;
                // VCOM_DC -2.5V, "same value as in OTP" per GxEPD2.
                self.interface
                    .cmd_with_data_async(spi, Command::VcmDcSetting, &[0x30])
                    .await?;
                // CDI 0x39: LUTBD border + N2OCP + DDX=01.
                self.interface
                    .cmd_with_data_async(spi, Command::VcomAndDataIntervalSetting, &[0x39, 0x07])
                    .await?;
                self.interface
                    .cmd_with_data_async(spi, Command::LutForVcom, &FAST_PARTIAL_LUT_VCOM)
                    .await?;
                self.interface
                    .cmd_with_data_async(spi, Command::LutBlack, &FAST_PARTIAL_LUT_WHITE_TO_WHITE)
                    .await?;
                self.interface
                    .cmd_with_data_async(spi, Command::LutWhite, &FAST_PARTIAL_LUT_BLACK_TO_WHITE)
                    .await?;
                self.interface
                    .cmd_with_data_async(spi, Command::LutGray1, &FAST_PARTIAL_LUT_WHITE_TO_BLACK)
                    .await?;
                self.interface
                    .cmd_with_data_async(spi, Command::LutGray2, &FAST_PARTIAL_LUT_BLACK_TO_BLACK)
                    .await?;
                self.interface
                    .cmd_with_data_async(spi, Command::LutRed0, &FAST_PARTIAL_LUT_BORDER)
                    .await?;
            }
        }

        // _PowerOn: config first, THEN power on (the plain init above does the
        // reverse, per the Waveshare demo).
        self.interface.cmd_async(spi, Command::PowerOn).await?;
        delay.delay_ms(100).await;
        let mut never_cancel = || false;
        self.interface
            .wait_until_idle_with_cmd_timeout_async(
                spi,
                delay,
                IS_BUSY_LOW,
                Command::GetStatus,
                timeout_us,
                &mut never_cancel,
            )
            .await?;
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
