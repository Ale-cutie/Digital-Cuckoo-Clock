//! Wiring:
//!   SPI1 (LCD + Touch):
//!     D8  PC7 -> LCD RESET       D11 PA7 -> SPI1 MOSI
//!     D9  PC6 -> LCD DC          D12 PA6 -> SPI1 MISO
//!     D10 PC9 -> LCD CS          D13 PA5 -> SPI1 SCK
//!     D14 PB7 -> Touch CS        3V3      -> VCC, LED
//!     D15 PB6 -> Touch IRQ       GND      -> GND
//!
//!   SD card (shared SPI1 on display module):
//!     D13 PA5 -> SD_SCK          D11 PA7 -> SD_MOSI
//!     D12 PA6 -> SD_MISO         PB2 (CN10-22) -> SD_CS

#![no_std]
#![no_main]

use core::cell::RefCell;

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::dac::{DacChannel, Value};
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::rcc::mux;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::hz;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::{
    simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel},
    GeneralInstance4Channel,
};
use embassy_stm32::Config;
use embassy_stm32::{
    adc::{Adc, SampleTime},
    peripherals,
};
use embassy_time::{Delay, Duration, Instant, Timer};
use micromath::F32Ext;

use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::RefCellDevice;

use display_interface_spi::SPIInterface;
use ili9341::{DisplaySize240x320, Ili9341, Mode};

use embedded_graphics::{
    image::GetPixel,
    mono_font::{ascii::FONT_10X20, MonoFont},
    pixelcolor::{BinaryColor, Rgb565},
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
};

// ---------------------------------------------------------------------------
// Touch mapping
// ---------------------------------------------------------------------------

const RAW_X_MIN: i32 = 250;
const RAW_X_MAX: i32 = 3750;
const RAW_Y_MIN: i32 = 250;
const RAW_Y_MAX: i32 = 3750;

const SWAP_XY: bool = true;
const FLIP_X: bool = false; // inverted vs 0xA8 orientation
const FLIP_Y: bool = false; // inverted vs 0xA8 orientation

fn map_touch(rx: u16, ry: u16, width: i32, height: i32) -> (i32, i32) {
    let (ax, ay) = if SWAP_XY {
        (ry as i32, rx as i32)
    } else {
        (rx as i32, ry as i32)
    };

    let px = if FLIP_X {
        ((RAW_X_MAX - ax) * width) / (RAW_X_MAX - RAW_X_MIN)
    } else {
        ((ax - RAW_X_MIN) * width) / (RAW_X_MAX - RAW_X_MIN)
    };

    let py = if FLIP_Y {
        ((RAW_Y_MAX - ay) * height) / (RAW_Y_MAX - RAW_Y_MIN)
    } else {
        ((ay - RAW_Y_MIN) * height) / (RAW_Y_MAX - RAW_Y_MIN)
    };

    (px.clamp(0, width - 1), py.clamp(0, height - 1))
}

// ---------------------------------------------------------------------------
// Page state
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, PartialEq, Eq)]
enum Page {
    Clock,
    Menu,
    Music,
    SetTime,
    SetAlarm,
    AirQuality,
    AlarmRinging,
}

// ---------------------------------------------------------------------------
// Layout constants
// ---------------------------------------------------------------------------

const MENU_BTN_W: i32 = 70;
const MENU_BTN_H: i32 = 40;
const MENU_BTN_MARGIN: i32 = 8;

const BACK_BTN_X: i32 = 5;
const BACK_BTN_Y: i32 = 2;
const BACK_BTN_SZ: i32 = 30;

const GRID_BAR_H: i32 = 24;
const GRID_EDGE_M: i32 = 10;
const GRID_MARGIN: i32 = 12;

const VDDA_MV: u32 = 3_300;
const ADC_FULL_SCALE: u32 = 4_095;
const AIR_OVERSAMPLE_COUNT: u32 = 16;
const MQ135_R_LOAD_OHM: f32 = 10_000.0;
const MQ135_R0_OHM: f32 = 10_000.0;

struct AudioTone<'d> {
    ch: DacChannel<'d, embassy_stm32::mode::Blocking>,
    phase: f32,
}

impl<'d> AudioTone<'d> {
    fn new(ch: DacChannel<'d, embassy_stm32::mode::Blocking>) -> Self {
        Self { ch, phase: 0.0 }
    }

    // freq_hz == 0.0 → silence. Any positive value plays at that frequency.
    async fn run_for(&mut self, duration: Duration, freq_hz: f32) {
        if freq_hz <= 0.0 {
            self.ch.set(Value::Bit12Right(2048));
            Timer::after(duration).await;
            return;
        }

        const SAMPLE_RATE_HZ: f32 = 32_000.0;
        const DAC_CENTER: i32 = 2048;
        const AMPLITUDE: i32 = 2000;
        // sample_period in ticks — Instant arithmetic is sub-µs regardless of
        // the scheduler tick rate, so busy-waiting here gives exact 32 kHz.
        let sample_period = Duration::from_micros((1_000_000.0 / SAMPLE_RATE_HZ) as u64);
        let phase_step = 2.0 * core::f32::consts::PI * freq_hz / SAMPLE_RATE_HZ;
        let two_pi = 2.0 * core::f32::consts::PI;
        let end = Instant::now() + duration;
        let mut next = Instant::now();

        while Instant::now() < end {
            let v =
                (DAC_CENTER + (self.phase.sin() * AMPLITUDE as f32) as i32).clamp(0, 4095) as u16;
            self.ch.set(Value::Bit12Right(v));
            self.phase += phase_step;
            if self.phase >= two_pi {
                self.phase -= two_pi;
            }
            next += sample_period;
            // Busy-wait: guarantees exact sample timing without relying on the
            // async scheduler tick resolution (which may be too coarse for 31 µs).
            while Instant::now() < next {}
        }
    }
}

struct AlarmServo<'d, T: GeneralInstance4Channel> {
    ch: SimplePwmChannel<'d, T>,
    min_duty: u32,
    max_duty: u32,
    active: bool,
    elapsed_ms: u32,
}

impl<'d, T: GeneralInstance4Channel> AlarmServo<'d, T> {
    fn new(mut ch: SimplePwmChannel<'d, T>) -> Self {
        ch.enable();
        let max = ch.max_duty_cycle();
        let min_duty = max * 500 / 20_000;
        let max_duty = max * 1500 / 20_000;
        ch.set_duty_cycle(min_duty);
        Self {
            ch,
            min_duty,
            max_duty,
            active: false,
            elapsed_ms: 0,
        }
    }

    fn start(&mut self) {
        self.active = true;
        self.elapsed_ms = 0;
    }

    fn stop(&mut self) {
        self.active = false;
        self.elapsed_ms = 0;
        self.ch.set_duty_cycle(self.min_duty);
    }

    #[allow(dead_code)]
    fn is_active(&self) -> bool {
        self.active
    }

    /// True for the first 1 s of every 2 s window → clean 1 s on / 1 s off beep.
    fn audio_on(&self) -> bool {
        self.active && self.elapsed_ms % 2000 < 1000
    }

    fn tick(&mut self, dt_ms: u32) -> bool {
        if !self.active {
            return false;
        }
        self.elapsed_ms = self.elapsed_ms.saturating_add(dt_ms);
        let cycle_ms = self.elapsed_ms % 3000;
        let duty = if cycle_ms < 1000 {
            self.min_duty + (cycle_ms * (self.max_duty - self.min_duty)) / 1000
        } else if cycle_ms < 2000 {
            let progress = cycle_ms - 1000;
            self.max_duty - (progress * (self.max_duty - self.min_duty)) / 1000
        } else {
            self.min_duty // hold at 0° during pause
        };
        self.ch.set_duty_cycle(duty);
        if self.elapsed_ms >= 12000 {
            // 4 cycles × 3000 ms
            self.stop();
            true
        } else {
            false
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
struct AirReading {
    raw: u32,
    voltage_mv: u32,
    ppm: u32,
}

// ---------------------------------------------------------------------------
// SD card / music player
// ---------------------------------------------------------------------------

/// Maximum number of tracks we will discover on the card.
const MAX_TRACKS: usize = 16;
/// Maximum filename length (8.3 + null).
const MAX_NAME: usize = 13;

/// A tiny fixed-size string for a FAT 8.3 filename.
#[derive(Copy, Clone)]
struct TrackName {
    buf: [u8; MAX_NAME],
    len: usize,
}

impl TrackName {
    fn new() -> Self {
        Self {
            buf: [0; MAX_NAME],
            len: 0,
        }
    }

    fn from_bytes(src: &[u8]) -> Self {
        let mut t = Self::new();
        let n = src.len().min(MAX_NAME - 1);
        t.buf[..n].copy_from_slice(&src[..n]);
        t.len = n;
        t
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("???")
    }
}

/// Minimal SPI-mode SD card driver.
/// Holds the raw blocking SPI + CS pin so we can control CS independently —
/// the SD spec requires ≥74 clocks with CS HIGH before the first CMD0, which
/// is impossible if CS is automatically toggled by SpiDevice::write().
struct SdCard<'a, 'd, M: embassy_stm32::mode::Mode, CM: embassy_stm32::spi::mode::CommunicationMode>
{
    spi_bus: &'a RefCell<embassy_stm32::spi::Spi<'d, M, CM>>,
    cs: embassy_stm32::gpio::Output<'d>,
    block_addressing: bool,
}

impl<'a, 'd, M: embassy_stm32::mode::Mode, CM: embassy_stm32::spi::mode::CommunicationMode>
    SdCard<'a, 'd, M, CM>
{
    /// Initialise the card into SPI mode. Returns `true` on success.
    fn init(&mut self) -> bool {
        // >=74 clocks (10 bytes = 80 clocks) with CS HIGH so the card
        // can power up and enter SPI mode. Send extra to be safe.
        self.cs.set_high();
        self.xfer_ff(80);

        // Sanity check: with SD CS deasserted, MISO must be 0xFF (pulled up).
        // If it reads 0x00 here, another device on the bus is actively driving
        // the line low — the SD card will never be reachable in software.
        {
            let mut b = [0xFFu8];
            let _ = self.spi_bus.borrow_mut().blocking_transfer_in_place(&mut b);
            if b[0] != 0xFF {
                let _ = b[0];
            }
        }

        // CMD0 - GO_IDLE_STATE (expected R1 = 0x01).
        let mut r0 = 0xFFu8;
        for _ in 0..80 {
            r0 = self.cmd(0, 0, 0x95);
            if r0 == 0x01 || r0 == 0x00 {
                break;
            }
            self.cs.set_high();
            self.xfer_ff(8);
        }
        if r0 != 0x01 && r0 != 0x00 {
            self.cs.set_high();
            return false;
        }
        self.cs.set_high();
        self.xfer_ff(1);

        if r0 == 0x00 {
            // Card is already operational (stayed powered through MCU reset).
            // Sending CMD8/ACMD41 to a running card can confuse its state machine,
            // so skip straight to CMD13 (status) + CMD58 (OCR) and return ready.

            // CMD13 - SEND_STATUS: reveals locked/error bits in R2.
            let r13 = self.cmd(13, 0, 0xFF);
            let mut r2b = [0xFFu8];
            self.spi_read(&mut r2b);
            self.cs.set_high();
            self.xfer_ff(1);
            let _ = (r13, r2b);

            // CMD58 - READ_OCR to determine block addressing.
            let r58 = self.cmd(58, 0, 0xFF);
            let mut ocr = [0xFFu8; 4];
            self.spi_read(&mut ocr);
            self.cs.set_high();
            self.xfer_ff(1);
            let _ = r58;
            if ocr != [0x00; 4] && ocr != [0xFF; 4] {
                self.block_addressing = (ocr[0] & 0x40) != 0;
            }
            return true;
        }

        // --- Normal init path (CMD0 returned 0x01 = card properly idle) ---

        // CMD8 - SEND_IF_COND (0x01AA check pattern, CRC 0x87).
        let r8 = self.cmd(8, 0x0000_01AA, 0x87);
        let mut r7 = [0xFFu8; 4];
        self.spi_read(&mut r7);
        let sd_v2 = r8 == 0x01 || r8 == 0x00;
        self.cs.set_high();
        self.xfer_ff(1);

        // ACMD41 - SD_SEND_OP_COND (poll until idle bit clears).
        let hcs = if sd_v2 { 0x4000_0000u32 } else { 0 };
        let mut ready = false;
        for _ in 0..2000u32 {
            if self.acmd(41, hcs) == 0x00 {
                ready = true;
                break;
            }
            self.cs.set_high();
            self.xfer_ff(1);
        }
        self.cs.set_high();
        self.xfer_ff(1);
        if !ready {
            for _ in 0..2000u32 {
                if self.cmd(1, 0, 0xFF) == 0x00 {
                    ready = true;
                    break;
                }
                self.cs.set_high();
                self.xfer_ff(1);
            }
            self.cs.set_high();
            self.xfer_ff(1);
            if !ready {
                return false;
            }
        }

        // CMD58 - READ_OCR.
        let r58 = self.cmd(58, 0, 0xFF);
        let mut ocr = [0xFFu8; 4];
        self.spi_read(&mut ocr);
        self.cs.set_high();
        self.xfer_ff(1);
        let _ = r58;
        let ocr_valid = ocr != [0x00; 4] && ocr != [0xFF; 4];
        if ocr_valid {
            self.block_addressing = sd_v2 && (ocr[0] & 0x40) != 0;
        }

        if self.block_addressing {
            true
        } else {
            // CMD16 - SET_BLOCKLEN 512.
            let r16 = self.cmd(16, 512, 0xFF);
            self.cs.set_high();
            self.xfer_ff(1);
            r16 == 0x00
        }
    }
    /// Read one 512-byte sector. Returns `true` on success.
    fn read_sector(&mut self, lba: u32, buf: &mut [u8; 512]) -> bool {
        if self.read_sector_addr(lba, self.block_addressing, buf) {
            return true;
        }

        let fallback_block_addressing = !self.block_addressing;
        if self.read_sector_addr(lba, fallback_block_addressing, buf) {
            self.block_addressing = fallback_block_addressing;
            return true;
        }

        false
    }

    fn read_sector_addr(&mut self, lba: u32, block_addressing: bool, buf: &mut [u8; 512]) -> bool {
        let addr = if block_addressing {
            lba
        } else {
            lba.saturating_mul(512)
        };
        let r = self.cmd(17, addr, 0xFF);
        if r != 0x00 {
            self.cs.set_high();
            self.xfer_ff(1);
            return false;
        }

        let mut token = 0xFFu8;
        for _ in 0..65535u32 {
            let mut b = [0xFFu8];
            self.spi_read(&mut b);
            if b[0] != 0xFF {
                token = b[0];
                break;
            }
        }
        if token != 0xFE {
            self.cs.set_high();
            self.xfer_ff(1);
            return false;
        }

        self.spi_read(buf);
        let mut crc = [0xFFu8; 2];
        self.spi_read(&mut crc);

        self.cs.set_high();
        self.xfer_ff(1);
        true
    }
    /// Send `n` 0xFF bytes with CS in its current state (used as clock pulses).
    fn xfer_ff(&mut self, n: usize) {
        for _ in 0..n {
            let mut b = [0xFFu8];
            let _ = self.spi_bus.borrow_mut().blocking_transfer_in_place(&mut b);
        }
    }

    /// Read `buf.len()` bytes by clocking out 0xFF bytes.
    fn spi_read(&mut self, buf: &mut [u8]) {
        for b in buf.iter_mut() {
            let mut tmp = [0xFFu8];
            let _ = self
                .spi_bus
                .borrow_mut()
                .blocking_transfer_in_place(&mut tmp);
            *b = tmp[0];
        }
    }

    /// Send a 6-byte SD command with CS asserted, return the R1 byte.
    /// CS is left LOW after this returns; caller must set CS high when done.
    fn cmd(&mut self, index: u8, arg: u32, crc: u8) -> u8 {
        // One guard byte with CS high, then assert CS.
        self.cs.set_high();
        self.xfer_ff(1);
        self.cs.set_low();

        // Send the 6-byte command frame all at once.
        let mut frame = [
            0x40 | index,
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
            crc,
        ];
        let _ = self
            .spi_bus
            .borrow_mut()
            .blocking_transfer_in_place(&mut frame);

        // NCR: poll for a valid R1 (MSB must be 0).
        for _ in 0..64 {
            let mut r = [0xFFu8];
            let _ = self.spi_bus.borrow_mut().blocking_transfer_in_place(&mut r);
            if r[0] & 0x80 == 0 {
                return r[0];
            }
        }
        0xFF // timeout
    }

    fn acmd(&mut self, index: u8, arg: u32) -> u8 {
        // CMD55 prefix
        self.cmd(55, 0, 0xFF);
        self.cs.set_high();
        self.xfer_ff(1);
        // Then the application command
        self.cmd(index, arg, 0xFF)
    }
}

/// Very small FAT16/FAT32 root-directory scanner.
/// Finds up to MAX_TRACKS files whose names end in `.WAV` or `.MP3` (case-insensitive)
/// and returns (count, names, first-cluster-of-each-file).
fn scan_audio_files<
    'a,
    'd,
    M: embassy_stm32::mode::Mode,
    CM: embassy_stm32::spi::mode::CommunicationMode,
>(
    sd: &mut SdCard<'a, 'd, M, CM>,
    names: &mut [TrackName; MAX_TRACKS],
    clusters: &mut [u32; MAX_TRACKS],
) -> usize {
    let mut sector = [0u8; 512];

    // ---- Read MBR / Boot Sector -----------------------------------------
    if !sd.read_sector(0, &mut sector) {
        return 0;
    }

    // If sector 0 is a boot sector (signature 0x55 0xAA at offset 510) use it
    // directly, otherwise read the first partition's start LBA.
    let fat_start_lba: u32 = if sector[510] == 0x55 && sector[511] == 0xAA {
        if sector[0] == 0xEB || sector[0] == 0xE9 {
            0 // sector 0 IS the VBR
        } else {
            // MBR: partition 0 LBA at offset 0x1BE + 8
            u32::from_le_bytes([
                sector[0x1BE + 8],
                sector[0x1BE + 9],
                sector[0x1BE + 10],
                sector[0x1BE + 11],
            ])
        }
    } else {
        0
    };

    if !sd.read_sector(fat_start_lba, &mut sector) {
        return 0;
    }

    // ---- Parse BPB (BIOS Parameter Block) --------------------------------
    let bytes_per_sector = u16::from_le_bytes([sector[11], sector[12]]) as u32;
    let sectors_per_cluster = sector[13] as u32;
    let reserved_sectors = u16::from_le_bytes([sector[14], sector[15]]) as u32;
    let num_fats = sector[16] as u32;
    let root_entry_count = u16::from_le_bytes([sector[17], sector[18]]) as u32;
    let fat_size16 = u16::from_le_bytes([sector[22], sector[23]]) as u32;
    let fat_size32 = u32::from_le_bytes([sector[36], sector[37], sector[38], sector[39]]);
    let fat_size = if fat_size16 != 0 {
        fat_size16
    } else {
        fat_size32
    };
    let root_cluster_fat32 = u32::from_le_bytes([sector[44], sector[45], sector[46], sector[47]]);

    let is_fat32 = fat_size16 == 0;

    // Sector addresses.
    let fat_lba = fat_start_lba + reserved_sectors;
    let data_lba = fat_lba + num_fats * fat_size;
    let root_lba = if is_fat32 {
        data_lba + (root_cluster_fat32 - 2) * sectors_per_cluster
    } else {
        data_lba // root dir comes before data area in FAT16
    };

    // In FAT16 root dir occupies root_entry_count * 32 bytes starting at root_lba.
    // In FAT32 it is a regular cluster chain starting at root_cluster_fat32.
    let root_sectors = if is_fat32 {
        sectors_per_cluster
    } else {
        (root_entry_count * 32 + bytes_per_sector - 1) / bytes_per_sector
    };

    let _ = (data_lba, bytes_per_sector); // suppress unused warnings

    // ---- Walk root directory entries ------------------------------------
    let mut count = 0usize;

    'outer: for s in 0..root_sectors {
        if !sd.read_sector(root_lba + s, &mut sector) {
            break;
        }

        for e in 0..16usize {
            // 16 entries × 32 bytes = 512
            let off = e * 32;
            let first_byte = sector[off];
            if first_byte == 0x00 {
                break 'outer;
            } // end of directory
            if first_byte == 0xE5 {
                continue;
            } // deleted entry

            let attr = sector[off + 11];
            if attr & 0x0F == 0x0F {
                continue;
            } // LFN entry
            if attr & 0x10 != 0 {
                continue;
            } // directory

            // 8.3 name: bytes 0-7 = name, 8-10 = extension
            let ext = &sector[off + 8..off + 11];
            let is_wav = (ext[0] == b'W' || ext[0] == b'w')
                && (ext[1] == b'A' || ext[1] == b'a')
                && (ext[2] == b'V' || ext[2] == b'v');
            let is_mp3 = (ext[0] == b'M' || ext[0] == b'm')
                && (ext[1] == b'P' || ext[1] == b'p')
                && (ext[2] == b'3');
            if !is_wav && !is_mp3 {
                continue;
            }

            // Build display name "FILENAME.WAV"
            let mut name_buf = [b' '; MAX_NAME];
            let mut nlen = 0usize;
            for i in 0..8 {
                if sector[off + i] == b' ' {
                    break;
                }
                name_buf[nlen] = sector[off + i];
                nlen += 1;
            }
            name_buf[nlen] = b'.';
            nlen += 1;
            name_buf[nlen] = ext[0];
            nlen += 1;
            name_buf[nlen] = ext[1];
            nlen += 1;
            name_buf[nlen] = ext[2];
            nlen += 1;

            // First cluster (high word only present in FAT32).
            let clus_hi = u16::from_le_bytes([sector[off + 20], sector[off + 21]]) as u32;
            let clus_lo = u16::from_le_bytes([sector[off + 26], sector[off + 27]]) as u32;
            let cluster = (clus_hi << 16) | clus_lo;

            names[count] = TrackName::from_bytes(&name_buf[..nlen]);
            clusters[count] = cluster;
            count += 1;
            if count >= MAX_TRACKS {
                break 'outer;
            }
        }
    }

    count
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = Config::default();
    config.rcc.mux.dac1sel = mux::Dacsel::LSI;
    let p = embassy_stm32::init(config);

    // ---- SPI1: LCD + Touch + SD ------------------------------------------
    let mut spi1_cfg = SpiConfig::default();
    spi1_cfg.frequency = Hertz(400_000);
    spi1_cfg.miso_pull = Pull::Up;
    let spi1 = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi1_cfg);
    let spi1_bus = RefCell::new(spi1);

    let lcd_cs = Output::new(p.PC9, Level::High, Speed::VeryHigh);
    let lcd_dc = Output::new(p.PC6, Level::Low, Speed::VeryHigh);
    let lcd_rst = Output::new(p.PC7, Level::High, Speed::Low);
    let touch_cs = Output::new(p.PB7, Level::High, Speed::Low);
    let _touch_irq = Input::new(p.PB6, Pull::Up);
    let sd_cs_pin = Output::new(p.PB2, Level::High, Speed::VeryHigh);

    // ---- SD card: shared SPI1 bus with separate SD CS --------------------
    // PB2 / CN10-22; CN10-16 is NC on this board (UM3062).
    let mut sd = SdCard {
        spi_bus: &spi1_bus,
        cs: sd_cs_pin,
        block_addressing: true,
    };

    // Init LCD and touch FIRST so ILI9341/XPT2046 SDO lines are in a known
    // quiescent state before SD clocking begins — prevents spurious MISO drive.
    let lcd_dev = RefCellDevice::new(&spi1_bus, lcd_cs, Delay).unwrap();
    let di = SPIInterface::new(lcd_dev, lcd_dc);
    let mut delay = Delay;
    let mut display = Ili9341::new(di, lcd_rst, &mut delay, LandscapeFlip, DisplaySize240x320)
        .expect("ILI9341 init failed");
    let mut touch_dev = RefCellDevice::new(&spi1_bus, touch_cs, Delay).unwrap();

    Timer::after(Duration::from_millis(500)).await;

    let mut track_names = [TrackName::new(); MAX_TRACKS];
    let mut track_clusters = [0u32; MAX_TRACKS];
    let track_count;
    let sd_ok;

    if sd.init() {
        sd_ok = true;
        track_count = scan_audio_files(&mut sd, &mut track_names, &mut track_clusters);
    } else {
        sd_ok = false;
        track_count = 0;
    }

    // ---- Audio, servo, and air-quality peripherals ----------------------
    let mut audio = AudioTone::new(DacChannel::new_blocking(p.DAC1, p.PA4));
    let mut amp_sw = Output::new(p.PA1, Level::Low, Speed::Low);
    let servo_pin = PwmPin::new(p.PA8, OutputType::PushPull);
    let mut servo_pwm = SimplePwm::new(
        p.TIM1,
        Some(servo_pin),
        None,
        None,
        None,
        hz(50),
        Default::default(),
    );
    let mut servo = AlarmServo::new(servo_pwm.ch1());

    let mut air_adc = Adc::new(p.ADC1);
    let mut air_pin = p.PA0;
    let mut air_tick_ms = 0u32;

    // ---- Display setup ---------------------------------------------------
    let background = Rgb565::new(0x02, 0x0F, 0x07);
    let clock_text = Rgb565::new(0xDF, 0xF3, 0xFF);

    const CLOCK_SCALE: u32 = 5;
    const FONT: &MonoFont = &FONT_10X20;

    let display_size = display.size();
    let w = display_size.width as i32;
    let h = display_size.height as i32;

    // ---- State -----------------------------------------------------------
    let mut page = Page::Clock;
    let mut hours = 12u8;
    let mut minutes = 0u8;
    let mut tick_ms = 0u32; // counts 100 ms ticks for the clock

    let initial_text = "00:00";
    let text_size = scaled_text_size(initial_text, FONT, CLOCK_SCALE);
    let time_origin = Point::new(
        (w - text_size.width as i32) / 2,
        (h - text_size.height as i32) / 2,
    );

    let mut last_hours = 255u8;
    let mut last_minutes = 255u8;
    let mut time_buf = [0u8; 5];
    let mut last_time_buf = [0xFF_u8; 5];

    let menu_btn_origin = Point::new(
        w - (MENU_BTN_W + MENU_BTN_MARGIN),
        h - (MENU_BTN_H + MENU_BTN_MARGIN),
    );
    let menu_btn_size = Size::new(MENU_BTN_W as u32, MENU_BTN_H as u32);

    // Music player state
    let mut current_track: usize = 0;
    let mut music_playing: bool = false;
    let mut last_air_reading: Option<AirReading> = None;

    // Alarm state
    let mut alarm_hours: u8 = 7;
    let mut alarm_minutes: u8 = 0;
    let mut alarm_enabled: bool = false;
    let mut alarm_fired: bool = false;

    // Temporary edit state for Set Time / Set Alarm pages
    let mut edit_hours: u8 = 0;
    let mut edit_minutes: u8 = 0;

    // Initial draw
    display.clear(background).unwrap();
    draw_menu_button(
        &mut display,
        menu_btn_origin,
        menu_btn_size,
        FONT,
        background,
    )
    .unwrap();

    // Debounce
    const DEBOUNCE_MS: u64 = 300;
    let mut last_touch_ms: u64 = 0;

    // -----------------------------------------------------------------------
    // Main loop
    // -----------------------------------------------------------------------

    loop {
        // ---- Clock tick (100 ms) -----------------------------------------
        let music_audio_enabled = page == Page::Music && music_playing;
        let alarm_active = page == Page::AlarmRinging && servo.is_active();

        // Keep amp on for the whole alarm/music session — toggling it per-100 ms
        // triggers the PAM8403 soft-start ramp every beep, making the output dim.
        if alarm_active || music_audio_enabled {
            amp_sw.set_high();
        } else {
            amp_sw.set_low();
        }

        // Alarm tone: constant 880 Hz for 1 s on / 1 s off, in sync with servo elapsed time.
        let tone_hz: f32 = if alarm_active && servo.audio_on() {
            880.0
        } else if music_audio_enabled {
            440.0
        } else {
            0.0
        };
        audio.run_for(Duration::from_millis(100), tone_hz).await;
        tick_ms += 1;
        if tick_ms >= 600 {
            // 60 000 ms / 100 = 600 ticks per minute
            tick_ms = 0;
            minutes += 1;
            if minutes >= 60 {
                minutes = 0;
                hours = (hours + 1) % 24;
            }
        }

        servo.tick(100);

        air_tick_ms += 100;
        if air_tick_ms >= 2000 {
            air_tick_ms = 0;
            let air_reading = read_air_quality(&mut air_adc, &mut air_pin).await;
            if page == Page::AirQuality {
                if let Some(previous) = last_air_reading {
                    if previous != air_reading {
                        update_air_quality_values(
                            &mut display,
                            FONT,
                            background,
                            previous,
                            air_reading,
                        )
                        .ok();
                        last_air_reading = Some(air_reading);
                    }
                } else {
                    draw_air_quality_page(
                        &mut display,
                        display_size,
                        FONT,
                        background,
                        air_reading,
                    )
                    .ok();
                    last_air_reading = Some(air_reading);
                }
            }
        }

        // ---- Clock display update ----------------------------------------
        if page == Page::Clock && (hours != last_hours || minutes != last_minutes) {
            format_time(hours, minutes, &mut time_buf);
            for idx in 0..time_buf.len() {
                if time_buf[idx] != last_time_buf[idx] {
                    let char_origin = Point::new(
                        time_origin.x
                            + idx as i32
                                * (FONT.character_size.width + FONT.character_spacing) as i32
                                * CLOCK_SCALE as i32,
                        time_origin.y,
                    );
                    draw_scaled_char(
                        &mut display,
                        FONT,
                        time_buf[idx] as char,
                        char_origin,
                        CLOCK_SCALE,
                        clock_text,
                        background,
                    )
                    .unwrap();
                }
            }
            last_hours = hours;
            last_minutes = minutes;
            last_time_buf = time_buf;
        }

        // ---- Touch polling -----------------------------------------------
        // Poll directly — IRQ pin (PB6) may not be wired on this module.
        if let Some((rx, ry)) = read_touch_avg(&mut touch_dev) {
            let now_ms = embassy_time::Instant::now().as_millis();
            if now_ms.saturating_sub(last_touch_ms) < DEBOUNCE_MS {
                Timer::after(Duration::from_millis(100)).await;
                continue;
            }
            last_touch_ms = now_ms;

            let (x, y) = map_touch(rx, ry, w, h);
            defmt::info!("touch ({}, {})", x, y);

            match page {
                // ---- Clock page -----------------------------------
                Page::Clock => {
                    if point_in_rect_padded(Point::new(x, y), menu_btn_origin, menu_btn_size, 10) {
                        defmt::info!("→ Menu");
                        display.clear(background).ok();
                        draw_menu_page(
                            &mut display,
                            display_size,
                            FONT,
                            hours,
                            minutes,
                            background,
                        )
                        .ok();
                        page = Page::Menu;
                    }
                }

                // ---- Menu page ------------------------------------
                Page::Menu => {
                    let back_origin = Point::new(BACK_BTN_X, BACK_BTN_Y);
                    let back_size = Size::new(BACK_BTN_SZ as u32, BACK_BTN_SZ as u32);

                    if point_in_rect_padded(Point::new(x, y), back_origin, back_size, 10) {
                        defmt::info!("→ Clock");
                        go_to_clock(
                            &mut display,
                            &mut page,
                            menu_btn_origin,
                            menu_btn_size,
                            FONT,
                            background,
                            &mut last_hours,
                            &mut last_minutes,
                            &mut last_time_buf,
                        );
                    } else if let Some((idx, gx, gy)) =
                        get_tapped_grid_button(Point::new(x, y), display_size)
                    {
                        let _ = (gx, gy);
                        // Grid order matches labels array in draw_menu_page.
                        // idx 0 = top-left  = Set Time
                        // idx 1 = top-right = Music         (was Set Alarm — swapped)
                        // idx 2 = bot-left  = Set Alarm     (was Music     — swapped)
                        // idx 3 = bot-right = Air Quality
                        match idx {
                            0 => {
                                // Set Time
                                edit_hours = hours;
                                edit_minutes = minutes;
                                display.clear(background).ok();
                                draw_time_edit_page(
                                    &mut display,
                                    display_size,
                                    FONT,
                                    background,
                                    edit_hours,
                                    edit_minutes,
                                    false,
                                )
                                .ok();
                                page = Page::SetTime;
                            }
                            1 => {
                                // Set Alarm
                                edit_hours = alarm_hours;
                                edit_minutes = alarm_minutes;
                                display.clear(background).ok();
                                draw_time_edit_page(
                                    &mut display,
                                    display_size,
                                    FONT,
                                    background,
                                    edit_hours,
                                    edit_minutes,
                                    true,
                                )
                                .ok();
                                page = Page::SetAlarm;
                            }
                            2 => {
                                // Music
                                defmt::info!("→ Music");
                                display.clear(background).ok();
                                draw_music_page(
                                    &mut display,
                                    display_size,
                                    FONT,
                                    background,
                                    sd_ok,
                                    &track_names,
                                    track_count,
                                    current_track,
                                    music_playing,
                                )
                                .ok();
                                page = Page::Music;
                            }
                            3 => {
                                defmt::info!("→ Air Quality");
                                let air_reading =
                                    read_air_quality(&mut air_adc, &mut air_pin).await;
                                display.clear(background).ok();
                                draw_air_quality_page(
                                    &mut display,
                                    display_size,
                                    FONT,
                                    background,
                                    air_reading,
                                )
                                .ok();
                                last_air_reading = Some(air_reading);
                                page = Page::AirQuality;
                            }
                            _ => {}
                        }
                    }
                }

                // ---- Music page -----------------------------------
                Page::Music => {
                    let back_origin = Point::new(BACK_BTN_X, BACK_BTN_Y);
                    let back_size = Size::new(BACK_BTN_SZ as u32, BACK_BTN_SZ as u32);

                    if point_in_rect_padded(Point::new(x, y), back_origin, back_size, 10) {
                        defmt::info!("Music → Clock");
                        music_playing = false;
                        go_to_clock(
                            &mut display,
                            &mut page,
                            menu_btn_origin,
                            menu_btn_size,
                            FONT,
                            background,
                            &mut last_hours,
                            &mut last_minutes,
                            &mut last_time_buf,
                        );
                    } else {
                        // Check the three music control buttons.
                        let hit = get_tapped_music_btn(Point::new(x, y), display_size);
                        match hit {
                            Some(MusicBtn::Prev) => {
                                if track_count > 0 {
                                    current_track = if current_track == 0 {
                                        track_count - 1
                                    } else {
                                        current_track - 1
                                    };
                                }
                            }
                            Some(MusicBtn::PlayPause) => {
                                music_playing = !music_playing;
                            }
                            Some(MusicBtn::Next) => {
                                if track_count > 0 {
                                    current_track = (current_track + 1) % track_count;
                                }
                            }
                            None => {}
                        }
                        // Redraw music page to reflect new state.
                        draw_music_page(
                            &mut display,
                            display_size,
                            FONT,
                            background,
                            sd_ok,
                            &track_names,
                            track_count,
                            current_track,
                            music_playing,
                        )
                        .ok();
                    }
                }
                // ---- Set Time page --------------------------------
                Page::SetTime => {
                    let back_origin = Point::new(BACK_BTN_X, BACK_BTN_Y);
                    let back_size = Size::new(BACK_BTN_SZ as u32, BACK_BTN_SZ as u32);
                    if point_in_rect_padded(Point::new(x, y), back_origin, back_size, 10) {
                        // Discard changes
                        go_to_clock(
                            &mut display,
                            &mut page,
                            menu_btn_origin,
                            menu_btn_size,
                            FONT,
                            background,
                            &mut last_hours,
                            &mut last_minutes,
                            &mut last_time_buf,
                        );
                    } else if let Some(btn) = get_tapped_time_btn(Point::new(x, y), display_size) {
                        match btn {
                            TimeBtn::HourUp => {
                                edit_hours = (edit_hours + 1) % 24;
                            }
                            TimeBtn::HourDown => {
                                edit_hours = if edit_hours == 0 { 23 } else { edit_hours - 1 };
                            }
                            TimeBtn::MinUp => {
                                edit_minutes = (edit_minutes + 1) % 60;
                            }
                            TimeBtn::MinDown => {
                                edit_minutes = if edit_minutes == 0 {
                                    59
                                } else {
                                    edit_minutes - 1
                                };
                            }
                            TimeBtn::Confirm => {
                                hours = edit_hours;
                                minutes = edit_minutes;
                                tick_ms = 0; // reset sub-minute counter so time doesn't jump
                                go_to_clock(
                                    &mut display,
                                    &mut page,
                                    menu_btn_origin,
                                    menu_btn_size,
                                    FONT,
                                    background,
                                    &mut last_hours,
                                    &mut last_minutes,
                                    &mut last_time_buf,
                                );
                                // Draw the new time immediately.
                                format_time(hours, minutes, &mut time_buf);
                                for idx in 0..time_buf.len() {
                                    let char_origin = Point::new(
                                        time_origin.x
                                            + idx as i32
                                                * (FONT.character_size.width
                                                    + FONT.character_spacing)
                                                    as i32
                                                * CLOCK_SCALE as i32,
                                        time_origin.y,
                                    );
                                    draw_scaled_char(
                                        &mut display,
                                        FONT,
                                        time_buf[idx] as char,
                                        char_origin,
                                        CLOCK_SCALE,
                                        clock_text,
                                        background,
                                    )
                                    .ok();
                                }
                                last_hours = hours;
                                last_minutes = minutes;
                                last_time_buf = time_buf;
                                // Don't fall through to draw_time_values — page is now Clock.
                            }
                        }
                        // Only redraw the values (not the whole page) for +/- taps.
                        if page == Page::SetTime {
                            draw_time_values(
                                &mut display,
                                display_size,
                                FONT,
                                background,
                                edit_hours,
                                edit_minutes,
                            )
                            .ok();
                        }
                    }
                }

                // ---- Set Alarm page --------------------------------
                Page::SetAlarm => {
                    let back_origin = Point::new(BACK_BTN_X, BACK_BTN_Y);
                    let back_size = Size::new(BACK_BTN_SZ as u32, BACK_BTN_SZ as u32);
                    if point_in_rect_padded(Point::new(x, y), back_origin, back_size, 10) {
                        go_to_clock(
                            &mut display,
                            &mut page,
                            menu_btn_origin,
                            menu_btn_size,
                            FONT,
                            background,
                            &mut last_hours,
                            &mut last_minutes,
                            &mut last_time_buf,
                        );
                    } else if let Some(btn) = get_tapped_time_btn(Point::new(x, y), display_size) {
                        match btn {
                            TimeBtn::HourUp => {
                                edit_hours = (edit_hours + 1) % 24;
                            }
                            TimeBtn::HourDown => {
                                edit_hours = if edit_hours == 0 { 23 } else { edit_hours - 1 };
                            }
                            TimeBtn::MinUp => {
                                edit_minutes = (edit_minutes + 1) % 60;
                            }
                            TimeBtn::MinDown => {
                                edit_minutes = if edit_minutes == 0 {
                                    59
                                } else {
                                    edit_minutes - 1
                                };
                            }
                            TimeBtn::Confirm => {
                                alarm_hours = edit_hours;
                                alarm_minutes = edit_minutes;
                                alarm_enabled = true;
                                alarm_fired = false;
                                defmt::info!("alarm set {}:{:02}", alarm_hours, alarm_minutes);
                                go_to_clock(
                                    &mut display,
                                    &mut page,
                                    menu_btn_origin,
                                    menu_btn_size,
                                    FONT,
                                    background,
                                    &mut last_hours,
                                    &mut last_minutes,
                                    &mut last_time_buf,
                                );
                            }
                        }
                        if page == Page::SetAlarm {
                            draw_time_values(
                                &mut display,
                                display_size,
                                FONT,
                                background,
                                edit_hours,
                                edit_minutes,
                            )
                            .ok();
                        }
                    }
                }

                Page::AirQuality => {
                    let back_origin = Point::new(BACK_BTN_X, BACK_BTN_Y);
                    let back_size = Size::new(BACK_BTN_SZ as u32, BACK_BTN_SZ as u32);
                    if point_in_rect_padded(Point::new(x, y), back_origin, back_size, 10) {
                        go_to_clock(
                            &mut display,
                            &mut page,
                            menu_btn_origin,
                            menu_btn_size,
                            FONT,
                            background,
                            &mut last_hours,
                            &mut last_minutes,
                            &mut last_time_buf,
                        );
                    }
                }

                Page::AlarmRinging => {
                    let p = Point::new(x, y);
                    let (stop_origin, stop_size) = alarm_stop_hitbox();
                    let visible_stop_origin = alarm_stop_button(display_size);
                    let visible_stop_size = Size::new(160, 44);
                    if point_in_rect(p, stop_origin, stop_size)
                        || point_in_rect(p, visible_stop_origin, visible_stop_size)
                    {
                        defmt::info!("alarm stopped by user");
                        servo.stop();
                        music_playing = false;
                        alarm_fired = true;
                        go_to_clock(
                            &mut display,
                            &mut page,
                            menu_btn_origin,
                            menu_btn_size,
                            FONT,
                            background,
                            &mut last_hours,
                            &mut last_minutes,
                            &mut last_time_buf,
                        );
                    }
                }
            } // end match page
        } // end if let Some(rx,ry)

        // ---- Alarm check -------------------------------------------------
        if alarm_enabled
            && !alarm_fired
            && page == Page::Clock
            && hours == alarm_hours
            && minutes == alarm_minutes
        {
            alarm_fired = true;
            defmt::info!("ALARM! {}:{:02}", alarm_hours, alarm_minutes);
            page = Page::AlarmRinging;
            music_playing = false;
            current_track = find_track_005(&track_names, track_count).unwrap_or(current_track);
            servo.start();
            draw_alarm_page(
                &mut display,
                display_size,
                FONT,
                background,
                track_names[current_track].as_str(),
            )
            .ok();
        }

        if page == Page::AlarmRinging && (hours != alarm_hours || minutes != alarm_minutes) {
            servo.stop();
            music_playing = false;
            alarm_fired = false;
            go_to_clock(
                &mut display,
                &mut page,
                menu_btn_origin,
                menu_btn_size,
                FONT,
                background,
                &mut last_hours,
                &mut last_minutes,
                &mut last_time_buf,
            );
        } else if alarm_enabled
            && alarm_fired
            && page == Page::Clock
            && (hours != alarm_hours || minutes != alarm_minutes)
        {
            alarm_fired = false;
        }
    } // end loop
} // end main

// ---------------------------------------------------------------------------
// Time / Alarm editor
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, PartialEq, Eq)]
enum TimeBtn {
    HourUp,
    HourDown,
    MinUp,
    MinDown,
    Confirm,
}

/// Layout for the time-editor buttons.
/// Returns (col0_x, col1_x, row_up_y, row_val_y, row_dn_y, confirm_y, btn_w, btn_h, confirm_h)
fn time_edit_layout(display_size: Size) -> (i32, i32, i32, i32, i32, i32, i32, i32, i32) {
    let w = display_size.width as i32;
    let btn_w = (w - 60) / 2;
    let btn_h = 34i32;
    let confirm_h = 44i32;
    let col0_x = 10i32;
    let col1_x = w - 10 - btn_w;
    let row_up = 42i32;
    let row_val = 96i32;
    let row_dn = 176i32;
    let confirm = 106i32;
    (
        col0_x, col1_x, row_up, row_val, row_dn, confirm, btn_w, btn_h, confirm_h,
    )
}

fn get_tapped_time_btn(p: Point, display_size: Size) -> Option<TimeBtn> {
    let (c0, c1, ru, _rv, rd, cy, bw, bh, confirm_h) = time_edit_layout(display_size);
    let sz = Size::new(bw as u32, bh as u32);

    // Match the drawn buttons exactly.
    if point_in_rect(p, Point::new(c0, ru), sz) {
        return Some(TimeBtn::HourUp);
    }
    if point_in_rect(p, Point::new(c0, rd), sz) {
        return Some(TimeBtn::MinUp);
    }
    if point_in_rect(p, Point::new(c1, ru), sz) {
        return Some(TimeBtn::HourDown);
    }
    if point_in_rect(p, Point::new(c1, rd), sz) {
        return Some(TimeBtn::MinDown);
    }

    let confirm_w = 76i32;
    if point_in_rect(
        p,
        Point::new(display_size.width as i32 - confirm_w - 4, cy),
        Size::new(confirm_w as u32, confirm_h as u32),
    ) {
        return Some(TimeBtn::Confirm);
    }

    None
}

impl TimeBtn {
    #[allow(dead_code)]
    fn as_str(self) -> &'static str {
        match self {
            TimeBtn::HourUp => "HourUp",
            TimeBtn::HourDown => "HourDown",
            TimeBtn::MinUp => "MinUp",
            TimeBtn::MinDown => "MinDown",
            TimeBtn::Confirm => "Confirm",
        }
    }
}

/// Redraw only the hour and minute values on the time-edit page.
/// Much faster than a full page redraw — no flicker, no full clear.
fn draw_time_values<D>(
    display: &mut D,
    display_size: Size,
    font: &MonoFont<'_>,
    background: Rgb565,
    h_val: u8,
    m_val: u8,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let (_c0, _c1, _ru, rv, _rd, _cy, _bw, _bh, _confirm_h) = time_edit_layout(display_size);
    let val_fg = Rgb565::new(0xDF, 0xF3, 0xFF);

    let value_area_w = display_size.width;
    let value_scale = 3;
    let mut time_buf = [0u8; 5];
    format_time(h_val, m_val, &mut time_buf);
    let time_str = core::str::from_utf8(&time_buf).unwrap_or("00:00");
    let ts = scaled_text_size(time_str, font, value_scale);

    display.fill_solid(
        &Rectangle::new(Point::new(0, rv - 10), Size::new(value_area_w, 76)),
        background,
    )?;
    draw_text_line(
        display,
        font,
        time_str,
        Point::new((value_area_w as i32 - ts.width as i32) / 2, rv - 2),
        value_scale,
        val_fg,
        background,
    )?;

    Ok(())
}

fn draw_time_edit_page<D>(
    display: &mut D,
    display_size: Size,
    font: &MonoFont<'_>,
    background: Rgb565,
    h_val: u8,
    m_val: u8,
    is_alarm: bool,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let w = display_size.width as i32;

    display.fill_solid(&Rectangle::new(Point::new(0, 0), display_size), background)?;

    // Top bar
    let bar_h = GRID_BAR_H as u32;
    display.fill_solid(
        &Rectangle::new(Point::new(0, 0), Size::new(display_size.width, bar_h)),
        Rgb565::new(0x02, 0x0F, 0x07),
    )?;

    // Back button
    display.fill_solid(
        &Rectangle::new(
            Point::new(BACK_BTN_X, BACK_BTN_Y),
            Size::new(BACK_BTN_SZ as u32, BACK_BTN_SZ as u32),
        ),
        Rgb565::new(0x00, 0x04, 0x02),
    )?;
    let arrow_color = Rgb565::new(0xFF, 0xFF, 0xFF);
    let ax = BACK_BTN_X + BACK_BTN_SZ / 2;
    let ay = BACK_BTN_Y + BACK_BTN_SZ / 2;
    let al = 6i32;
    Line::new(Point::new(ax + al, ay - al), Point::new(ax - al, ay))
        .into_styled(PrimitiveStyle::with_stroke(arrow_color, 2))
        .draw(display)?;
    Line::new(Point::new(ax + al, ay + al), Point::new(ax - al, ay))
        .into_styled(PrimitiveStyle::with_stroke(arrow_color, 2))
        .draw(display)?;

    // Title centred in top bar
    let title = if is_alarm { "SET ALARM" } else { "SET TIME" };
    let ts = scaled_text_size(title, font, 1);
    draw_text_line(
        display,
        font,
        title,
        Point::new(
            (w - ts.width as i32) / 2,
            (bar_h as i32 - ts.height as i32) / 2,
        ),
        1,
        Rgb565::new(0xDF, 0xF3, 0xFF),
        Rgb565::new(0x02, 0x0F, 0x07),
    )?;

    // Column headers
    let (c0, c1, ru, rv, rd, _cy, bw, bh, _confirm_h) = time_edit_layout(display_size);
    let btn_bg = Rgb565::new(0x00, 0x04, 0x02);
    let btn_fg = Rgb565::new(0xFF, 0xFF, 0xFF);
    let _ = rv;

    // Both columns draw '+' on top and '-' in the lower touch band.
    draw_btn_simple(display, font, c0, ru, "+", bw, bh, btn_bg, btn_fg)?; // HourUp   — top-left
    draw_btn_simple(display, font, c0, rd, "+", bw, bh, btn_bg, btn_fg)?; // MinUp
    draw_btn_simple(display, font, c1, ru, "-", bw, bh, btn_bg, btn_fg)?; // HourDown
    draw_btn_simple(display, font, c1, rd, "-", bw, bh, btn_bg, btn_fg)?; // MinDown  — bottom-right

    draw_time_values(display, display_size, font, background, h_val, m_val)?;

    // Confirm button
    let confirm_w = 120i32;
    let confirm_h_draw = 30i32;
    let confirm_x = (w - confirm_w) / 2;
    let confirm_y = 210i32;
    let confirm_label = "OK";
    display.fill_solid(
        &Rectangle::new(
            Point::new(confirm_x, confirm_y),
            Size::new(confirm_w as u32, confirm_h_draw as u32),
        ),
        Rgb565::new(0x00, 0x10, 0x06),
    )?;
    let cls = scaled_text_size(confirm_label, font, 1);
    draw_text_line(
        display,
        font,
        confirm_label,
        Point::new(
            confirm_x + (confirm_w - cls.width as i32) / 2,
            confirm_y + (confirm_h_draw - cls.height as i32) / 2,
        ),
        1,
        Rgb565::new(0xFF, 0xFF, 0xFF),
        Rgb565::new(0x00, 0x10, 0x06),
    )?;

    Ok(())
}

// ---------------------------------------------------------------------------
// Music button layout
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, PartialEq, Eq)]
enum MusicBtn {
    Prev,
    PlayPause,
    Next,
}

/// Returns which music control button was tapped, if any.
fn get_tapped_music_btn(p: Point, display_size: Size) -> Option<MusicBtn> {
    let (btn_w, btn_h, y0, gap) = music_btn_layout(display_size);
    let origins = [
        Point::new(gap, y0),
        Point::new(gap + btn_w + gap, y0),
        Point::new(gap + (btn_w + gap) * 2, y0),
    ];
    let variants = [MusicBtn::Prev, MusicBtn::PlayPause, MusicBtn::Next];
    for (i, &o) in origins.iter().enumerate() {
        if point_in_rect_padded(p, o, Size::new(btn_w as u32, btn_h as u32), 8) {
            return Some(variants[i]);
        }
    }
    None
}

/// (btn_w, btn_h, y_top, gap) for the three music control buttons.
fn music_btn_layout(display_size: Size) -> (i32, i32, i32, i32) {
    let w = display_size.width as i32;
    let h = display_size.height as i32;
    let gap = 10i32;
    let btn_w = (w - gap * 4) / 3;
    let btn_h = 36i32;
    let y0 = h - btn_h - gap;
    (btn_w, btn_h, y0, gap)
}

fn alarm_stop_button(display_size: Size) -> Point {
    Point::new(
        (display_size.width as i32 - 160) / 2,
        display_size.height as i32 - 58,
    )
}

fn alarm_stop_hitbox() -> (Point, Size) {
    (Point::new(240, 86), Size::new(80, 70))
}

fn draw_alarm_page<D>(
    display: &mut D,
    display_size: Size,
    font: &MonoFont<'_>,
    background: Rgb565,
    track_name: &str,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    display.fill_solid(&Rectangle::new(Point::new(0, 0), display_size), background)?;
    let fg = Rgb565::new(0xFF, 0xFF, 0xFF);
    let alert = Rgb565::new(0xFF, 0x40, 0x40);
    draw_text_line(
        display,
        font,
        "ALARM",
        Point::new(88, 36),
        2,
        alert,
        background,
    )?;
    draw_text_line(
        display,
        font,
        "Playing:",
        Point::new(28, 100),
        1,
        fg,
        background,
    )?;
    draw_text_line(
        display,
        font,
        track_name,
        Point::new(28, 126),
        1,
        fg,
        background,
    )?;

    let origin = alarm_stop_button(display_size);
    draw_btn_simple(
        display,
        font,
        origin.x,
        origin.y,
        "STOP",
        160,
        44,
        Rgb565::new(0x14, 0x00, 0x00),
        fg,
    )?;
    Ok(())
}

fn draw_air_quality_page<D>(
    display: &mut D,
    display_size: Size,
    font: &MonoFont<'_>,
    background: Rgb565,
    reading: AirReading,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    display.fill_solid(&Rectangle::new(Point::new(0, 0), display_size), background)?;
    display.fill_solid(
        &Rectangle::new(
            Point::new(0, 0),
            Size::new(display_size.width, GRID_BAR_H as u32),
        ),
        Rgb565::new(0x02, 0x0F, 0x07),
    )?;
    display.fill_solid(
        &Rectangle::new(
            Point::new(BACK_BTN_X, BACK_BTN_Y),
            Size::new(BACK_BTN_SZ as u32, BACK_BTN_SZ as u32),
        ),
        Rgb565::new(0x00, 0x04, 0x02),
    )?;
    let arrow_color = Rgb565::new(0xFF, 0xFF, 0xFF);
    let ax = BACK_BTN_X + BACK_BTN_SZ / 2;
    let ay = BACK_BTN_Y + BACK_BTN_SZ / 2;
    Line::new(Point::new(ax + 6, ay - 6), Point::new(ax - 6, ay))
        .into_styled(PrimitiveStyle::with_stroke(arrow_color, 2))
        .draw(display)?;
    Line::new(Point::new(ax + 6, ay + 6), Point::new(ax - 6, ay))
        .into_styled(PrimitiveStyle::with_stroke(arrow_color, 2))
        .draw(display)?;

    let fg = Rgb565::new(0xDF, 0xF3, 0xFF);
    draw_text_line(
        display,
        font,
        "AIR QUALITY",
        Point::new(78, 3),
        1,
        fg,
        background,
    )?;

    let mut raw_buf = [0u8; 4];
    write_u4(reading.raw.min(9999), &mut raw_buf);
    let raw = core::str::from_utf8(&raw_buf).unwrap_or("0000");
    draw_text_line(display, font, "RAW", Point::new(28, 58), 1, fg, background)?;
    draw_text_line(display, font, raw, Point::new(170, 58), 1, fg, background)?;

    let mut mv_buf = [0u8; 4];
    write_u4(reading.voltage_mv.min(9999), &mut mv_buf);
    let mv = core::str::from_utf8(&mv_buf).unwrap_or("0000");
    draw_text_line(
        display,
        font,
        "VOLT mV",
        Point::new(28, 92),
        1,
        fg,
        background,
    )?;
    draw_text_line(display, font, mv, Point::new(170, 92), 1, fg, background)?;

    let mut ppm_buf = [0u8; 4];
    write_u4(reading.ppm.min(9999), &mut ppm_buf);
    let ppm = core::str::from_utf8(&ppm_buf).unwrap_or("0000");
    draw_text_line(
        display,
        font,
        "CO2 ppm",
        Point::new(28, 126),
        1,
        fg,
        background,
    )?;
    draw_text_line(display, font, ppm, Point::new(170, 126), 1, fg, background)?;

    let status = air_quality_status(reading.ppm);
    draw_text_line(
        display,
        font,
        status,
        Point::new(80, 178),
        2,
        fg,
        background,
    )?;
    Ok(())
}

fn update_air_quality_values<D>(
    display: &mut D,
    font: &MonoFont<'_>,
    background: Rgb565,
    previous: AirReading,
    reading: AirReading,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let fg = Rgb565::new(0xDF, 0xF3, 0xFF);
    if previous.raw != reading.raw {
        draw_air_value(
            display,
            font,
            background,
            reading.raw,
            Point::new(170, 58),
            1,
        )?;
    }
    if previous.voltage_mv != reading.voltage_mv {
        draw_air_value(
            display,
            font,
            background,
            reading.voltage_mv,
            Point::new(170, 92),
            1,
        )?;
    }
    if previous.ppm != reading.ppm {
        draw_air_value(
            display,
            font,
            background,
            reading.ppm,
            Point::new(170, 126),
            1,
        )?;
    }

    let old_status = air_quality_status(previous.ppm);
    let new_status = air_quality_status(reading.ppm);
    if old_status != new_status {
        display.fill_solid(
            &Rectangle::new(Point::new(70, 174), Size::new(190, 44)),
            background,
        )?;
        draw_text_line(
            display,
            font,
            new_status,
            Point::new(80, 178),
            2,
            fg,
            background,
        )?;
    }

    Ok(())
}

fn draw_air_value<D>(
    display: &mut D,
    font: &MonoFont<'_>,
    background: Rgb565,
    value: u32,
    origin: Point,
    scale: u32,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let fg = Rgb565::new(0xDF, 0xF3, 0xFF);
    let area = Size::new(
        (font.character_size.width + font.character_spacing) * 4 * scale,
        font.character_size.height * scale,
    );
    display.fill_solid(&Rectangle::new(origin, area), background)?;

    let mut buf = [0u8; 4];
    write_u4(value.min(9999), &mut buf);
    let text = core::str::from_utf8(&buf).unwrap_or("0000");
    draw_text_line(display, font, text, origin, scale, fg, background)
}

fn air_quality_status(ppm: u32) -> &'static str {
    if ppm < 800 {
        "GOOD"
    } else if ppm < 1200 {
        "VENTILATE"
    } else {
        "POOR"
    }
}

fn write_u4(mut value: u32, buf: &mut [u8; 4]) {
    for i in (0..4).rev() {
        buf[i] = b'0' + (value % 10) as u8;
        value /= 10;
    }
}

// ---------------------------------------------------------------------------
// Draw: music page
// ---------------------------------------------------------------------------

fn draw_music_page<D>(
    display: &mut D,
    display_size: Size,
    font: &MonoFont<'_>,
    background: Rgb565,
    sd_ok: bool,
    tracks: &[TrackName; MAX_TRACKS],
    track_count: usize,
    current: usize,
    playing: bool,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let w = display_size.width as i32;

    display.fill_solid(&Rectangle::new(Point::new(0, 0), display_size), background)?;

    // ---- Top bar with back arrow ----------------------------------------
    let bar_h = GRID_BAR_H as u32;
    display.fill_solid(
        &Rectangle::new(Point::new(0, 0), Size::new(display_size.width, bar_h)),
        Rgb565::new(0x02, 0x0F, 0x07),
    )?;

    // Back arrow
    let ax = BACK_BTN_X + BACK_BTN_SZ / 2;
    let ay = BACK_BTN_Y + BACK_BTN_SZ / 2;
    let al = 6i32;
    let arrow_color = Rgb565::new(0xFF, 0xFF, 0xFF);
    // Draw filled back button background
    display.fill_solid(
        &Rectangle::new(
            Point::new(BACK_BTN_X, BACK_BTN_Y),
            Size::new(BACK_BTN_SZ as u32, BACK_BTN_SZ as u32),
        ),
        Rgb565::new(0x00, 0x04, 0x02),
    )?;
    Line::new(Point::new(ax + al, ay - al), Point::new(ax - al, ay))
        .into_styled(PrimitiveStyle::with_stroke(arrow_color, 2))
        .draw(display)?;
    Line::new(Point::new(ax + al, ay + al), Point::new(ax - al, ay))
        .into_styled(PrimitiveStyle::with_stroke(arrow_color, 2))
        .draw(display)?;

    // "MUSIC" title centred in top bar
    let title = "MUSIC";
    let ts = scaled_text_size(title, font, 1);
    let to = Point::new(
        (w - ts.width as i32) / 2,
        (bar_h as i32 - ts.height as i32) / 2,
    );
    draw_text_line(
        display,
        font,
        title,
        to,
        1,
        Rgb565::new(0xDF, 0xF3, 0xFF),
        Rgb565::new(0x02, 0x0F, 0x07),
    )?;

    // ---- Track list (up to 5 visible) -----------------------------------
    let list_top = GRID_BAR_H + 6;
    let (_, _btn_h, btn_y0, _) = music_btn_layout(display_size);
    let list_bottom = btn_y0 - 6;
    let row_h = font.character_size.height as i32 + 4;
    let visible = ((list_bottom - list_top) / row_h).min(5).max(0) as usize;

    let highlight_bg = Rgb565::new(0x00, 0x08, 0x04);
    let normal_fg = Rgb565::new(0xAA, 0xCC, 0xBB);
    let highlight_fg = Rgb565::new(0xFF, 0xFF, 0xFF);

    if track_count == 0 {
        let msg = if sd_ok {
            "No MP3/WAV files"
        } else {
            "SD init failed"
        };
        let ms = scaled_text_size(msg, font, 1);
        let mo = Point::new((w - ms.width as i32) / 2, list_top + row_h);
        draw_text_line(display, font, msg, mo, 1, normal_fg, background)?;
    } else {
        // Centre the window around `current`.
        let window_start = if current < visible / 2 {
            0
        } else if current + visible - visible / 2 > track_count {
            track_count.saturating_sub(visible)
        } else {
            current - visible / 2
        };

        for vi in 0..visible {
            let ti = window_start + vi;
            if ti >= track_count {
                break;
            }

            let row_y = list_top + vi as i32 * row_h;
            let is_sel = ti == current;
            let (fg, bg) = if is_sel {
                (highlight_fg, highlight_bg)
            } else {
                (normal_fg, background)
            };

            // Highlight bar for selected track
            if is_sel {
                display.fill_solid(
                    &Rectangle::new(
                        Point::new(0, row_y),
                        Size::new(display_size.width, row_h as u32),
                    ),
                    highlight_bg,
                )?;
            }

            // Prefix: "▶ " for current+playing, "  " otherwise
            let prefix = if is_sel && playing { "> " } else { "  " };
            let name = tracks[ti].as_str();

            // Draw prefix
            let pfx_size = scaled_text_size(prefix, font, 1);
            draw_text_line(
                display,
                font,
                prefix,
                Point::new(4, row_y + 2),
                1,
                highlight_fg,
                bg,
            )?;
            // Draw name (truncate to fit)
            draw_text_line(
                display,
                font,
                name,
                Point::new(4 + pfx_size.width as i32, row_y + 2),
                1,
                fg,
                bg,
            )?;
        }
    }

    // ---- Control buttons: |< Play/Pause >| ------------------------------
    let (btn_w, btn_h2, y0, gap) = music_btn_layout(display_size);
    let btn_bg = Rgb565::new(0x00, 0x04, 0x02);
    let btn_fg = Rgb565::new(0xFF, 0xFF, 0xFF);
    let labels = ["<<", if playing { "||" } else { "|>" }, ">>"];
    let btn_x = [gap, gap + btn_w + gap, gap + (btn_w + gap) * 2];

    for i in 0..3 {
        let bx = btn_x[i];
        let rect = Rectangle::new(Point::new(bx, y0), Size::new(btn_w as u32, btn_h2 as u32));
        display.fill_solid(&rect, btn_bg)?;
        let ls = scaled_text_size(labels[i], font, 1);
        let lo = Point::new(
            bx + (btn_w - ls.width as i32) / 2,
            y0 + (btn_h2 - ls.height as i32) / 2,
        );
        draw_text_line(display, font, labels[i], lo, 1, btn_fg, btn_bg)?;
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Draw: menu page
// ---------------------------------------------------------------------------

fn draw_menu_page<D>(
    display: &mut D,
    display_size: Size,
    font: &MonoFont<'_>,
    hours: u8,
    minutes: u8,
    background: Rgb565,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let w = display_size.width as i32;
    let h = display_size.height as i32;

    display.fill_solid(&Rectangle::new(Point::new(0, 0), display_size), background)?;

    // Top bar
    let bar_h = GRID_BAR_H as u32;
    display.fill_solid(
        &Rectangle::new(Point::new(0, 0), Size::new(display_size.width, bar_h)),
        Rgb565::new(0x02, 0x0F, 0x07),
    )?;

    // Time in top bar
    let mut time_buf = [0u8; 5];
    format_time(hours, minutes, &mut time_buf);
    let time_str = core::str::from_utf8(&time_buf).unwrap_or("00:00");
    let ts = scaled_text_size(time_str, font, 1);
    let to = Point::new(
        (w - ts.width as i32) / 2,
        (bar_h as i32 - ts.height as i32) / 2,
    );
    draw_text_line(
        display,
        font,
        time_str,
        to,
        1,
        Rgb565::new(0xDF, 0xF3, 0xFF),
        background,
    )?;

    // Grid buttons
    let top = GRID_BAR_H + GRID_EDGE_M;
    let avail_h = h - top - GRID_EDGE_M;
    let btn_w = (w - GRID_EDGE_M * 2 - GRID_MARGIN) / 2;
    let btn_h = (avail_h - GRID_MARGIN) / 2;
    let btn_size = btn_w.min(btn_h);
    let grid_w = btn_size * 2 + GRID_MARGIN;
    let h_off = (w - grid_w) / 2;

    // Labels match the touch-handler grid order.
    let labels = ["Set Time", "Music", "Set Alarm", "Air Quality"];
    let btn_bg = Rgb565::new(0x00, 0x04, 0x02);

    for row in 0..2i32 {
        for col in 0..2i32 {
            let idx = (row * 2 + col) as usize;
            let origin = Point::new(
                h_off + col * (btn_size + GRID_MARGIN),
                top + row * (btn_size + GRID_MARGIN),
            );
            display.fill_solid(
                &Rectangle::new(origin, Size::new(btn_size as u32, btn_size as u32)),
                btn_bg,
            )?;

            let label = labels[idx];
            let ls = scaled_text_size(label, font, 1);
            let lo = Point::new(
                origin.x + (btn_size - ls.width as i32) / 2,
                origin.y + (btn_size - ls.height as i32) / 2,
            );
            draw_text_line(
                display,
                font,
                label,
                lo,
                1,
                Rgb565::new(0xFF, 0xFF, 0xFF),
                btn_bg,
            )?;
        }
    }

    // Back button
    let back_origin = Point::new(BACK_BTN_X, BACK_BTN_Y);
    display.fill_solid(
        &Rectangle::new(
            back_origin,
            Size::new(BACK_BTN_SZ as u32, BACK_BTN_SZ as u32),
        ),
        Rgb565::new(0x00, 0x04, 0x02),
    )?;

    let arrow_color = Rgb565::new(0xFF, 0xFF, 0xFF);
    let ax = BACK_BTN_X + BACK_BTN_SZ / 2;
    let ay = BACK_BTN_Y + BACK_BTN_SZ / 2;
    let al = 6i32;
    Line::new(Point::new(ax + al, ay - al), Point::new(ax - al, ay))
        .into_styled(PrimitiveStyle::with_stroke(arrow_color, 2))
        .draw(display)?;
    Line::new(Point::new(ax + al, ay + al), Point::new(ax - al, ay))
        .into_styled(PrimitiveStyle::with_stroke(arrow_color, 2))
        .draw(display)?;

    Ok(())
}

// ---------------------------------------------------------------------------
// Helper: go back to clock page
// ---------------------------------------------------------------------------

fn go_to_clock<D>(
    display: &mut D,
    page: &mut Page,
    menu_btn_origin: Point,
    menu_btn_size: Size,
    font: &MonoFont<'_>,
    background: Rgb565,
    last_hours: &mut u8,
    last_minutes: &mut u8,
    last_time_buf: &mut [u8; 5],
) where
    D: DrawTarget<Color = Rgb565>,
{
    *page = Page::Clock;
    display.clear(background).ok();
    draw_menu_button(display, menu_btn_origin, menu_btn_size, font, background).ok();
    *last_hours = 255;
    *last_minutes = 255;
    *last_time_buf = [0xFF; 5];
}

async fn read_air_quality(
    adc: &mut Adc<'_, peripherals::ADC1>,
    pin: &mut embassy_stm32::Peri<'_, peripherals::PA0>,
) -> AirReading {
    let mut raw_sum = 0u32;
    for _ in 0..AIR_OVERSAMPLE_COUNT {
        raw_sum += adc.blocking_read(pin, SampleTime::CYCLES160_5) as u32;
        Timer::after(Duration::from_micros(500)).await;
    }
    let raw = raw_sum / AIR_OVERSAMPLE_COUNT;
    let voltage_mv = (raw * VDDA_MV) / ADC_FULL_SCALE;
    let ppm = if voltage_mv == 0 || voltage_mv >= VDDA_MV {
        0
    } else {
        let vout = voltage_mv as f32;
        let vcc = VDDA_MV as f32;
        let rs = MQ135_R_LOAD_OHM * (vcc - vout) / vout;
        let ratio = rs / MQ135_R0_OHM;
        (116.602_07 * libm::powf(ratio, -2.769_035)).clamp(0.0, 9999.0) as u32
    };
    AirReading {
        raw,
        voltage_mv,
        ppm,
    }
}

fn find_track_005(tracks: &[TrackName; MAX_TRACKS], track_count: usize) -> Option<usize> {
    for i in 0..track_count {
        let name = tracks[i].as_str().as_bytes();
        if name.len() >= 3 && name[0] == b'0' && name[1] == b'0' && name[2] == b'5' {
            return Some(i);
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Hit testing
// ---------------------------------------------------------------------------

fn point_in_rect(p: Point, origin: Point, size: Size) -> bool {
    p.x >= origin.x
        && p.x < origin.x + size.width as i32
        && p.y >= origin.y
        && p.y < origin.y + size.height as i32
}

fn point_in_rect_padded(p: Point, origin: Point, size: Size, pad: i32) -> bool {
    p.x >= origin.x - pad
        && p.x < origin.x + size.width as i32 + pad
        && p.y >= origin.y - pad
        && p.y < origin.y + size.height as i32 + pad
}

fn get_tapped_grid_button(point: Point, display_size: Size) -> Option<(usize, i32, i32)> {
    let top = GRID_BAR_H + GRID_EDGE_M;
    let available_h = display_size.height as i32 - top - GRID_EDGE_M;
    let btn_w = (display_size.width as i32 - GRID_EDGE_M * 2 - GRID_MARGIN) / 2;
    let btn_h = (available_h - GRID_MARGIN) / 2;
    let btn_size = btn_w.min(btn_h);
    let grid_total_w = btn_size * 2 + GRID_MARGIN;
    let h_offset = (display_size.width as i32 - grid_total_w) / 2;

    for row in 0..2i32 {
        for col in 0..2i32 {
            let idx = (row * 2 + col) as usize;
            let origin = Point::new(
                h_offset + col * (btn_size + GRID_MARGIN),
                top + row * (btn_size + GRID_MARGIN),
            );
            if point_in_rect(point, origin, Size::new(btn_size as u32, btn_size as u32)) {
                return Some((idx, origin.x, origin.y));
            }
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Touch reading
// ---------------------------------------------------------------------------

fn read_touch<S: SpiDevice>(spi: &mut S) -> Option<(u16, u16)> {
    let mut buf = [0x90_u8, 0x00, 0x00, 0xD0, 0x00, 0x00];
    spi.transfer_in_place(&mut buf).ok()?;
    let x = ((((buf[1] as u16) << 8) | buf[2] as u16) >> 3) & 0x0FFF;
    let y = ((((buf[4] as u16) << 8) | buf[5] as u16) >> 3) & 0x0FFF;
    if x < 150 || y < 150 || x > 3950 || y > 3950 {
        None
    } else {
        Some((x, y))
    }
}

fn read_touch_avg<S: SpiDevice>(spi: &mut S) -> Option<(u16, u16)> {
    let mut xs = [0u16; 8];
    let mut ys = [0u16; 8];
    let mut n = 0usize;
    for _ in 0..8 {
        if let Some((x, y)) = read_touch(spi) {
            xs[n] = x;
            ys[n] = y;
            n += 1;
        }
    }
    if n < 3 {
        return None;
    }
    xs[..n].sort_unstable();
    ys[..n].sort_unstable();
    let avg =
        |s: &[u16]| -> u16 { (s.iter().map(|&v| v as u32).sum::<u32>() / s.len() as u32) as u16 };
    Some((avg(&xs[1..n - 1]), avg(&ys[1..n - 1])))
}

// ---------------------------------------------------------------------------
// Display mode
// ---------------------------------------------------------------------------

struct LandscapeFlip;

impl Mode for LandscapeFlip {
    /// 0x68 = landscape + horizontal flip (normal upright orientation).
    /// Change to 0xA8 if the image is upside-down.
    fn mode(&self) -> u8 {
        0x68
    }
    fn is_landscape(&self) -> bool {
        true
    }
}

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------

fn format_time(hours: u8, minutes: u8, buf: &mut [u8; 5]) {
    buf[0] = b'0' + hours / 10;
    buf[1] = b'0' + hours % 10;
    buf[2] = b':';
    buf[3] = b'0' + minutes / 10;
    buf[4] = b'0' + minutes % 10;
}

fn scaled_text_size(text: &str, font: &MonoFont<'_>, scale: u32) -> Size {
    let n = text.chars().count() as u32;
    let width = n
        .saturating_mul(font.character_size.width + font.character_spacing)
        .saturating_mul(scale)
        .saturating_sub(font.character_spacing.saturating_mul(scale));
    Size::new(width, font.character_size.height.saturating_mul(scale))
}

fn draw_text_line<D>(
    display: &mut D,
    font: &MonoFont<'_>,
    text: &str,
    origin: Point,
    scale: u32,
    fg: Rgb565,
    bg: Rgb565,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let step = (font.character_size.width + font.character_spacing) as i32 * scale as i32;
    for (idx, ch) in text.chars().enumerate() {
        draw_scaled_char(
            display,
            font,
            ch,
            Point::new(origin.x + idx as i32 * step, origin.y),
            scale,
            fg,
            bg,
        )?;
    }
    Ok(())
}

fn draw_scaled_char<D, C>(
    target: &mut D,
    font: &MonoFont<'_>,
    c: char,
    origin: Point,
    scale: u32,
    text_color: C,
    background_color: C,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = C>,
    C: PixelColor + Copy,
{
    target.fill_solid(
        &Rectangle::new(
            origin,
            Size::new(
                font.character_size.width.saturating_mul(scale),
                font.character_size.height.saturating_mul(scale),
            ),
        ),
        background_color,
    )?;
    if c != ' ' {
        draw_scaled_glyph(target, font, c, origin, scale, text_color)?;
    }
    Ok(())
}

fn draw_scaled_glyph<D, C>(
    target: &mut D,
    font: &MonoFont<'_>,
    c: char,
    origin: Point,
    scale: u32,
    color: C,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = C>,
    C: PixelColor + Copy,
{
    let glyph_index = font.glyph_mapping.index(c) as u32;
    let glyphs_per_row = font.image.size().width / font.character_size.width;
    let row = glyph_index / glyphs_per_row;
    let glyph_x = (glyph_index - row * glyphs_per_row) * font.character_size.width;
    let glyph_y = row * font.character_size.height;

    for y in 0..font.character_size.height {
        let mut x = 0u32;
        while x < font.character_size.width {
            while x < font.character_size.width {
                if font.image.pixel(Point::new(
                    glyph_x as i32 + x as i32,
                    glyph_y as i32 + y as i32,
                )) == Some(BinaryColor::On)
                {
                    break;
                }
                x += 1;
            }
            if x >= font.character_size.width {
                break;
            }
            let run_start = x;
            while x < font.character_size.width {
                if font.image.pixel(Point::new(
                    glyph_x as i32 + x as i32,
                    glyph_y as i32 + y as i32,
                )) != Some(BinaryColor::On)
                {
                    break;
                }
                x += 1;
            }
            let run_width = (x - run_start) as u32;
            target.fill_solid(
                &Rectangle::new(
                    Point::new(
                        origin.x + (run_start as i32).saturating_mul(scale as i32),
                        origin.y + (y as i32).saturating_mul(scale as i32),
                    ),
                    Size::new(run_width.saturating_mul(scale), scale),
                ),
                color,
            )?;
        }
    }
    Ok(())
}

fn draw_btn_simple<D>(
    display: &mut D,
    font: &MonoFont<'_>,
    x: i32,
    y: i32,
    label: &str,
    bw: i32,
    bh: i32,
    bg: Rgb565,
    fg: Rgb565,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    display.fill_solid(
        &Rectangle::new(Point::new(x, y), Size::new(bw as u32, bh as u32)),
        bg,
    )?;
    let ls = scaled_text_size(label, font, 1);
    draw_text_line(
        display,
        font,
        label,
        Point::new(
            x + (bw - ls.width as i32) / 2,
            y + (bh - ls.height as i32) / 2,
        ),
        1,
        fg,
        bg,
    )
}

fn draw_menu_button<D>(
    display: &mut D,
    origin: Point,
    size: Size,
    font: &MonoFont<'_>,
    background: Rgb565,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let button_color = Rgb565::new(0x00, 0x04, 0x02);
    let corner_radius = 4u32;

    Rectangle::new(origin, size)
        .into_styled(PrimitiveStyle::with_fill(button_color))
        .draw(display)?;

    for i in 0..corner_radius {
        let a = i as i32;
        for &p in &[
            Point::new(origin.x + a, origin.y + corner_radius as i32 - 1 - a),
            Point::new(
                origin.x + size.width as i32 - 1 - a,
                origin.y + corner_radius as i32 - 1 - a,
            ),
            Point::new(
                origin.x + a,
                origin.y + size.height as i32 - corner_radius as i32 + a,
            ),
            Point::new(
                origin.x + size.width as i32 - 1 - a,
                origin.y + size.height as i32 - corner_radius as i32 + a,
            ),
        ] {
            Rectangle::new(p, Size::new(1, 1))
                .into_styled(PrimitiveStyle::with_fill(background))
                .draw(display)?;
        }
    }

    let ts = scaled_text_size("MENU", font, 1);
    let to = Point::new(
        origin.x + (size.width as i32 - ts.width as i32) / 2,
        origin.y + (size.height as i32 - ts.height as i32) / 2,
    );
    draw_text_line(
        display,
        font,
        "MENU",
        to,
        1,
        Rgb565::new(0xFF, 0xFF, 0xFF),
        button_color,
    )?;
    Ok(())
}
