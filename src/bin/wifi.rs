#![no_std]
#![no_main]

//! ESP8266 (ESP-01 / A411) AT-command driver test for STM32U545 / Nucleo-U545RE-Q.
//!
//! Wiring (Nucleo-U545RE-Q  <->  ESP-01):
//!   PA9  (USART1_TX) ----> ESP RX
//!   PA10 (USART1_RX) <---- ESP TX
//!   3V3              ----> ESP VCC  and  ESP CH_PD (via 10 kΩ to 3V3)
//!   GND              ----> ESP GND
//!   ESP GPIO0        ----> 3V3 (via 10 kΩ, must be HIGH at boot)
//!   ESP GPIO2        ----> leave floating
//!
//! Power note: put a 10 µF + 100 nF cap at the ESP's VCC pin. If it still
//! brown-outs, power the ESP from an external 3V3 supply (common GND).

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals;
use embassy_stm32::usart::{Config, InterruptHandler, Uart};
use embassy_time::{with_timeout, Duration, Timer};

// USART1 needs its own interrupt, plus the interrupts for the two GPDMA
// channels used for TX and RX DMA transfers.
bind_interrupts!(struct Irqs {
    USART1 => InterruptHandler<peripherals::USART1>;
    GPDMA1_CHANNEL0 => embassy_stm32::dma::InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<peripherals::GPDMA1_CH1>;
});

/// Send a command and read the response into `buf`, with a timeout.
/// Returns the number of bytes received (0 on error or timeout).
async fn at_command<'d>(
    uart: &mut Uart<'d, Async>,
    cmd: &[u8],
    buf: &mut [u8],
    timeout: Duration,
) -> usize {
    // Drain any leftover bytes from a previous reply so they don't pollute this read.
    let _ = with_timeout(Duration::from_millis(20), uart.read_until_idle(buf)).await;

    if uart.write(cmd).await.is_err() {
        warn!("UART write error");
        return 0;
    }

    match with_timeout(timeout, uart.read_until_idle(buf)).await {
        Ok(Ok(n)) => n,
        Ok(Err(_)) => {
            warn!("UART read error");
            0
        }
        Err(_) => {
            warn!("Timeout waiting for response");
            0
        }
    }
}

/// Pretty-print a response buffer over defmt (trims trailing whitespace).
fn log_response(label: &str, buf: &[u8]) {
    match core::str::from_utf8(buf) {
        Ok(s) => info!("{} -> {:?}", label, s.trim()),
        Err(_) => info!("{} -> (non-utf8, {} bytes): {:?}", label, buf.len(), buf),
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());

    info!("ESP8266 AT-command driver starting");

    // ---------------- UART CONFIG ----------------
    // ESP-01 AT firmware default is 115200 8N1, no flow control.
    let mut config = Config::default();
    config.baudrate = 115200;

    // embassy-stm32 0.6.0 signature:
    //   Uart::new(peri, rx, tx, tx_dma, rx_dma, irqs, config)
    let mut uart = Uart::new(
        p.USART1,
        p.PA10,       // RX
        p.PA9,        // TX
        p.GPDMA1_CH0, // TX DMA channel
        p.GPDMA1_CH1, // RX DMA channel
        Irqs,
        config,
    )
    .unwrap();

    // ---------------- BOOT WAIT ----------------
    // The ESP8266 prints garbage at 74880 baud during its bootloader phase.
    // Wait ~2 s for it to settle into AT firmware @ 115200 before talking to it.
    info!("Waiting for ESP8266 to boot...");
    Timer::after(Duration::from_millis(2000)).await;

    let mut buf = [0u8; 256];

    // ---------------- DISABLE ECHO ----------------
    // ATE0 stops the ESP from echoing our commands back, which makes parsing
    // responses much easier.
    info!("Disabling echo (ATE0)...");
    let n = at_command(&mut uart, b"ATE0\r\n", &mut buf, Duration::from_millis(500)).await;
    if n > 0 {
        log_response("ATE0", &buf[..n]);
    } else {
        warn!("No response to ATE0");
    }

    // ---------------- SANITY CHECK ----------------
    info!("Sanity check (AT)...");
    let n = at_command(&mut uart, b"AT\r\n", &mut buf, Duration::from_millis(500)).await;
    if n > 0 {
        log_response("AT", &buf[..n]);
    } else {
        error!("ESP8266 did not respond to AT. Check wiring, power, and pins.");
    }

    // ---------------- FIRMWARE VERSION ----------------
    info!("Querying firmware version (AT+GMR)...");
    let n = at_command(
        &mut uart,
        b"AT+GMR\r\n",
        &mut buf,
        Duration::from_millis(1000),
    )
    .await;
    if n > 0 {
        log_response("AT+GMR", &buf[..n]);
    }

    // ---------------- HEARTBEAT LOOP ----------------
    info!("Entering heartbeat loop");
    loop {
        Timer::after(Duration::from_secs(5)).await;
        info!("Ping...");
        let n = at_command(&mut uart, b"AT\r\n", &mut buf, Duration::from_millis(500)).await;
        if n > 0 {
            log_response("AT", &buf[..n]);
        } else {
            warn!("No response from ESP8266");
        }
    }
}
