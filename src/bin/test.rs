#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Delay;
use embedded_hal::delay::DelayNs;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    info!("PA1 test started");

    let mut pin = Output::new(p.PA1, Level::Low, Speed::Low);
    let mut delay = Delay;

    loop {
        info!("HIGH");
        pin.set_high();
        delay.delay_ms(1000u32);

        info!("LOW");
        pin.set_low();
        delay.delay_ms(1000u32);
    }
}
