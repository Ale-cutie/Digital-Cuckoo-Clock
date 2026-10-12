#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Flex, Pull, Speed};
use embassy_time::Delay;
use embedded_hal_async::delay::DelayNs as _;

use dht_sensor::dht11;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());

    info!("DHT11 test started");

    let mut delay = Delay;

    let mut one_wire_pin = Flex::new(p.PC8);

    one_wire_pin.set_as_input_output_pull(Speed::VeryHigh, Pull::Up);
    one_wire_pin.set_high();

    info!("Waiting for sensor...");
    delay.delay_ms(3000).await;

    loop {
        match dht11::r#async::read(&mut delay, &mut one_wire_pin).await {
            Ok(dht11::Reading {
                temperature,
                relative_humidity,
            }) => {
                info!(
                    "Temperature: {} C, Humidity: {} %",
                    temperature, relative_humidity
                );
            }
            Err(_) => {
                warn!("DHT11 read failed");
            }
        }

        delay.delay_ms(3000).await;
    }
}
