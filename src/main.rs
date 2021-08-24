#![deny(unsafe_code)]
#![no_main]
#![no_std]

use compass_2_0::config::initialization::{entry, init, iprintln, switch_hal::OutputSwitch};

use compass_2_0::magnetometer::magnetometer::{direction, magnitude};
use stm32f3_discovery::stm32f3xx_hal::prelude::*;

/// This program upgrades the compass functionality by providing direction(N, S, W, E, NS, etc)
/// Based on theta value and also print the magnitude of the Compass.
///
/// #Arguements
/// None
///
/// #Return
/// function is using "no_std, no_main" return-> none
#[entry]
fn main() -> ! {
    let (leds, mut lsm303agr, mut delay, mut itm) = init();
    let mut f3_led = leds.into_array();
    let magnetometer_reading = direction(&mut lsm303agr).0;
    loop {
        f3_led.iter_mut().for_each(|leds| match leds.off() {
            Ok(leds) => leds,
            Err(..) => {}
        });
        // printing values of x, y, z-axis on itm
        f3_led[direction(&mut lsm303agr).1 as usize].on().ok();
        iprintln!(
            &mut itm.stim[0],
            "\nx = {} y = {} z = {} theta {}\n",
            magnetometer_reading.x,
            magnetometer_reading.y,
            magnetometer_reading.z,
            direction(&mut lsm303agr).2
        );
        // printing the magnitude of the Magnetic Field.
        iprintln!(
            &mut itm.stim[0],
            "Magnetometer Magnitude {} mG",
            magnitude(&mut lsm303agr) * 1_000.
        );
        delay.delay_ms(1_000_u16);
    }
}
