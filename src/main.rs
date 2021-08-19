#![deny(unsafe_code)]
#![no_main]
#![no_std]

use compass_2_0::config::initialization::{
    entry, init, iprintln, switch_hal::OutputSwitch, Direction,
};
use core::f32::consts::PI;
use m::Float;
use stm32f3_discovery::stm32f3xx_hal::prelude::*;

/// This program upgrades the compass functionality by providing direction(N, S, W, E, NS, etc)
/// based on theta value and also print the magnitude of the compass.
#[entry]
fn main() -> ! {
    const X_Y_GAIN: f32 = 1100.;
    const Z_GAIN: f32 = 980.;
    let (leds, mut lsm303agr, mut delay, mut itm) = init();
    let mut f3_led = leds.into_array();
    loop {
        let magnetometer_reading = lsm303agr.mag_data();
        let x_y_axis = match magnetometer_reading {
            Ok(value) => value,
            Err(error) => panic!("Failed to read Magnetometer Data {:?}", error),
        };
        let theta = (x_y_axis.y as f32).atan2(x_y_axis.x as f32);

        let direction = if theta < -7. * PI / 8. {
            Direction::North
        } else if theta < -5. * PI / 8. {
            Direction::Northwest
        } else if theta < -3. * PI / 8. {
            Direction::West
        } else if theta < -PI / 8. {
            Direction::Southwest
        } else if theta < PI / 8. {
            Direction::South
        } else if theta < 3. * PI / 8. {
            Direction::Southeast
        } else if theta < 5. * PI / 8. {
            Direction::East
        } else if theta < 7. * PI / 8. {
            Direction::Northeast
        } else {
            Direction::North
        };
        let x = f32::from(x_y_axis.x) / X_Y_GAIN;
        let y = f32::from(x_y_axis.y) / X_Y_GAIN;
        let z = f32::from(x_y_axis.z) / Z_GAIN;

        let magnitude = (x * x + y * y + z * z).sqrt();

        f3_led.iter_mut().for_each(|leds| match leds.off() {
            Ok(leds) => leds,
            Err(..) => {}
        });
        f3_led[direction as usize].on().ok();
        iprintln!(
            &mut itm.stim[0],
            "\nx = {} y = {} z = {} theta {}\n",
            x_y_axis.x,
            x_y_axis.y,
            x_y_axis.z,
            theta
        );
        iprintln!(
            &mut itm.stim[0],
            "Magnetometer Magnitude {} mG",
            magnitude * 1_000.
        );
        delay.delay_ms(2_000_u16);
    }
}
