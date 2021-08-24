use crate::config::initialization::init;
use crate::config::initialization::*;
use core::f32::consts::PI;
use lsm303agr::UnscaledMeasurement;
use m::Float;
use stm32f3_discovery::stm32f3xx_hal::{
    gpio::gpiob::{PB6, PB7},
    gpio::AF4,
    i2c::I2c,
    pac::I2C1,
};
use stm32f3_discovery::*;

/// This function calculates the Direction of the Magnetic Field using x, y, z-axis readings.
///
/// #Input
/// Function magnitude takes a single input lsm303agr as package("Lsm303agr") type.
///
/// #Return
/// magnitude function return enum of (take_readings, direction, theta)
/// take_readings -> this return the readings of the Magnetometer sensor.
/// direction -> return the direction of Magnetic Field based on theta value.
/// theta -> return the value of theta based on "atan2" function.
pub fn direction(
    lsm303agr: &mut Lsm303agr<I2cInterface<I2c<I2C1, (PB6<AF4>, PB7<AF4>)>>, mode::MagContinuous>,
) -> (UnscaledMeasurement, Direction, f32) {
    let take_readings = lsm303agr.mag_data().expect("Error in readings package.");
    let theta = (take_readings.y as f32).atan2(take_readings.x as f32);
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
    (take_readings, direction, theta)
}
/// This function calculates the Magnitude of the Magnetic Field using x, y, z-axis readings.
///
/// #Input
/// Function magnitude takes a single input lsm303agr as package("Lsm303agr") type.
///
/// #Return
/// magnitude function return the calculated magnitude of the Magnetic Field.
pub fn magnitude(
    lsm303agr: &mut Lsm303agr<I2cInterface<I2c<I2C1, (PB6<AF4>, PB7<AF4>)>>, mode::MagContinuous>,
) -> f32 {
    let take_readings = lsm303agr.mag_data().expect("Failed to read the values.");
    const XY_GAIN: f32 = 1100.;
    const Z_GAIN: f32 = 980.;
    let x = f32::from(take_readings.x) / XY_GAIN;
    let y = f32::from(take_readings.y) / XY_GAIN;
    let z = f32::from(take_readings.z) / Z_GAIN;
    let magnitude = (x * x + y * y + z * z).sqrt();
    magnitude
}
