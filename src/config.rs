use crate::geometry::{Rotation, Vector};
use crate::partials::Variable;
use anyhow::{anyhow, Error};
use serde::Deserialize;

/// A lidar equation configuration.
#[derive(Clone, Copy, Debug, Deserialize, Default)]
pub struct Config {
    pub utm_zone: u8,
    pub lever_arm: Vector,
    pub boresight: Rotation,
}

impl Config {
    /// Adjust the configuration by the associated delta.
    pub fn adjust(&mut self, variable: Variable, value: f64) -> Result<(), Error> {
        match variable {
            Variable::BoresightRoll => self.boresight.roll -= value,
            Variable::BoresightPitch => self.boresight.pitch -= value,
            Variable::BoresightYaw => self.boresight.yaw -= value,
            _ => return Err(anyhow!("Cannot adjust variable: {}", variable)),
        }
        Ok(())
    }
}
