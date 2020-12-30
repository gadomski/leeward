use crate::{Point, RollPitchYaw, Variable};
use anyhow::{anyhow, Error};
use nalgebra::DVector;
use serde::Deserialize;
use std::{fs::File, io::Read, path::Path};

/// Configuration structure.
#[derive(Debug, Deserialize, Clone, Copy, PartialEq)]
pub struct Config {
    pub utm_zone: u8,
    pub boresight: RollPitchYaw,
    pub lever_arm: Point,
}

impl Config {
    /// Reads a new configuration from a toml file.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Config;
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// ```
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Config, Error> {
        let mut string = String::new();
        File::open(path).and_then(|mut f| f.read_to_string(&mut string))?;
        toml::from_str(&string).map_err(Error::from)
    }

    /// Returns a vector of values as specified by the provided variables.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Variable, Config};
    /// let variables = vec![Variable::BoresightRoll, Variable::BoresightPitch];
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let values = config.values(&variables).unwrap();
    /// ```
    pub fn values(&self, variables: &[Variable]) -> Result<DVector<f64>, Error> {
        let mut values = DVector::zeros(variables.len());
        for (variable, value) in variables.iter().zip(values.iter_mut()) {
            *value = match variable {
                Variable::BoresightRoll => self.boresight.roll,
                Variable::BoresightPitch => self.boresight.pitch,
                Variable::BoresightYaw => self.boresight.yaw,
                Variable::LeverArmX => self.lever_arm.x,
                Variable::LeverArmY => self.lever_arm.y,
                Variable::LeverArmZ => self.lever_arm.z,
                _ => return Err(anyhow!("cannot get value for variable: {:?}", variable)),
            };
        }
        Ok(values)
    }
}
