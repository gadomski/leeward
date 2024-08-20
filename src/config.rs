use crate::{Point, RollPitchYaw, Variable};
use anyhow::{anyhow, Error};
use nalgebra::DVector;
use serde::{Deserialize, Serialize};
use std::{fs::File, io::Read, path::Path};

/// Configuration structure.
#[derive(Debug, Serialize, Deserialize, Clone, Copy, PartialEq)]
pub struct Config {
    pub utm_zone: u8,
    pub beam_divergence: f64,
    pub lever_arm: Point,
    pub boresight: RollPitchYaw,
    pub uncertainty: Uncertainty,
}

/// Configuration for uncertainty config.
#[derive(Debug, Serialize, Deserialize, Clone, Copy, PartialEq)]
pub struct Uncertainty {
    pub gnss_x: f64,
    pub gnss_y: f64,
    pub gnss_z: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub boresight_roll: f64,
    pub boresight_pitch: f64,
    pub boresight_yaw: f64,
    pub lever_arm_x: f64,
    pub lever_arm_y: f64,
    pub lever_arm_z: f64,
    pub range: f64,
    pub scan_angle: f64,
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

    /// Returns a new configuration with the provided variables set to the provided values.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Variable, Config};
    /// let variables = vec![Variable::BoresightRoll, Variable::BoresightPitch];
    /// let values = vec![0.1, 0.2];
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let config = config.with_values(&variables, &values).unwrap();
    /// assert_eq!(0.1, config.boresight.roll);
    /// assert_eq!(0.2, config.boresight.pitch);
    /// ```
    pub fn with_values(&self, variables: &[Variable], values: &[f64]) -> Result<Config, Error> {
        let mut config = *self;
        for (variable, value) in variables.iter().zip(values) {
            let target = match variable {
                Variable::BoresightRoll => &mut config.boresight.roll,
                Variable::BoresightPitch => &mut config.boresight.pitch,
                Variable::BoresightYaw => &mut config.boresight.yaw,
                Variable::LeverArmX => &mut config.lever_arm.x,
                Variable::LeverArmY => &mut config.lever_arm.y,
                Variable::LeverArmZ => &mut config.lever_arm.z,
                _ => return Err(anyhow!("cannot set variable: {:?}", variable)),
            };
            *target = *value;
        }
        Ok(config)
    }
}
