//! Configuration structures.
//!
//! These are all "dumb" structures to hold data.

use anyhow::Error;
use nalgebra::{Matrix3, Vector3};
use serde::Deserialize;
use std::path::Path;

/// System and platform configuration.
#[derive(Debug, Deserialize, Default)]
pub struct Config {
    pub utm_zone: u8,
    pub boresight: RollPitchYaw,
    pub lever_arm: Vector3<f64>,
}

/// Rotation as defined by a roll, pitch, and yaw.
#[derive(Debug, Deserialize, Default)]
pub struct RollPitchYaw {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl Config {
    /// Creates a configuration from a toml path.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Config;
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// ```
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Config, Error> {
        use std::{fs::File, io::Read};
        let mut string = String::new();
        File::open(path).and_then(|mut f| f.read_to_string(&mut string))?;
        toml::from_str(&string).map_err(Error::from)
    }
}

impl RollPitchYaw {
    /// Returns this rpy as a rotation matrix.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::config::RollPitchYaw;
    /// let rpy = RollPitchYaw { roll: 0., pitch: 0., yaw: 0. };
    /// let rotation_matrix = rpy.to_rotation_matrix();
    /// ```
    pub fn to_rotation_matrix(&self) -> Matrix3<f64> {
        crate::utils::rotation_matrix(self.roll, self.pitch, self.yaw)
    }
}
