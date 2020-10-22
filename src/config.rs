use crate::geometry::{Rotation, Vector};
use anyhow::Error;
use serde::Deserialize;
use std::path::Path;

/// A lidar equation configuration.
#[derive(Clone, Copy, Debug, Deserialize, Default)]
pub struct Config {
    /// The UTM zone in which to do the calculations.
    pub utm_zone: u8,

    /// The offset between the scanner origin and the IMU origin.
    pub lever_arm: Vector,

    /// The rotational misalignment between the IMU and the frame of the aircraft.
    pub boresight: Rotation,
}

impl Config {
    /// Creates a configuration from TOML in a file path.
    ///
    /// # Examples
    ///
    /// ```
    /// let config = leeward::Config::from_path("examples/config.toml").unwrap();
    /// ```
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Config, Error> {
        toml::from_str(&std::fs::read_to_string(path)?).map_err(Into::into)
    }
}
