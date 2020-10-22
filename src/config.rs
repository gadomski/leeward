use crate::geometry::{Rotation, Vector};
use anyhow::Error;
use serde::Deserialize;
use std::path::Path;

/// A lidar equation configuration.
#[derive(Clone, Copy, Debug, Deserialize, Default)]
pub struct Config {
    pub utm_zone: u8,
    pub lever_arm: Vector,
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
