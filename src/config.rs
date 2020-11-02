//! Configuration structures.
//!
//! These are all "dumb" structures to hold data.

use crate::Rotation;
use anyhow::Error;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::path::Path;

/// System and platform configuration.
#[derive(Copy, Clone, Debug, Deserialize, Serialize, Default)]
pub struct Config {
    pub utm_zone: u8,
    pub use_las_scan_angle: bool,
    pub lever_arm: Vector3<f64>,
    pub boresight: Rotation,
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
