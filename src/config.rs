use crate::{Point, RollPitchYaw};
use anyhow::Error;
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
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Config, Error> {
        let mut string = String::new();
        File::open(path).and_then(|mut f| f.read_to_string(&mut string))?;
        toml::from_str(&string).map_err(Error::from)
    }
}
