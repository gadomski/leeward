use anyhow::Error;
use serde::Deserialize;
use std::{fs::File, io::Read, path::Path};

/// Configuration structure.
#[derive(Debug, Deserialize, Clone, Copy)]
pub struct Config {
    pub utm_zone: u8,
}

impl Config {
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Config, Error> {
        let mut string = String::new();
        File::open(path).and_then(|mut f| f.read_to_string(&mut string))?;
        toml::from_str(&string).map_err(Error::from)
    }
}
