use anyhow::Error;
use serde::Deserialize;
use std::path::Path;

/// System and platform configuration.
#[derive(Debug, Deserialize, Default)]
pub struct Config {}

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
