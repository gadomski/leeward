use anyhow::Error;
use std::path::Path;

/// System and platform configuration.
#[derive(Debug)]
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
    pub fn from_path<P: AsRef<Path>>(_path: P) -> Result<Config, Error> {
        Ok(Config {})
    }
}
