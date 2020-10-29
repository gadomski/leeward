use crate::{Config, Measurement, Trajectory};
use anyhow::Error;
use std::path::Path;

/// One-stop shop to create a vector of measurements from a trajectory file, point cloud file, and config file.
///
/// Dive deeper into the API to control aspects of the measurement computations.
///
/// # Examples
///
/// ```
/// let measurements = leeward::measurements(
///     "data/sbet.out",
///     "data/points.las",
///     "data/config.toml"
/// ).unwrap();
/// ```
pub fn measurements<P0: AsRef<Path>, P1: AsRef<Path>, P2: AsRef<Path>>(
    sbet: P0,
    las: P1,
    config: P2,
) -> Result<Vec<Measurement>, Error> {
    let trajectory = Trajectory::from_path(sbet)?;
    let config = Config::from_path(config)?;
    trajectory.read_las(las, &config)
}

/// Utility method to read a las file to a vector of points.
///
/// # Examples
///
/// ```
/// let points = leeward::read_las("data/points.las").unwrap();
/// ```
pub fn read_las<P: AsRef<Path>>(path: P) -> Result<Vec<las::Point>, Error> {
    use las::{Read, Reader};
    let mut reader = Reader::from_path(path)?;
    reader
        .points()
        .map(|result| result.map_err(Error::from))
        .collect()
}
