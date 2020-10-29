//! Lidar Equation Engine With Already Racked Derivatives
//!
//! # Examples
//!
//! The core structure is a `Measurement`, which combines information about system configuration, platform orientation and position, and measured point into a single structure.
//! **leeward** uses Smoothed Best Estimate of Trajectory (sbet) files, which usually have a .out extension, and las point clouds.
//! Use `leeward::measurements(sbet_path, las_path, config_path)` as a convenience function to create a vector of `Measurement`:
//!
//! ```
//! let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
//! ```

mod measurement;
mod trajectory;

use anyhow::Error;
pub use measurement::Measurement;
use std::path::Path;
pub use trajectory::Trajectory;

/// One-stop shop to create a vector of measurements from a trajectory file, point cloud file, and config file.
///
/// Dive deeper into the API to control aspects of the measurement computations.
///
/// # Examples
///
/// ```
/// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
/// ```
pub fn measurements<P0: AsRef<Path>, P1: AsRef<Path>, P2: AsRef<Path>>(
    sbet: P0,
    las: P1,
    config: P2,
) -> Result<Vec<Measurement>, Error> {
    let trajectory = Trajectory::from_path(sbet)?;
    unimplemented!()
}
