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

mod config;
mod geometry;
mod measurement;
mod trajectory;
mod uncertainty;
mod utils;

pub use config::Config;
pub use geometry::Point;
pub use measurement::Measurement;
pub use trajectory::Trajectory;
pub use uncertainty::Uncertainty;
pub use utils::{measurements, read_las};
