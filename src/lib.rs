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

mod boresight;
#[cfg(feature = "capi")]
pub mod capi;
mod config;
mod measurement;
mod partial;
mod rotation;
mod trajectory;
mod utils;

pub use boresight::Boresight;
pub use config::{Config, ErrorConfig};
pub use measurement::{Measurement, Uncertainty};
pub use partial::{Dimension, Partial, Variable};
pub use rotation::Rotation;
pub use trajectory::Trajectory;
pub use utils::{measurements, read_las};
