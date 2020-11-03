//! Lidar Equation Engine With Already Racked Derivatives (aka leeward)
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
//!
//! Each measurement has an `uncertainty` method which produces the error covariance matrix:
//!
//! ```
//! # use leeward::Measurement;
//! let measurement = Measurement::default();
//! let uncertainty = measurement.uncertainty();
//! let vertical_error = uncertainty.covariance[(2, 2)].sqrt();
//! ```
//!
//! The range error of a lidar shot is heavily influenced by topography.
//! To include topography information, set the measurement's normal; this will be used to compute laser incidence angle and topography-induced uncertainty:
//!
//! ```
//! # use leeward::Measurement;
//! use nalgebra::Vector3;
//! let measurement = Measurement::default();
//! measurement.set_normal(Vector3::new(0., 0., 1.));
//! let incidence_angle = measurement.incidence_angle();
//! let uncertainty = measurement.uncertainty();
//! assert!(uncertainty.includes_incidence_angle);
//! ```

mod boresight;
pub mod capi;
mod config;
mod measurement;
mod partial;
mod rotation;
mod trajectory;
mod utils;

pub use boresight::Boresight;
pub use config::{Config, ErrorConfig};
pub use measurement::{Lidar, Measurement, Platform, Projectable, Uncertainty};
pub use partial::{Dimension, Partial, Variable};
pub use rotation::{rotation_matrix, Rotation};
pub use trajectory::Trajectory;
pub use utils::{measurements, read_las, read_sbet};
