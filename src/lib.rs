//! Calculates total propgated uncertainty (TPU) for lidar data.
//!
//! # Step 1: Import a trajectory
//!
//! In order to calculate the uncertainty for each lidar point, you need information about the trajectory of the platform (aka aircraft).
//! Use `leeward::Trajectory`:
//!
//! ```
//! use leeward::Trajectory;
//! let trajectory = Trajectory::from_path("examples/sbet.out").unwrap();
//! ```
//!
//! To use a trajectory for TPU, it must be quantized.
//! This means converting the vector of trajectory points into a hash map, using an integer mapping of the point times as the hash map keys and the points themselves as values.
//! The integer mapping of the keys is, by default, converting the times to centiseconds (10 milliseconds) and rounding to the nearest integer.
//!
//! ```
//! # use leeward::Trajectory;
//! # let trajectory = Trajectory::from_path("examples/sbet.out").unwrap();
//! let quantized_trajectory = trajectory.quantize(100);
//! ```

pub mod app;
#[cfg(feature = "capi")]
pub mod capi;
mod config;
mod geometry;
mod lidar;
mod partials;
mod trajectory;

pub use app::App;
pub use config::Config;
pub use lidar::Measurement;
pub use trajectory::{QuantizedTrajectory, Trajectory};
