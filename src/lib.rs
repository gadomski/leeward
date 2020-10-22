//! Calculates total propagated uncertainty (TPU) for lidar data.
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
//! # Step 2: Create a measurement
//!
//! A `Measurement` is a combination of a trajectory point, a lidar point, and platform configuration.
//! The configuration can be read from a toml file:
//!
//! ```
//! use leeward::Config;
//! let config = Config::from_path("examples/config.toml").unwrap();
//! ```
//!
//! The lidar point can be read from a las file:
//!
//! ```
//! use las::{Reader, Read};
//! let mut reader = Reader::from_path("examples/one-point.las").unwrap();
//! let point = reader.points().next().unwrap().unwrap();
//! ```
//!
//! The trajectory is then used to create the measurement:
//!
//! ```
//! # use leeward::Trajectory;
//! # let trajectory = Trajectory::from_path("examples/sbet.out").unwrap();
//! # use leeward::Config;
//! # let config = Config::from_path("examples/config.toml").unwrap();
//! # use las::{Reader, Read};
//! # let mut reader = Reader::from_path("examples/one-point.las").unwrap();
//! # let point = reader.points().next().unwrap().unwrap();
//! let measurement = trajectory.measurement(point, config).unwrap();
//! ```
//!
//! # Step 3: Calculate the total propagated uncertainty (TPU)
//!
//! Getting the TPU covariance matrix for a measurement is easy:
//!
//! ```
//! # use leeward::Trajectory;
//! # let trajectory = Trajectory::from_path("examples/sbet.out").unwrap();
//! # use leeward::Config;
//! # let config = Config::from_path("examples/config.toml").unwrap();
//! # use las::{Reader, Read};
//! # let mut reader = Reader::from_path("examples/one-point.las").unwrap();
//! # let point = reader.points().next().unwrap().unwrap();
//! # let measurement = trajectory.measurement(point, config).unwrap();
//! let covariance = measurement.tpu();
//! ```
//!
//! The diagonal of the covariance matrix is the x, y, and z variances respectively.
//! To get uncertainty in terms of standard deviation, take the appropriate square root:
//!
//! ```
//! # use leeward::Trajectory;
//! # let trajectory = Trajectory::from_path("examples/sbet.out").unwrap();
//! # use leeward::Config;
//! # let config = Config::from_path("examples/config.toml").unwrap();
//! # use las::{Reader, Read};
//! # let mut reader = Reader::from_path("examples/one-point.las").unwrap();
//! # let point = reader.points().next().unwrap().unwrap();
//! # let measurement = trajectory.measurement(point, config).unwrap();
//! # let covariance = measurement.tpu();
//! let sigma_x = covariance[(0, 0)].sqrt();
//! let sigma_y = covariance[(1, 1)].sqrt();
//! let sigma_horizontal = (covariance[(0, 0)] + covariance[(1, 1)]).sqrt();
//! let sigma_z = covariance[(2, 2)].sqrt();
//! let sigma_magnitude = (covariance[(0, 0)] + covariance[(1, 1)] + covariance[(2, 2)]).sqrt();
//! ```

#![warn(missing_docs)]

pub mod app;
#[cfg(feature = "capi")]
pub mod capi;
mod config;
mod geometry;
mod lidar;
mod partials;
mod trajectory;

pub use config::Config;
pub use geometry::{Point, Rotation, Vector};
pub use lidar::Measurement;
pub use trajectory::Trajectory;
