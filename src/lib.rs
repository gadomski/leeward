//! Calculates total propgated uncertainty for lidar data.
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
pub use trajectory::Trajectory;
