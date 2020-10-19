//! Calculates total propgated uncertainty for lidar data.

#[macro_use]
extern crate anyhow;

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
