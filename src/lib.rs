//! Calculates total propgated uncertainty for lidar data.

#[macro_use]
extern crate anyhow;

pub mod app;
mod config;
mod geometry;
mod lidar;
mod partials;

pub use app::App;
pub use config::Config;
