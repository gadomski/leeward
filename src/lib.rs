//! Lidar Equation Engine With Already Racked Derivatives.

mod config;
pub mod convert;
mod measurement;
mod trajectory;

pub use config::Config;
pub use measurement::Measurement;
pub use trajectory::Trajectory;

pub type Point = nalgebra::Vector3<f64>;
