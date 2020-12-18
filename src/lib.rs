//! Lidar Equation Engine With Already Racked Derivatives.

mod config;
pub mod convert;
mod measurement;
mod trajectory;

pub use config::Config;
pub use measurement::{measurements, Measurement};
pub use trajectory::Trajectory;

/// A nalgebra vector3 for f64s.
pub type Point = nalgebra::Vector3<f64>;
/// A nalgebra matrix3 for f64s.
pub type Matrix = nalgebra::Matrix3<f64>;
