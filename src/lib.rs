//! Lidar Equation Engine With Already Racked Derivatives.
//!
//! Here's some fun stuff you can do:
//!
//! - Use the trajectory to transform the point cloud into the body frame of the plane:
//!
//! ```
//! use leeward::Point;
//! let measurements = leeward::measurements(
//!     "data/sbet.out",
//!     "data/points.las",
//!     "data/config.toml",
//! ).unwrap();
//! let body_frame_coordinates: Vec<Point> = measurements.iter().map(|m| m.body_frame()).collect();
//! ```

mod config;
pub mod convert;
mod measurement;
mod trajectory;

pub use config::{Config, RollPitchYaw};
pub use measurement::{measurements, Measurement};
pub use trajectory::Trajectory;

/// A nalgebra vector3 for f64s.
pub type Point = nalgebra::Vector3<f64>;
/// A nalgebra matrix3 for f64s.
pub type Matrix = nalgebra::Matrix3<f64>;

/// The three dimensions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Dimension {
    X,
    Y,
    Z,
}

impl Dimension {
    /// Iterate over all three dimensions.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Dimension;
    /// let mut iter = Dimension::iter();
    /// assert_eq!(Dimension::X, iter.next().unwrap());
    /// assert_eq!(Dimension::Y, iter.next().unwrap());
    /// assert_eq!(Dimension::Z, iter.next().unwrap());
    /// assert_eq!(None, iter.next());
    /// ```
    pub fn iter() -> DimensionIter {
        DimensionIter::new()
    }
}

/// An iterator over the dimensions.
#[derive(Debug)]
pub struct DimensionIter {
    index: usize,
}

impl DimensionIter {
    fn new() -> DimensionIter {
        DimensionIter { index: 0 }
    }
}

impl Iterator for DimensionIter {
    type Item = Dimension;
    fn next(&mut self) -> Option<Dimension> {
        match self.index {
            0 => {
                self.index += 1;
                Some(Dimension::X)
            }
            1 => {
                self.index += 1;
                Some(Dimension::Y)
            }
            2 => {
                self.index += 1;
                Some(Dimension::Z)
            }
            _ => None,
        }
    }
}

/// The variables in the lidar equation.
#[derive(Clone, Copy, Debug)]
pub enum Variable {
    Range,
    ScanAngle,
    BoresightRoll,
    BoresightPitch,
    BoresightYaw,
    LeverArmX,
    LeverArmY,
    LeverArmZ,
    Roll,
    Pitch,
    Yaw,
    GnssX,
    GnssY,
    GnssZ,
}
