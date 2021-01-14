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

pub mod adjust;
mod config;
pub mod convert;
mod measurement;
mod trajectory;

pub use adjust::Adjust;
pub use config::Config;
pub use measurement::{decimated_measurements, measurements, Measurement};
use serde::{Deserialize, Serialize};
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
#[derive(PartialEq, Clone, Copy, Debug)]
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

/// Roll, pitch, and yaw.
#[derive(Debug, Serialize, Deserialize, Clone, Copy, PartialEq)]
pub struct RollPitchYaw {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl RollPitchYaw {
    /// Creates a new roll, pitch, and yaw.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::RollPitchYaw;
    /// let rpy = RollPitchYaw::new(0., 0., 0.);
    /// ```
    pub fn new(roll: f64, pitch: f64, yaw: f64) -> RollPitchYaw {
        RollPitchYaw { roll, pitch, yaw }
    }

    /// Returns a rotation matrix from a roll, pitch, and yaw.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::RollPitchYaw;
    /// let rpy = RollPitchYaw::new(0., 0., 0.);
    /// let matrix = rpy.as_matrix();
    /// assert_eq!(matrix[(0, 0)], 1.);
    /// assert_eq!(matrix[(1, 1)], 1.);
    /// assert_eq!(matrix[(2, 2)], 1.);
    /// ```
    pub fn as_matrix(&self) -> Matrix {
        let cy = self.yaw.cos();
        let sy = self.yaw.sin();
        let cp = self.pitch.cos();
        let sp = self.pitch.sin();
        let cr = self.roll.cos();
        let sr = self.roll.sin();
        Matrix::new(
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            -sp,
            cp * sr,
            cp * cr,
        )
    }
}
