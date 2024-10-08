//! Lidar Equation Engine With Already Racked Derivatives.
//!
//!
//! # Body frame
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
//!
//! # TPU
//!
//! ```
//! use leeward::{Point, Matrix3};
//! let measurements = leeward::measurements(
//!     "data/sbet.out",
//!     "data/points.las",
//!     "data/config.toml",
//! ).unwrap();
//! let tpu: Vec<_> = measurements.iter().map(|m| m.tpu(Point::new(0., 0., 1.)).unwrap()).collect();
//! ```

pub mod adjust;
pub mod capi;
mod config;
pub mod convert;
mod measurement;
mod trajectory;
pub mod utils;

pub use adjust::Adjust;
pub use config::Config;
pub use measurement::{decimated_measurements, measurements, Lasish, Measurement};
use serde::{Deserialize, Serialize};
pub use trajectory::Trajectory;

/// A nalgebra vector3 for f64s.
pub type Point = nalgebra::Vector3<f64>;
/// A nalgebra matrix3 for f64s.
pub type Matrix3 = nalgebra::Matrix3<f64>;

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

#[derive(Debug)]
pub struct VariableIter {
    variables: [Variable; 14],
    index: usize,
}

impl Variable {
    pub fn iter() -> VariableIter {
        VariableIter {
            variables: [
                Variable::Range,
                Variable::ScanAngle,
                Variable::BoresightRoll,
                Variable::BoresightPitch,
                Variable::BoresightYaw,
                Variable::LeverArmX,
                Variable::LeverArmY,
                Variable::LeverArmZ,
                Variable::Roll,
                Variable::Pitch,
                Variable::Yaw,
                Variable::GnssX,
                Variable::GnssY,
                Variable::GnssZ,
            ],
            index: 0,
        }
    }
}

impl Iterator for VariableIter {
    type Item = Variable;
    fn next(&mut self) -> Option<Variable> {
        let variable = self.variables.get(self.index);
        if variable.is_some() {
            self.index += 1;
        }
        variable.copied()
    }
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
    pub fn as_matrix(&self) -> Matrix3 {
        let cy = self.yaw.cos();
        let sy = self.yaw.sin();
        let cp = self.pitch.cos();
        let sp = self.pitch.sin();
        let cr = self.roll.cos();
        let sr = self.roll.sin();
        Matrix3::new(
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
