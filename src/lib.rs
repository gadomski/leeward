//! Lidar Equation Engine With Already Racked Derivatives.

mod boresight;
mod config;
pub mod convert;
mod measurement;
mod trajectory;

pub use boresight::Boresight;
pub use config::{Config, RollPitchYaw};
pub use measurement::{measurements, Measurement};
pub use trajectory::Trajectory;

/// A nalgebra vector3 for f64s.
pub type Point = nalgebra::Vector3<f64>;
/// A nalgebra matrix3 for f64s.
pub type Matrix = nalgebra::Matrix3<f64>;

/// The three dimensions.
#[derive(Clone, Copy, Debug)]
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
    /// assert_eq!(vec![Dimension::X, Dimension::Y, Dimension::Z], Dimension::iter().collect());
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
