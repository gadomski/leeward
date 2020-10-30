use nalgebra::Matrix3;
use serde::Deserialize;

/// Converts this rotation into a matrix following yaw, pitch, roll rotations.
///
/// Technically the rotation is an intrinsic Z -> Y' -> X'' rotation, but
/// that's the same as an extrinsic X -> Y -> Z as given below.
pub fn rotation_matrix(roll: f64, pitch: f64, yaw: f64) -> Matrix3<f64> {
    let c1 = roll.cos();
    let s1 = roll.sin();
    let c2 = pitch.cos();
    let s2 = pitch.sin();
    let c3 = yaw.cos();
    let s3 = yaw.sin();
    Matrix3::new(
        c2 * c3,
        -c2 * s3,
        s2,
        c1 * s3 + c3 * s1 * s2,
        c1 * c3 - s1 * s2 * s3,
        -c2 * s1,
        s1 * s3 - c1 * c3 * s2,
        c3 * s1 + c1 * s2 * s3,
        c1 * c2,
    )
}

/// Rotation as defined by a roll, pitch, and yaw.
#[derive(Debug, Deserialize, Default, Clone, Copy)]
pub struct Rotation {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl Rotation {
    /// Creates a new rotation with degrees in radians.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Rotation;
    /// let rotation = Rotation::new(0., 0., 0.);
    /// ```
    pub fn new(roll: f64, pitch: f64, yaw: f64) -> Rotation {
        Rotation { roll, pitch, yaw }
    }

    /// Returns this rpy as a rotation matrix.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Rotation;
    /// let rotation = Rotation::new(0., 0., 0.);
    /// let rotation_matrix = rotation.to_rotation_matrix();
    /// ```
    pub fn to_rotation_matrix(&self) -> Matrix3<f64> {
        rotation_matrix(self.roll, self.pitch, self.yaw)
    }

    /// Returns this rotation with a new roll value.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Rotation;
    /// let before = Rotation::new(0., 0., 0.);
    /// let after = before.with_roll(3.14);
    /// assert_eq!(after.roll, 3.14);
    /// ```
    pub fn with_roll(&self, roll: f64) -> Rotation {
        Rotation::new(roll, self.pitch, self.yaw)
    }

    /// Returns this rotation with a new pitch value.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Rotation;
    /// let before = Rotation::new(0., 0., 0.);
    /// let after = before.with_pitch(3.14);
    /// assert_eq!(after.pitch, 3.14);
    /// ```
    pub fn with_pitch(&self, pitch: f64) -> Rotation {
        Rotation::new(self.roll, pitch, self.yaw)
    }

    /// Returns this rotation with a new yaw value.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Rotation;
    /// let before = Rotation::new(0., 0., 0.);
    /// let after = before.with_yaw(3.14);
    /// assert_eq!(after.yaw, 3.14);
    /// ```
    pub fn with_yaw(&self, yaw: f64) -> Rotation {
        Rotation::new(self.roll, self.pitch, yaw)
    }
}
