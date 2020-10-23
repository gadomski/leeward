use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use std::ops::Sub;
use std::vec::IntoIter;

/// A three dimensional vector.
#[derive(Debug, Clone, Copy, Deserialize, Serialize, Default)]
#[allow(missing_docs)]
pub struct Vector {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
/// A point is just a vector by another name.
pub type Point = Vector;

/// A three dimensional rotation.
#[derive(Debug, Clone, Copy, Deserialize, Serialize, Default)]
#[allow(missing_docs)]
pub struct Rotation {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl Vector {
    /// Calculates the L2 norm for this vector.
    pub fn norm(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    /// Returns an iterator over this vector/point's values, x then y then z.
    pub fn into_iter(self) -> IntoIter<f64> {
        vec![self.x, self.y, self.z].into_iter()
    }
}

impl From<Point> for Vector3<f64> {
    fn from(point: Point) -> Vector3<f64> {
        Vector3::new(point.x, point.y, point.z)
    }
}

impl From<Vector3<f64>> for Point {
    fn from(vector: Vector3<f64>) -> Point {
        Point {
            x: vector[0],
            y: vector[1],
            z: vector[2],
        }
    }
}

impl Sub<Point> for Point {
    type Output = Vector;
    fn sub(self, other: Point) -> Vector {
        Vector {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl From<Rotation> for Matrix3<f64> {
    /// Converts this rotation into a matrix following yaw, pitch, roll rotations.
    ///
    /// Technically the rotation is an intrinsic Z -> Y' -> X'' rotation, but
    /// that's the same as an extrinsic X -> Y -> Z as given below.
    fn from(rotation: Rotation) -> Matrix3<f64> {
        let c1 = rotation.roll.cos();
        let s1 = rotation.roll.sin();
        let c2 = rotation.pitch.cos();
        let s2 = rotation.pitch.sin();
        let c3 = rotation.yaw.cos();
        let s3 = rotation.yaw.sin();
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
}

#[cfg(test)]
mod test {
    use super::*;
    use approx::assert_relative_eq;

    fn check_matrix(roll: f64, pitch: f64, yaw: f64, matrix: Vec<f64>) {
        let matrix = Matrix3::from_iterator(matrix);
        let rotation = Matrix3::from(Rotation { roll, pitch, yaw });
        assert_relative_eq!(rotation[(0, 0)], matrix[(0, 0)]);
        assert_relative_eq!(rotation[(0, 1)], matrix[(0, 1)]);
        assert_relative_eq!(rotation[(0, 2)], matrix[(0, 2)]);
        assert_relative_eq!(rotation[(1, 0)], matrix[(1, 0)]);
        assert_relative_eq!(rotation[(1, 1)], matrix[(1, 1)]);
        assert_relative_eq!(rotation[(1, 2)], matrix[(1, 2)]);
        assert_relative_eq!(rotation[(2, 0)], matrix[(2, 0)]);
        assert_relative_eq!(rotation[(2, 1)], matrix[(2, 1)]);
        assert_relative_eq!(rotation[(2, 2)], matrix[(2, 2)]);
    }

    #[test]
    fn rotations() {
        check_matrix(
            90f64.to_radians(),
            0.,
            0.,
            vec![1., 0., 0., 0., 0., 1., 0., -1., 0.],
        );
        check_matrix(
            -90f64.to_radians(),
            0.,
            -90f64.to_radians(),
            vec![0., 0., 1., 1., 0., 0., 0., 1., 0.],
        );
    }
}
