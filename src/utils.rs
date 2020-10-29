use crate::{Config, Measurement, Trajectory};
use anyhow::Error;
use nalgebra::Matrix3;
use std::path::Path;

/// One-stop shop to create a vector of measurements from a trajectory file, point cloud file, and config file.
///
/// Dive deeper into the API to control aspects of the measurement computations.
///
/// # Examples
///
/// ```
/// let measurements = leeward::measurements(
///     "data/sbet.out",
///     "data/points.las",
///     "data/config.toml"
/// ).unwrap();
/// ```
pub fn measurements<P0: AsRef<Path>, P1: AsRef<Path>, P2: AsRef<Path>>(
    sbet: P0,
    las: P1,
    config: P2,
) -> Result<Vec<Measurement>, Error> {
    let trajectory = Trajectory::from_path(sbet)?;
    let config = Config::from_path(config)?;
    trajectory.read_las(las, &config)
}

/// Utility method to read a las file to a vector of points.
///
/// # Examples
///
/// ```
/// let points = leeward::read_las("data/points.las").unwrap();
/// ```
pub fn read_las<P: AsRef<Path>>(path: P) -> Result<Vec<las::Point>, Error> {
    use las::{Read, Reader};
    let mut reader = Reader::from_path(path)?;
    reader
        .points()
        .map(|result| result.map_err(Error::from))
        .collect()
}

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
