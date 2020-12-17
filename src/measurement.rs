use crate::{Config, Trajectory};
use anyhow::{anyhow, Error};
use nalgebra::{Matrix3, Vector3};
use std::path::Path;

/// Reads in a vector of measurements from files.
///
/// # Examples
///
/// ```
/// let measurements = leeward::measurements(
///     "data/sbet.out",
///     "data/points.las",
///     "data/config.toml",
/// ).unwrap();
/// ```
pub fn measurements<P0: AsRef<Path>, P1: AsRef<Path>, P2: AsRef<Path>>(
    sbet: P0,
    las: P1,
    config: P2,
) -> Result<Vec<Measurement>, Error> {
    unimplemented!()
}

/// A measurement combines trajectory information with the lidar point.
#[derive(Debug)]
pub struct Measurement {
    las: las::Point,
    sbet: sbet::Point,
    config: Config,
}

impl Measurement {
    /// Creates a new measurement.
    ///
    /// # Examples
    ///
    /// TODO
    pub fn new(
        trajectory: &Trajectory,
        las: las::Point,
        config: Config,
    ) -> Result<Measurement, Error> {
        let gps_time = las.gps_time.ok_or(anyhow!("missing gps time on point"))?;
        let sbet = trajectory.get(gps_time).ok_or(anyhow!(
            "could not find sbet point for gps time: {}",
            gps_time
        ))?;
        Ok(Measurement {
            las,
            sbet: *sbet,
            config,
        })
    }

    pub fn x(&self) -> f64 {
        self.las.x
    }

    pub fn y(&self) -> f64 {
        self.las.y
    }

    pub fn z(&self) -> f64 {
        self.las.z
    }

    pub fn time(&self) -> f64 {
        self.sbet.time
    }

    pub fn body_frame(&self) -> Vector3<f64> {
        unimplemented!()
    }
}

fn rotation_matrix(roll: f64, pitch: f64, yaw: f64) -> Matrix3<f64> {
    Matrix3::new(
        yaw.cos(),
        -yaw.sin(),
        0.,
        yaw.sin(),
        yaw.cos(),
        0.,
        0.,
        0.,
        1.,
    ) * Matrix3::new(
        pitch.cos(),
        0.,
        pitch.sin(),
        0.,
        1.,
        0.,
        -pitch.sin(),
        0.,
        pitch.cos(),
    ) * Matrix3::new(
        1.,
        0.,
        0.,
        0.,
        roll.cos(),
        -roll.sin(),
        0.,
        roll.sin(),
        roll.cos(),
    )
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    #[test]
    fn body_frame() {
        let measurements =
            super::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let measurement = &measurements[0];
        assert_eq!(400825.80649573973, measurement.time());
        assert_eq!(320024.07, measurement.x());
        assert_eq!(4181361.65, measurement.y());
        assert_eq!(2680.53, measurement.z());
        let body_frame = measurement.body_frame();
        assert_relative_eq!(-396.095, body_frame.x);
        assert_relative_eq!(1741.869, body_frame.y);
        assert_relative_eq!(4295.007, body_frame.z);
    }
}
