use crate::{Config, GeodeticConverter, GeodeticPoint, Trajectory, WGS_84};
use anyhow::{anyhow, Error};
use nalgebra::{Matrix3, Vector3};

/// A measurement combines trajectory information with the lidar point.
pub struct Measurement {
    las: las::Point,
    sbet: sbet::Point,
    body_frame: Vector3<f64>,
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
        let converter = GeodeticConverter::new(config.utm_zone)?;
        let geodetic_las = converter.convert(&las)?;
        let geocentric_las = geodetic_las.to_ecef(WGS_84); // TODO this should be configurable
        let geodetic_sbet = GeodeticPoint::from(sbet);
        let geocentric_sbet = geodetic_sbet.to_ecef(WGS_84);
        let navigation_frame = geodetic_sbet.to_navigation_frame(geocentric_las - geocentric_sbet);
        let body_frame =
            rotation_matrix(sbet.roll, sbet.pitch, sbet.yaw).transpose() * navigation_frame;
        Ok(Measurement {
            las,
            sbet: *sbet,
            body_frame,
        })
    }

    pub fn las(&self) -> &las::Point {
        &self.las
    }

    pub fn time(&self) -> f64 {
        self.sbet.time
    }

    pub fn body_frame(&self) -> Vector3<f64> {
        self.body_frame
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
