use crate::{Config, Dimension, Variable};
use anyhow::{anyhow, Error};
use nalgebra::{Matrix3, Vector3};
use serde::Serialize;

/// A lidar measurement.
///
/// More than just a point, a `Measurement` contains system configuration and platform orientation.
#[derive(Clone, Debug, Default)]
pub struct Measurement {
    lidar: Lidar,
    platform: Platform,
    config: Config,
    normal: Option<Vector3<f64>>,
}

#[derive(Debug, Serialize)]
pub struct Variables {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub gnss_x: f64,
    pub gnss_y: f64,
    pub gnss_z: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub boresight_roll: f64,
    pub boresight_pitch: f64,
    pub boresight_yaw: f64,
    pub range: f64,
    pub scan_angle: f64,
    pub lever_arm_x: f64,
    pub lever_arm_y: f64,
    pub lever_arm_z: f64,
}

/// A lidar measurement.
///
/// For now, this is almost always derived from a las point, but it doesn't have to be.
#[derive(Clone, Copy, Debug, Default)]
pub struct Lidar {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub scan_angle: Option<f64>,
}

/// A gnss+ins measurement.
#[derive(Clone, Copy, Debug, Default)]
pub struct Platform {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

/// Measurement uncertainty.
#[derive(Debug)]
pub struct Uncertainty {
    pub covariance: Matrix3<f64>,
    pub x: f64,
    pub y: f64,
    pub horizontal: f64,
    pub vertical: f64,
    pub total: f64,
    pub incidence_angle: Option<f64>,
}

/// A trait implemented by things that can be turned into a projected (UTM) Gnss+Ins measurement.
pub trait Projectable {
    /// Project this into a UTM Platform measurement.
    ///
    /// # Examples
    ///
    /// `Projectable` is implemented for `sbet::Point`.
    ///
    /// ```
    /// use leeward::Projectable;
    /// let points = leeward::read_sbet("data/sbet.out").unwrap();
    /// let projected = points[0].project(11);
    /// ```
    fn project(&self, utm_zone: u8) -> Platform;
}

impl Measurement {
    /// Creates a new measurement from a lidar point, a gnss+ins measurement, and a configuration.
    pub fn new<L: Into<Lidar>, P: Projectable>(
        lidar: L,
        projectable: P,
        config: Config,
    ) -> Measurement {
        let lidar = lidar.into();
        let platform = projectable.project(config.utm_zone);
        Measurement {
            lidar,
            platform,
            config,
            normal: None,
        }
    }

    /// Sets the lidar point for this measurement.
    pub fn set_lidar<L: Into<Lidar>>(&mut self, lidar: L) {
        self.lidar = lidar.into();
    }

    /// Sets the config for this measurement.
    pub fn set_config(&mut self, config: Config) {
        self.config = config;
    }

    /// Sets the normal for this measurement.
    ///
    /// Should be calculated from some sort of curvature measurement, e.g. PDAL's `filters.normal`.
    pub fn set_normal(&mut self, x: f64, y: f64, z: f64) {
        self.normal = Some(Vector3::new(x, y, z));
    }

    /// Returns the measured point, i.e. the las point.
    pub fn measured_point(&self) -> Vector3<f64> {
        Vector3::new(self.lidar.x, self.lidar.y, self.lidar.z)
    }

    /// Returns this measurement's projected gnss point.
    pub fn gnss(&self) -> Vector3<f64> {
        Vector3::new(self.platform.x, self.platform.y, self.platform.z)
    }

    /// Returns this measurement's roll, pitch, and yaw as a rotation matrix.
    pub fn imu(&self) -> Matrix3<f64> {
        crate::rotation_matrix(self.platform.roll, self.platform.pitch, self.platform.yaw)
    }

    /// Returns the measured point in body frame.
    pub fn measured_point_in_body_frame(&self) -> Vector3<f64> {
        self.imu().transpose()
            * self.ned_to_enu().transpose()
            * (self.measured_point() - self.gnss())
    }

    /// Returns the measured point in the scanner's coordinate system.
    pub fn scanner_point(&self) -> Result<Vector3<f64>, Error> {
        let range = self.range();
        let scan_angle = self.scan_angle()?;
        Ok(Vector3::new(
            range * scan_angle.cos(),
            0.,
            range * scan_angle.sin(),
        ))
    }

    /// Returns this measurement's range.
    ///
    /// Range is derived from the vector distance between the measured (lidar) point and the scanner's origin.
    pub fn range(&self) -> f64 {
        (self.measured_point() - self.scanner_origin()).norm()
    }

    /// Returns this measurement's scanner origin in global coordinates.
    pub fn scanner_origin(&self) -> Vector3<f64> {
        self.gnss() + self.ned_to_enu() * self.imu() * (-self.lever_arm())
    }

    fn ned_to_enu(&self) -> Matrix3<f64> {
        Matrix3::new(0., 1., 0., 1., 0., 0., 0., 0., -1.)
    }

    fn lever_arm(&self) -> Vector3<f64> {
        self.config.lever_arm
    }

    fn boresight(&self) -> Matrix3<f64> {
        self.config.boresight.to_rotation_matrix()
    }

    /// Returns this measurement's scan angle.
    ///
    /// The scan angle can be derived from the orientation of the platform, or from the lidar point itself.
    pub fn scan_angle(&self) -> Result<f64, Error> {
        if self.config.derive_scan_angle {
            let point = self.measured_point_in_scanner_frame();
            Ok((point.z / self.range()).asin())
        } else if let Some(scan_angle) = self.lidar.scan_angle {
            Ok(scan_angle)
        } else {
            Err(anyhow!("derive_scan_angle=false and no scan angle"))
        }
    }

    /// Returns the measured point in the scanner frame.
    pub fn measured_point_in_scanner_frame(&self) -> Vector3<f64> {
        self.boresight().transpose() * (self.measured_point_in_body_frame() + self.lever_arm())
    }

    /// Returns the point as calculated from the lidar equation.
    pub fn calculated_point(&self) -> Result<Vector3<f64>, Error> {
        let scanner_point = self.scanner_point()?;
        Ok(self.gnss()
            + self.ned_to_enu()
                * self.imu()
                * (self.boresight() * scanner_point - self.lever_arm()))
    }

    /// Returns the uncertainty structure for this measurement.
    pub fn uncertainty(&self) -> Result<Uncertainty, Error> {
        use nalgebra::{MatrixMN, MatrixN, U14, U3};

        let mut jacobian = MatrixMN::<f64, U3, U14>::zeros();
        let mut errors = MatrixN::<f64, U14>::zeros();
        for (col, variable) in Variable::all().into_iter().enumerate() {
            for (row, dimension) in Dimension::all().into_iter().enumerate() {
                jacobian[(row, col)] = self.partial((dimension, variable))?;
            }
            errors[(col, col)] = self.error(variable).powi(2);
        }
        let covariance = &jacobian * errors * jacobian.transpose();
        Ok(Uncertainty {
            covariance,
            x: covariance[(0, 0)].sqrt(),
            y: covariance[(1, 1)].sqrt(),
            horizontal: (covariance[(0, 0)] + covariance[(1, 1)]).sqrt(),
            vertical: covariance[(2, 2)].sqrt(),
            total: (covariance[(0, 0)] + covariance[(1, 1)] + covariance[(2, 2)]).sqrt(),
            incidence_angle: self.incidence_angle(),
        })
    }

    /// Returns this measurement's error for the given variable.
    pub fn error(&self, variable: Variable) -> f64 {
        match variable {
            Variable::GnssX => self.config.error.gnss.x,
            Variable::GnssY => self.config.error.gnss.y,
            Variable::GnssZ => self.config.error.gnss.z,
            Variable::ImuRoll => self.config.error.imu.roll,
            Variable::ImuPitch => self.config.error.imu.pitch,
            Variable::ImuYaw => self.config.error.imu.yaw,
            Variable::BoresightRoll => self.config.error.boresight.roll,
            Variable::BoresightPitch => self.config.error.boresight.pitch,
            Variable::BoresightYaw => self.config.error.boresight.yaw,
            Variable::Range => {
                if let Some(incidence_angle) = self.incidence_angle() {
                    (self.config.error.range.powi(2)
                        + (self.range() * self.config.error.beam_divergence / 4.0
                            * incidence_angle.tan()))
                    .sqrt()
                } else {
                    self.config.error.range
                }
            }
            Variable::ScanAngle => (self.config.error.angular_resolution.powi(2)
                + (self.config.error.beam_divergence / 4.0).powi(2))
            .sqrt(),
            Variable::LeverArmX => self.config.error.lever_arm.x,
            Variable::LeverArmY => self.config.error.lever_arm.y,
            Variable::LeverArmZ => self.config.error.lever_arm.z,
        }
    }

    /// Returns the partial derivative of this measurement.
    pub fn partial<P: Into<(Dimension, Variable)>>(&self, partial: P) -> Result<f64, Error> {
        let cr = self.platform.roll.cos();
        let sr = self.platform.roll.sin();
        let cp = self.platform.pitch.cos();
        let sp = self.platform.pitch.sin();
        let cy = self.platform.yaw.cos();
        let sy = self.platform.yaw.sin();
        let cbr = self.config.boresight.roll.cos();
        let sbr = self.config.boresight.roll.sin();
        let cbp = self.config.boresight.pitch.cos();
        let sbp = self.config.boresight.pitch.sin();
        let cby = self.config.boresight.yaw.cos();
        let sby = self.config.boresight.yaw.sin();
        let scan_angle = self.scan_angle()?;
        let ca = scan_angle.cos();
        let sa = scan_angle.sin();
        let d = self.range();
        let lx = self.config.lever_arm.x;
        let ly = self.config.lever_arm.y;
        let lz = self.config.lever_arm.z;
        Ok(match partial.into() {
            (Dimension::X, Variable::GnssX) => 1.,
            (Dimension::Y, Variable::GnssX) => 0.,
            (Dimension::Z, Variable::GnssX) => 0.,
            (Dimension::X, Variable::GnssY) => 0.,
            (Dimension::Y, Variable::GnssY) => 1.,
            (Dimension::Z, Variable::GnssY) => 0.,
            (Dimension::X, Variable::GnssZ) => 0.,
            (Dimension::Y, Variable::GnssZ) => 0.,
            (Dimension::Z, Variable::GnssZ) => 1.,
            (Dimension::X, Variable::ImuRoll) => {
                cp * cr * (-ca * d * (-cbr * cby * sbp + sbr * sby) - cbp * cbr * d * sa + lz)
                    + (cr * cy * sp - sr * sy) * (ca * cbp * cby * d + d * sa * sbp - lx)
                    + (-cr * sp * sy - cy * sr)
                        * (ca * d * (cbr * sby + cby * sbp * sbr) - cbp * d * sa * sbr - ly)
            }
            (Dimension::Y, Variable::ImuRoll) => 0.,
            (Dimension::Z, Variable::ImuRoll) => {
                -cp * sr * (-ca * d * (-cbr * cby * sbp + sbr * sby) - cbp * cbr * d * sa + lz)
                    + (-cr * cy + sp * sr * sy)
                        * (ca * d * (cbr * sby + cby * sbp * sbr) - cbp * d * sa * sbr - ly)
                    + (-cr * sy - cy * sp * sr) * (ca * cbp * cby * d + d * sa * sbp - lx)
            }
            (Dimension::X, Variable::ImuPitch) => {
                cp * cy * sr * (ca * cbp * cby * d + d * sa * sbp - lx)
                    - cp * sr
                        * sy
                        * (ca * d * (cbr * sby + cby * sbp * sbr) - cbp * d * sa * sbr - ly)
                    - sp * sr * (-ca * d * (-cbr * cby * sbp + sbr * sby) - cbp * cbr * d * sa + lz)
            }
            (Dimension::Y, Variable::ImuPitch) => {
                cp * (ca * d * (-cbr * cby * sbp + sbr * sby) + cbp * cbr * d * sa - lz)
                    - cy * sp * (ca * cbp * cby * d + d * sa * sbp - lx)
                    - sp * sy * (-ca * d * (cbr * sby + cby * sbp * sbr) + cbp * d * sa * sbr + ly)
            }
            (Dimension::Z, Variable::ImuPitch) => {
                cp * cr * cy * (ca * cbp * cby * d + d * sa * sbp - lx)
                    - cp * cr
                        * sy
                        * (ca * d * (cbr * sby + cby * sbp * sbr) - cbp * d * sa * sbr - ly)
                    - cr * sp * (-ca * d * (-cbr * cby * sbp + sbr * sby) - cbp * cbr * d * sa + lz)
            }
            (Dimension::X, Variable::ImuYaw) => {
                (cr * cy - sp * sr * sy) * (ca * cbp * cby * d + d * sa * sbp - lx)
                    + (-cr * sy - cy * sp * sr)
                        * (ca * d * (cbr * sby + cby * sbp * sbr) - cbp * d * sa * sbr - ly)
            }
            (Dimension::Y, Variable::ImuYaw) => {
                cp * cy * (-ca * d * (cbr * sby + cby * sbp * sbr) + cbp * d * sa * sbr + ly)
                    - cp * sy * (ca * cbp * cby * d + d * sa * sbp - lx)
            }
            (Dimension::Z, Variable::ImuYaw) => {
                (-cr * cy * sp + sr * sy)
                    * (ca * d * (cbr * sby + cby * sbp * sbr) - cbp * d * sa * sbr - ly)
                    + (-cr * sp * sy - cy * sr) * (ca * cbp * cby * d + d * sa * sbp - lx)
            }
            (Dimension::X, Variable::BoresightRoll) => {
                cp * sr * (-ca * d * (cbr * sby + cby * sbp * sbr) + cbp * d * sa * sbr)
                    + (cr * cy - sp * sr * sy)
                        * (ca * d * (cbr * cby * sbp - sbr * sby) - cbp * cbr * d * sa)
            }
            (Dimension::Y, Variable::BoresightRoll) => {
                cp * sy * (-ca * d * (cbr * cby * sbp - sbr * sby) + cbp * cbr * d * sa)
                    + sp * (ca * d * (cbr * sby + cby * sbp * sbr) - cbp * d * sa * sbr)
            }
            (Dimension::Z, Variable::BoresightRoll) => {
                cp * cr * (-ca * d * (cbr * sby + cby * sbp * sbr) + cbp * d * sa * sbr)
                    + (ca * d * (cbr * cby * sbp - sbr * sby) - cbp * cbr * d * sa)
                        * (-cr * sp * sy - cy * sr)
            }
            (Dimension::X, Variable::BoresightPitch) => {
                cp * sr * (ca * cbp * cbr * cby * d + cbr * d * sa * sbp)
                    + (cr * cy - sp * sr * sy) * (ca * cbp * cby * d * sbr + d * sa * sbp * sbr)
                    + (cr * sy + cy * sp * sr) * (-ca * cby * d * sbp + cbp * d * sa)
            }
            (Dimension::Y, Variable::BoresightPitch) => {
                cp * cy * (-ca * cby * d * sbp + cbp * d * sa)
                    + cp * sy * (-ca * cbp * cby * d * sbr - d * sa * sbp * sbr)
                    + sp * (-ca * cbp * cbr * cby * d - cbr * d * sa * sbp)
            }
            (Dimension::Z, Variable::BoresightPitch) => {
                cp * cr * (ca * cbp * cbr * cby * d + cbr * d * sa * sbp)
                    + (cr * cy * sp - sr * sy) * (-ca * cby * d * sbp + cbp * d * sa)
                    + (-cr * sp * sy - cy * sr) * (ca * cbp * cby * d * sbr + d * sa * sbp * sbr)
            }
            (Dimension::X, Variable::BoresightYaw) => {
                -ca * cbp * d * sby * (cr * sy + cy * sp * sr)
                    - ca * cp * d * sr * (cbr * sbp * sby + cby * sbr)
                    + ca * d * (cbr * cby - sbp * sbr * sby) * (cr * cy - sp * sr * sy)
            }
            (Dimension::Y, Variable::BoresightYaw) => {
                -ca * cbp * cp * cy * d * sby - ca * cp * d * sy * (cbr * cby - sbp * sbr * sby)
                    + ca * d * sp * (cbr * sbp * sby + cby * sbr)
            }
            (Dimension::Z, Variable::BoresightYaw) => {
                -ca * cbp * d * sby * (cr * cy * sp - sr * sy)
                    - ca * cp * cr * d * (cbr * sbp * sby + cby * sbr)
                    + ca * d * (cbr * cby - sbp * sbr * sby) * (-cr * sp * sy - cy * sr)
            }
            (Dimension::X, Variable::Range) => {
                cp * sr * (-ca * (-cbr * cby * sbp + sbr * sby) - cbp * cbr * sa)
                    + (ca * (cbr * sby + cby * sbp * sbr) - cbp * sa * sbr)
                        * (cr * cy - sp * sr * sy)
                    + (cr * sy + cy * sp * sr) * (ca * cbp * cby + sa * sbp)
            }
            (Dimension::Y, Variable::Range) => {
                cp * cy * (ca * cbp * cby + sa * sbp)
                    + cp * sy * (-ca * (cbr * sby + cby * sbp * sbr) + cbp * sa * sbr)
                    + sp * (ca * (-cbr * cby * sbp + sbr * sby) + cbp * cbr * sa)
            }
            (Dimension::Z, Variable::Range) => {
                cp * cr * (-ca * (-cbr * cby * sbp + sbr * sby) - cbp * cbr * sa)
                    + (ca * (cbr * sby + cby * sbp * sbr) - cbp * sa * sbr)
                        * (-cr * sp * sy - cy * sr)
                    + (ca * cbp * cby + sa * sbp) * (cr * cy * sp - sr * sy)
            }
            (Dimension::X, Variable::ScanAngle) => {
                cp * sr * (-ca * cbp * cbr * d + d * sa * (-cbr * cby * sbp + sbr * sby))
                    + (cr * cy - sp * sr * sy)
                        * (-ca * cbp * d * sbr - d * sa * (cbr * sby + cby * sbp * sbr))
                    + (cr * sy + cy * sp * sr) * (ca * d * sbp - cbp * cby * d * sa)
            }
            (Dimension::Y, Variable::ScanAngle) => {
                cp * cy * (ca * d * sbp - cbp * cby * d * sa)
                    + cp * sy * (ca * cbp * d * sbr + d * sa * (cbr * sby + cby * sbp * sbr))
                    + sp * (ca * cbp * cbr * d - d * sa * (-cbr * cby * sbp + sbr * sby))
            }
            (Dimension::Z, Variable::ScanAngle) => {
                cp * cr * (-ca * cbp * cbr * d + d * sa * (-cbr * cby * sbp + sbr * sby))
                    + (ca * d * sbp - cbp * cby * d * sa) * (cr * cy * sp - sr * sy)
                    + (-cr * sp * sy - cy * sr)
                        * (-ca * cbp * d * sbr - d * sa * (cbr * sby + cby * sbp * sbr))
            }
            (Dimension::X, Variable::LeverArmX) => -cr * sy - cy * sp * sr,
            (Dimension::Y, Variable::LeverArmX) => -cp * cy,
            (Dimension::Z, Variable::LeverArmX) => -cr * cy * sp + sr * sy,
            (Dimension::X, Variable::LeverArmY) => -cr * cy + sp * sr * sy,
            (Dimension::Y, Variable::LeverArmY) => cp * sy,
            (Dimension::Z, Variable::LeverArmY) => cr * sp * sy + cy * sr,
            (Dimension::X, Variable::LeverArmZ) => cp * sr,
            (Dimension::Y, Variable::LeverArmZ) => -sp,
            (Dimension::Z, Variable::LeverArmZ) => cp * cr,
        })
    }

    pub fn incidence_angle(&self) -> Option<f64> {
        self.normal.map(|normal| {
            let laser_direction = self.laser_direction();
            (laser_direction.dot(&-normal) / (normal.norm() * laser_direction.norm())).acos()
        })
    }

    fn laser_direction(&self) -> Vector3<f64> {
        let laser_vector = self.measured_point() - self.scanner_origin();
        laser_vector / laser_vector.norm()
    }

    pub fn finite_difference<P: Into<(Dimension, Variable)>>(&self, _partial: P) -> Option<f64> {
        unimplemented!()
    }

    pub fn calculated_point_in_body_frame(&self) -> Result<Vector3<f64>, Error> {
        Ok(self.boresight() * self.scanner_point()? + self.lever_arm())
    }

    pub fn variables(&self) -> Result<Variables, Error> {
        let las = self.measured_point();
        let gnss = self.gnss();
        Ok(Variables {
            x: las.x,
            y: las.y,
            z: las.z,
            gnss_x: gnss.x,
            gnss_y: gnss.y,
            gnss_z: gnss.z,
            roll: self.platform.roll,
            pitch: self.platform.pitch,
            yaw: self.platform.yaw,
            boresight_roll: self.config.boresight.roll,
            boresight_pitch: self.config.boresight.pitch,
            boresight_yaw: self.config.boresight.yaw,
            range: self.range(),
            scan_angle: self.scan_angle()?,
            lever_arm_x: self.config.lever_arm.x,
            lever_arm_y: self.config.lever_arm.y,
            lever_arm_z: self.config.lever_arm.z,
        })
    }
}

impl Lidar {
    pub fn new(x: f64, y: f64, z: f64, scan_angle: Option<f64>) -> Lidar {
        Lidar {
            x,
            y,
            z,
            scan_angle,
        }
    }
}

impl From<&las::Point> for Lidar {
    fn from(las: &las::Point) -> Lidar {
        Lidar {
            x: las.x,
            y: las.y,
            z: las.z,
            scan_angle: Some(f64::from(las.scan_angle.to_radians())),
        }
    }
}

impl From<las::Point> for Lidar {
    fn from(las: las::Point) -> Lidar {
        Lidar::from(&las)
    }
}

impl Platform {
    pub fn new(x: f64, y: f64, z: f64, roll: f64, pitch: f64, yaw: f64) -> Platform {
        Platform {
            x,
            y,
            z,
            roll,
            pitch,
            yaw,
        }
    }
}

impl Projectable for &sbet::Point {
    fn project(&self, utm_zone: u8) -> Platform {
        let (northing, easting, _) =
            utm::radians_to_utm_wgs84(self.latitude, self.longitude, utm_zone);
        Platform {
            x: easting,
            y: northing,
            z: self.altitude,
            roll: self.roll,
            pitch: self.pitch,
            yaw: self.yaw,
        }
    }
}

impl Projectable for sbet::Point {
    fn project(&self, utm_zone: u8) -> Platform {
        (&self).project(utm_zone)
    }
}

impl Projectable for Platform {
    fn project(&self, _utm_zone: u8) -> Platform {
        *self
    }
}

#[cfg(test)]
mod tests {
    use super::{Lidar, Measurement, Platform};
    use crate::Config;
    use approx::assert_relative_eq;
    use nalgebra::Vector3;

    #[test]
    fn from_las_and_sbet() {
        let las = crate::read_las("data/points.las").unwrap();
        let sbet = crate::read_sbet("data/sbet.out").unwrap();
        let config = Config::from_path("data/config.toml").unwrap();
        let _ = Measurement::new(&las[0], sbet[0], config);
    }

    #[test]
    fn scan_angle() {
        let mut config = Config::default();
        config.derive_scan_angle = true;
        let lidar = Lidar::new(0., 0., 0., Some(45f64.to_radians()));
        let platform = Platform::new(0., 0., 1., 0., 0., 0.);
        let mut config = Config::default();
        let mut measurement = Measurement::new(lidar, platform, config);
        assert_relative_eq!(measurement.scan_angle().unwrap(), 0.);

        config.derive_scan_angle = false;
        measurement.set_config(config);
        assert_relative_eq!(measurement.scan_angle().unwrap(), 45f64.to_radians());

        let lidar = Lidar::new(0., 0., 0., None);
        measurement.set_lidar(lidar);
        assert!(measurement.scan_angle().is_err());
    }

    #[test]
    fn measured_point_in_scanner_frame() {
        let lidar = Lidar::new(0., 0., 0., Some(45f64.to_radians()));
        let platform = Platform::new(0., 0., 1., 0., 0., 0.);
        let config = Config::default();
        let measurement = Measurement::new(lidar, platform, config);
        assert_relative_eq!(
            Vector3::new(1., 0., 0.),
            measurement.measured_point_in_scanner_frame()
        );
    }

    #[test]
    fn incidence_angle() {
        let lidar = Lidar::new(0., 0., 0., None);
        let platform = Platform::new(0., 0., 1., 0., 0., 0.);
        let config = Config::default();
        let mut measurement = Measurement::new(lidar, platform, config);
        assert_eq!(None, measurement.incidence_angle());
        measurement.set_normal(0., 0., 1.);
        assert_relative_eq!(0., measurement.incidence_angle().unwrap());

        measurement.set_normal(0., 0.5f64.sqrt(), 0.5f64.sqrt());
        assert_relative_eq!(
            std::f64::consts::FRAC_PI_4,
            measurement.incidence_angle().unwrap()
        );
    }
}
