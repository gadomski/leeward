use crate::{Config, Dimension, Rotation, Variable};
use anyhow::{anyhow, Error};
use nalgebra::{Matrix3, Vector3};

/// A lidar measurement.
///
/// More than just a point, a `Measurement` contains system configuration and platform orientation information as well.
#[derive(Debug)]
pub struct Measurement {
    las: Vector3<f64>,
    gnss: Vector3<f64>,
    imu: Rotation,
    ned_to_enu: Matrix3<f64>,
    boresight: Rotation,
    lever_arm: Vector3<f64>,
    las_scan_angle: Option<f64>,
}

/// Measurement uncertainty.
#[derive(Debug)]
pub struct Uncertainty {
    covariance: Matrix3<f64>,
}

impl Measurement {
    /// Creates a new measurement from the sbet, las, and config.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Config, Measurement};
    /// let measurement = Measurement::new(&sbet::Point::default(), &las::Point::default(), &Config::default());
    /// ```
    pub fn new(sbet: &sbet::Point, las: &las::Point, config: &Config) -> Measurement {
        let las_scan_angle = if config.use_las_scan_angle {
            Some(f64::from(las.scan_angle.to_radians()))
        } else {
            None
        };
        let (northing, easting, _) =
            utm::radians_to_utm_wgs84(sbet.latitude, sbet.longitude, config.utm_zone);
        let las = Vector3::new(las.x, las.y, las.z);
        let gnss = Vector3::new(easting, northing, sbet.altitude);
        let imu = Rotation::new(sbet.roll, sbet.pitch, sbet.yaw);
        Measurement::new_from_parts(
            las,
            gnss,
            imu,
            config.boresight,
            config.lever_arm,
            las_scan_angle,
        )
    }

    /// Creates a new measurement from the basic parts.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Measurement, Rotation};
    /// # use nalgebra::Vector3;
    /// let las = Vector3::new(1., 2., 3.);
    /// let gnss = Vector3::new(4., 5., 6.);
    /// let imu = Rotation::new(0., 0., 0.);
    /// let boresight = Rotation::new(0., 0., 0.);
    /// let lever_arm = Vector3::new(1., 2., 3.);
    /// let measurement = Measurement::new_from_parts(las, gnss, imu, boresight, lever_arm, None);
    /// ```
    pub fn new_from_parts(
        las: Vector3<f64>,
        gnss: Vector3<f64>,
        imu: Rotation,
        boresight: Rotation,
        lever_arm: Vector3<f64>,
        las_scan_angle: Option<f64>,
    ) -> Measurement {
        Measurement {
            las,
            gnss,
            imu,
            ned_to_enu: Matrix3::new(0., 1., 0., 1., 0., 0., 0., 0., -1.),
            boresight,
            lever_arm,
            las_scan_angle,
        }
    }

    /// Returns the las point.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].las_point();
    /// ```
    pub fn las_point(&self) -> Vector3<f64> {
        self.las
    }

    /// Returns the las point in platform coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].las_platform();
    /// ```
    pub fn las_platform(&self) -> Vector3<f64> {
        self.imu.to_rotation_matrix().transpose()
            * self.ned_to_enu.transpose()
            * (self.las - self.gnss)
    }

    /// Returns this measurement's gnss point in projected coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].gnss_point();
    /// ```
    pub fn gnss_point(&self) -> Vector3<f64> {
        self.gnss
    }

    /// Returns the measured point as calculated through the lidar equation.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].calculated();
    /// ```
    pub fn calculated(&self) -> Vector3<f64> {
        self.gnss
            + self.ned_to_enu
                * self.imu.to_rotation_matrix()
                * (self.boresight.to_rotation_matrix() * self.scanner() - self.lever_arm)
    }

    /// Returns the measured point as calculated through the lidar equation, in platform coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].calculated_platform();
    /// ```
    pub fn calculated_platform(&self) -> Vector3<f64> {
        self.boresight.to_rotation_matrix() * self.scanner() - self.lever_arm
    }

    /// Returns the measured point in the scanner's coordinate system.
    ///
    /// The scan angle can either be derived from the platform's position and orientation, or taken directly from the las file.
    /// Pass `true` if you want to use the derived value, and `false` to use the las scan angle rank.
    /// Generally, use the derived value unless you have some reason not to.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].scanner();
    /// ```
    pub fn scanner(&self) -> Vector3<f64> {
        let range = self.range();
        let scan_angle = self.scan_angle();
        Vector3::new(range * scan_angle.cos(), 0., range * scan_angle.sin())
    }

    /// Returns this measurement's range.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let range = measurements[0].range();
    /// ```
    pub fn range(&self) -> f64 {
        (self.las - self.scanner_origin()).norm()
    }

    /// Returns this measurement's scan angle.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let scan_angle = measurements[0].scan_angle();
    /// ```
    pub fn scan_angle(&self) -> f64 {
        self.las_scan_angle.unwrap_or_else(|| {
            ((self.boresight.to_rotation_matrix().transpose()
                * (self.lever_arm + self.las_platform()))
            .z / self.range())
            .asin()
        })
    }

    /// Returns this measurement's scanner origin in global coordinates.
    ///
    /// Lever arm.
    pub fn scanner_origin(&self) -> Vector3<f64> {
        self.gnss + self.ned_to_enu * self.imu.to_rotation_matrix() * (-self.lever_arm)
    }

    /// Returns the uncertainty structure for this measurement.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let uncertainty = measurements[0].uncertainty();
    /// ```
    pub fn uncertainty(&self) -> Uncertainty {
        use nalgebra::{MatrixMN, MatrixN, U14, U3};

        let mut jacobian = MatrixMN::<f64, U3, U14>::zeros();
        let mut errors = MatrixN::<f64, U14>::zeros();
        for (col, variable) in Variable::all().into_iter().enumerate() {
            for (row, dimension) in Dimension::all().into_iter().enumerate() {
                jacobian[(row, col)] = self.partial((dimension, variable));
            }
            errors[(col, col)] = self.error(variable).powi(2);
        }
        let covariance = &jacobian * errors * jacobian.transpose();
        Uncertainty { covariance }
    }

    /// Returns this measurement's error for the given variable.
    pub fn error(&self, _variable: Variable) -> f64 {
        unimplemented!()
    }

    /// Returns the partial derivative of this measurement.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Dimension, Variable};
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let partial = measurements[0].partial((Dimension::X, Variable::ScanAngle));
    /// ```
    pub fn partial<P: Into<(Dimension, Variable)>>(&self, partial: P) -> f64 {
        let cr = self.imu.roll.cos();
        let sr = self.imu.roll.sin();
        let cp = self.imu.pitch.cos();
        let sp = self.imu.pitch.sin();
        let cy = self.imu.yaw.cos();
        let sy = self.imu.yaw.sin();
        let cbr = self.boresight.roll.cos();
        let sbr = self.boresight.roll.sin();
        let cbp = self.boresight.pitch.cos();
        let sbp = self.boresight.pitch.sin();
        let cby = self.boresight.yaw.cos();
        let sby = self.boresight.yaw.sin();
        let scan_angle = self.scan_angle();
        let ca = scan_angle.cos();
        let sa = scan_angle.sin();
        let d = self.range();
        let lx = self.lever_arm.x;
        let ly = self.lever_arm.y;
        let lz = self.lever_arm.z;
        match partial.into() {
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
        }
    }

    /// Returns the partial derivative as determined by numerical differentiation via the finite difference method.
    ///
    /// Returns `None` if the variable is a derived value, e.g. range which is derived from the position of the las point and the scanner origin.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Dimension, Variable};
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let partial = measurements[0].finite_difference((Dimension::X, Variable::LeverArmX)).unwrap();
    /// ```
    pub fn finite_difference<P: Into<(Dimension, Variable)>>(&self, partial: P) -> Option<f64> {
        let (dimension, variable) = partial.into();
        let h = f64::EPSILON.sqrt() * 100.;
        if let Some((positive, negative)) = self
            .adjust(variable, h)
            .and_then(|p| self.adjust(variable, -h).map(|n| (p, n)))
        {
            Some((positive.value(dimension) - negative.value(dimension)) / (2.0 * h))
        } else {
            None
        }
    }

    /// Creates a new measurement with a new config.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Config;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let measurement = measurements[0].with_new_config(&Config::default());
    /// ```
    pub fn with_new_config(&self, config: &Config) -> Result<Measurement, Error> {
        if config.use_las_scan_angle & self.las_scan_angle.is_none() {
            Err(anyhow!(
                "new config uses the las scan angle, but las scan angle not present on measurement"
            ))
        } else {
            Ok(Measurement::new_from_parts(
                self.las,
                self.gnss,
                self.imu,
                config.boresight,
                config.lever_arm,
                self.las_scan_angle,
            ))
        }
    }

    fn with_new_boresight(&self, boresight: Rotation) -> Measurement {
        Measurement::new_from_parts(
            self.las,
            self.gnss,
            self.imu,
            boresight,
            self.lever_arm,
            self.las_scan_angle,
        )
    }

    fn with_new_imu(&self, imu: Rotation) -> Measurement {
        Measurement::new_from_parts(
            self.las,
            self.gnss,
            imu,
            self.boresight,
            self.lever_arm,
            self.las_scan_angle,
        )
    }

    fn with_new_lever_arm(&self, lever_arm: Vector3<f64>) -> Measurement {
        Measurement::new_from_parts(
            self.las,
            self.gnss,
            self.imu,
            self.boresight,
            lever_arm,
            self.las_scan_angle,
        )
    }

    fn with_new_gnss(&self, gnss: Vector3<f64>) -> Measurement {
        Measurement::new_from_parts(
            self.las,
            gnss,
            self.imu,
            self.boresight,
            self.lever_arm,
            self.las_scan_angle,
        )
    }

    fn adjust(&self, variable: Variable, delta: f64) -> Option<Measurement> {
        match variable {
            Variable::Range | Variable::ScanAngle => None,
            Variable::BoresightRoll => {
                Some(self.with_new_boresight(self.boresight.with_roll(self.boresight.roll + delta)))
            }
            Variable::BoresightPitch => Some(
                self.with_new_boresight(self.boresight.with_pitch(self.boresight.pitch + delta)),
            ),
            Variable::BoresightYaw => {
                Some(self.with_new_boresight(self.boresight.with_yaw(self.boresight.yaw + delta)))
            }
            Variable::ImuRoll => Some(self.with_new_imu(self.imu.with_roll(self.imu.roll + delta))),
            Variable::ImuPitch => {
                Some(self.with_new_imu(self.imu.with_pitch(self.imu.pitch + delta)))
            }
            Variable::ImuYaw => Some(self.with_new_imu(self.imu.with_yaw(self.imu.yaw + delta))),
            Variable::LeverArmX => Some(self.with_new_lever_arm(Vector3::new(
                self.lever_arm.x + delta,
                self.lever_arm.y,
                self.lever_arm.z,
            ))),
            Variable::LeverArmY => Some(self.with_new_lever_arm(Vector3::new(
                self.lever_arm.x,
                self.lever_arm.y + delta,
                self.lever_arm.z,
            ))),
            Variable::LeverArmZ => Some(self.with_new_lever_arm(Vector3::new(
                self.lever_arm.x,
                self.lever_arm.y,
                self.lever_arm.z + delta,
            ))),
            Variable::GnssX => Some(self.with_new_gnss(Vector3::new(
                self.gnss.x + delta,
                self.gnss.y,
                self.gnss.z,
            ))),
            Variable::GnssY => Some(self.with_new_gnss(Vector3::new(
                self.gnss.x,
                self.gnss.y + delta,
                self.gnss.z,
            ))),
            Variable::GnssZ => Some(self.with_new_gnss(Vector3::new(
                self.gnss.x,
                self.gnss.y,
                self.gnss.z + delta,
            ))),
        }
    }

    fn value(&self, dimension: Dimension) -> f64 {
        let calculated = self.calculated();
        match dimension {
            Dimension::X => calculated.x,
            Dimension::Y => calculated.y,
            Dimension::Z => calculated.z,
        }
    }
}
