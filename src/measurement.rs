use crate::{utils, Config, Partial, Uncertainty};
use nalgebra::{Matrix3, Vector3};

/// A lidar measurement.
///
/// More than just a point, a `Measurement` contains system configuration and platform orientation information as well.
#[derive(Debug)]
pub struct Measurement {
    las: Vector3<f64>,
    gnss: Vector3<f64>,
    imu: Matrix3<f64>,
    ned_to_enu: Matrix3<f64>,
    boresight: Matrix3<f64>,
    lever_arm: Vector3<f64>,
    trig: Trig,
}

#[derive(Debug)]
struct Trig {
    cr: f64,
    sr: f64,
    cp: f64,
    sp: f64,
    cy: f64,
    sy: f64,
    cbr: f64,
    sbr: f64,
    cbp: f64,
    sbp: f64,
    cby: f64,
    sby: f64,
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
        let (northing, easting, _) =
            utm::radians_to_utm_wgs84(sbet.latitude, sbet.longitude, config.utm_zone);
        let boresight = config.boresight.to_rotation_matrix();
        Measurement {
            las: Vector3::new(las.x, las.y, las.z),
            gnss: Vector3::new(easting, northing, sbet.altitude),
            imu: utils::rotation_matrix(sbet.roll, sbet.pitch, sbet.yaw),
            ned_to_enu: Matrix3::new(0., 1., 0., 1., 0., 0., 0., 0., -1.),
            boresight,
            lever_arm: config.lever_arm,
            trig: Trig {
                cr: sbet.roll.cos(),
                sr: sbet.roll.sin(),
                cp: sbet.pitch.cos(),
                sp: sbet.pitch.sin(),
                cy: sbet.yaw.cos(),
                sy: sbet.yaw.sin(),
                cbr: config.boresight.roll.cos(),
                sbr: config.boresight.roll.sin(),
                cbp: config.boresight.pitch.cos(),
                sbp: config.boresight.pitch.sin(),
                cby: config.boresight.yaw.cos(),
                sby: config.boresight.yaw.sin(),
            },
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
        self.imu.transpose() * self.ned_to_enu.transpose() * (self.las - self.gnss)
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
    /// let point = measurements[0].gnss_point();
    /// ```
    pub fn calculated(&self) -> Vector3<f64> {
        self.gnss + self.ned_to_enu * self.imu * (self.boresight * self.scanner() - self.lever_arm)
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
        self.boresight * self.scanner() - self.lever_arm
    }

    /// Returns the measured point in the scanner's coordinate system.
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
        ((self.boresight.transpose() * (self.lever_arm + self.las_platform())).z / self.range())
            .asin()
    }

    /// Returns this measurement's scanner origin in global coordinates.
    ///
    /// Lever arm.
    pub fn scanner_origin(&self) -> Vector3<f64> {
        self.gnss + self.ned_to_enu * self.imu * (-self.lever_arm)
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
        Uncertainty {}
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
    pub fn partial<P: Into<Partial>>(&self, partial: P) -> f64 {
        use crate::{Dimension, Variable};
        let cr = self.trig.cr;
        let sr = self.trig.sr;
        let cp = self.trig.cp;
        let sp = self.trig.sp;
        let cy = self.trig.cy;
        let sy = self.trig.sy;
        let cbr = self.trig.cbr;
        let sbr = self.trig.sbr;
        let cbp = self.trig.cbp;
        let sbp = self.trig.sbp;
        let cby = self.trig.cby;
        let sby = self.trig.sby;
        let scan_angle = self.scan_angle();
        let ca = scan_angle.cos();
        let sa = scan_angle.sin();
        let d = self.range();
        let lx = self.lever_arm.x;
        let ly = self.lever_arm.y;
        let lz = self.lever_arm.z;
        let partial: Partial = partial.into();
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
}
