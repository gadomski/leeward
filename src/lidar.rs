use crate::geometry::{Point, Rotation, Vector};
use crate::partials::{Dimension, Partial, Variable};
use crate::Config;
use nalgebra::base::dimension::{U14, U3};
use nalgebra::{Matrix3, MatrixMN, Vector3};

/// A lidar measurement.
#[derive(Clone, Debug)]
pub struct Measurement {
    gnss: Point,
    imu: Rotation,
    boresight: Rotation,
    lever_arm: Point,
    scan_angle: f64,
    range: f64,
    las: las::Point,
    sbet: sbet::Point,
    config: Config,
}

#[derive(Debug)]
pub struct PartialCheck {
    pub expected: f64,
    pub partial: f64,
    pub adjustment: f64,
    pub actual: f64,
    pub error: f64,
}

impl Measurement {
    pub fn new(las: las::Point, sbet: sbet::Point, config: Config) -> Measurement {
        let (northing, easting, _) =
            utm::radians_to_utm_wgs84(sbet.latitude, sbet.longitude, config.utm_zone);
        Measurement {
            gnss: Point {
                x: easting,
                y: northing,
                z: sbet.altitude,
            },
            imu: Rotation {
                roll: sbet.roll,
                pitch: sbet.pitch,
                yaw: sbet.yaw,
            },
            boresight: config.boresight,
            range: ((easting - las.x).powi(2)
                + (northing - las.y).powi(2)
                + (sbet.altitude - las.z).powi(2))
            .sqrt(),
            scan_angle: f64::from(las.scan_angle).to_radians(),
            lever_arm: config.lever_arm,
            las: las,
            sbet: sbet,
            config: config,
        }
    }

    /// Returns this measurement's las point.
    pub fn las_point(&self) -> Point {
        Point {
            x: self.las.x,
            y: self.las.y,
            z: self.las.z,
        }
    }

    pub fn range(&self) -> f64 {
        self.range
    }

    pub fn imu(&self) -> Rotation {
        self.imu
    }

    pub fn gnss(&self) -> Point {
        self.gnss
    }

    /// Returns this measurement's backconverted point.
    pub fn backconverted_point(&self) -> Point {
        (Vector3::from(self.gnss)
            + self.platform_to_global_rotation()
                * (Matrix3::from(self.boresight) * self.scanner_vector()
                    - Vector3::from(self.lever_arm)))
        .into()
    }

    fn scanner_vector(&self) -> Vector3<f64> {
        Vector3::new(
            self.range * self.scan_angle.cos(),
            0.,
            self.range * self.scan_angle.sin(),
        )
    }

    /// Returns the gps time of the las point, or an error if there isn't one.
    pub fn gps_time(&self) -> Option<f64> {
        self.las.gps_time
    }

    /// Returns the scan angle as calculated from the position, orientation, and the point.
    pub fn scan_angle(&self) -> f64 {
        unimplemented!()
    }

    /// Returns the las point in the platform coordinates.
    pub fn platform(&self) -> Point {
        (self.platform_to_global_rotation().transpose()
            * Vector3::from(self.las_point() - self.gnss))
        .into()
    }

    /// Returns the scan angle as reported in the las file.
    pub fn las_scan_angle(&self) -> f32 {
        self.las.scan_angle.to_radians()
    }

    /// Calculates the misalignment between the backconverted point and the las point.
    pub fn misalignment(&self) -> Vector {
        self.backconverted_point() - self.las_point()
    }

    fn platform_to_global_rotation(&self) -> Matrix3<f64> {
        Matrix3::new(0., 1., 0., 1., 0., 0., 0., 0., -1.) * Matrix3::from(self.imu)
    }

    /// Returns the partial derivative for this measurement.
    pub fn partial<P: Into<Partial>>(&self, partial: P) -> f64 {
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
        let ca = self.scan_angle.cos();
        let sa = self.scan_angle.sin();
        let d = self.range;
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
            (Dimension::X, Variable::Distance) => {
                cp * sr * (-ca * (-cbr * cby * sbp + sbr * sby) - cbp * cbr * sa)
                    + (ca * (cbr * sby + cby * sbp * sbr) - cbp * sa * sbr)
                        * (cr * cy - sp * sr * sy)
                    + (cr * sy + cy * sp * sr) * (ca * cbp * cby + sa * sbp)
            }
            (Dimension::Y, Variable::Distance) => {
                cp * cy * (ca * cbp * cby + sa * sbp)
                    + cp * sy * (-ca * (cbr * sby + cby * sbp * sbr) + cbp * sa * sbr)
                    + sp * (ca * (-cbr * cby * sbp + sbr * sby) + cbp * cbr * sa)
            }
            (Dimension::Z, Variable::Distance) => {
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

    pub fn partial_check(&self, partial: Partial, delta: f64) -> PartialCheck {
        let expected = self.value(partial.0) + delta;
        let partial_value = self.partial(partial);
        let adjustment = delta / partial_value;
        let mut new_measurement = self.clone();
        new_measurement.adjust(partial.1, adjustment);
        let actual = new_measurement.value(partial.0);
        let error = actual - expected;
        PartialCheck {
            expected,
            partial: partial_value,
            adjustment,
            actual,
            error,
        }
    }

    fn adjust(&mut self, variable: Variable, adjustment: f64) {
        let reference = match variable {
            Variable::GnssX => &mut self.gnss.x,
            Variable::GnssY => &mut self.gnss.y,
            Variable::GnssZ => &mut self.gnss.z,
            Variable::ImuRoll => &mut self.imu.roll,
            Variable::ImuPitch => &mut self.imu.pitch,
            Variable::ImuYaw => &mut self.imu.yaw,
            Variable::BoresightRoll => &mut self.boresight.roll,
            Variable::BoresightPitch => &mut self.boresight.pitch,
            Variable::BoresightYaw => &mut self.boresight.yaw,
            Variable::Distance => &mut self.range,
            Variable::ScanAngle => &mut self.scan_angle,
            Variable::LeverArmX => &mut self.lever_arm.x,
            Variable::LeverArmY => &mut self.lever_arm.y,
            Variable::LeverArmZ => &mut self.lever_arm.z,
        };
        *reference += adjustment;
    }

    fn value(&self, dimension: Dimension) -> f64 {
        let point = self.backconverted_point();
        match dimension {
            Dimension::X => point.x,
            Dimension::Y => point.y,
            Dimension::Z => point.z,
        }
    }

    pub fn tpu(&self) -> Matrix3<f64> {
        let mut a = MatrixMN::<f64, U3, U14>::zeros();
        let mut error_covariance = MatrixMN::<f64, U14, U14>::zeros();
        for (col, variable) in Variable::iter().enumerate() {
            for (row, dimension) in Dimension::iter().enumerate() {
                a[(row, col)] = self.partial((dimension, variable));
            }
            error_covariance[(col, col)] = self.error(variable).powi(2);
        }
        a * error_covariance * a.transpose()
    }

    pub fn error(&self, variable: Variable) -> f64 {
        match variable {
            Variable::GnssX => 0.02,
            Variable::GnssY => 0.02,
            Variable::GnssZ => 0.04,
            Variable::ImuRoll => 0.0025_f64.to_radians(),
            Variable::ImuPitch => 0.0025_f64.to_radians(),
            Variable::ImuYaw => 0.005_f64.to_radians(),
            Variable::BoresightRoll => 0.001_f64.to_radians(),
            Variable::BoresightPitch => 0.001_f64.to_radians(),
            Variable::BoresightYaw => 0.004_f64.to_radians(),
            Variable::Distance => 0.02,
            Variable::ScanAngle => 0.001_f64.to_radians(),
            Variable::LeverArmX => 0.02,
            Variable::LeverArmY => 0.02,
            Variable::LeverArmZ => 0.02,
        }
    }
}
