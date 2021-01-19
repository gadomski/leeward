use crate::{convert, Config, Dimension, Matrix, Point, RollPitchYaw, Trajectory, Variable};
use anyhow::{anyhow, Error};
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
) -> Result<Vec<Measurement<las::Point>>, Error> {
    decimated_measurements(sbet, las, config, 1)
}

/// Reads in a vector of measurements from files with the provided decimation.
///
/// # Examples
///
/// ```
/// let measurements = leeward::decimated_measurements(
///     "data/sbet.out",
///     "data/points.las",
///     "data/config.toml",
///     100
/// ).unwrap();
/// ```
pub fn decimated_measurements<P0: AsRef<Path>, P1: AsRef<Path>, P2: AsRef<Path>>(
    sbet: P0,
    las: P1,
    config: P2,
    decimation: usize,
) -> Result<Vec<Measurement<las::Point>>, Error> {
    use las::Read;
    if decimation == 0 {
        return Err(anyhow!("cannot decimate by zero"));
    }
    let trajectory = Trajectory::from_path(sbet)?;
    let config = Config::from_path(config)?;
    las::Reader::from_path(las)?
        .points()
        .step_by(decimation)
        .map(|r| {
            r.map_err(Error::from)
                .and_then(|p| Measurement::new(&trajectory, p, config))
        })
        .collect()
}

/// A measurement combines trajectory information with the lidar point.
#[derive(Debug)]
pub struct Measurement<L: Lidar> {
    lidar: L,
    sbet: sbet::Point,
    config: Config,
}

/// A trait implemented by 3D points with ancillary lidar information, e.g. `las::Point`.
pub trait Lidar: Clone {
    /// Returns the gps time from this point, or `None` if it is not defined.
    ///
    /// We can't really do anything useful without time, but since las points can
    /// come in w/o time we have to handle that case.
    fn time(&self) -> Option<f64>;

    /// Returns the x coordinate of this point.
    fn x(&self) -> f64;

    /// Returns the y coordinate of this point.
    fn y(&self) -> f64;

    /// Returns the z coordinate of this point.
    fn z(&self) -> f64;

    /// Returns the xyz point.
    fn point(&self) -> Point {
        Point::new(self.x(), self.y(), self.z())
    }

    /// Returns the scan angle of this point.
    fn scan_angle(&self) -> f64;
}

impl<L: Lidar> Measurement<L> {
    /// Creates a new measurement.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Trajectory, Measurement, Config};
    /// use las::Read;
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let point = las::Reader::from_path("data/points.las")
    ///     .unwrap()
    ///     .points()
    ///     .next()
    ///     .unwrap()
    ///     .unwrap();
    /// let measurement = Measurement::new(&trajectory, point, config).unwrap();
    /// ```
    pub fn new(trajectory: &Trajectory, lidar: L, config: Config) -> Result<Measurement<L>, Error> {
        let time = lidar.time().ok_or(anyhow!("missing time on point"))?;
        let sbet = trajectory
            .get(time)
            .ok_or(anyhow!("could not find sbet point for time: {}", time))?;
        Ok(Measurement {
            lidar,
            sbet: *sbet,
            config,
        })
    }

    /// Returns the x coordinate of this measurement, from the lidar point.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Trajectory, Measurement, Config};
    /// use las::Read;
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let point = las::Reader::from_path("data/points.las")
    ///     .unwrap()
    ///     .points()
    ///     .next()
    ///     .unwrap()
    ///     .unwrap();
    /// let measurement = Measurement::new(&trajectory, point.clone(), config).unwrap();
    /// assert_eq!(point.x, measurement.x());
    /// ```
    pub fn x(&self) -> f64 {
        self.lidar.x()
    }

    /// Returns the y coordinate of this measurement, from the las point.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Trajectory, Measurement, Config};
    /// use las::Read;
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let point = las::Reader::from_path("data/points.las")
    ///     .unwrap()
    ///     .points()
    ///     .next()
    ///     .unwrap()
    ///     .unwrap();
    /// let measurement = Measurement::new(&trajectory, point.clone(), config).unwrap();
    /// assert_eq!(point.y, measurement.y());
    /// ```
    pub fn y(&self) -> f64 {
        self.lidar.y()
    }

    /// Returns the z coordinate of this measurement, from the las point.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Trajectory, Measurement, Config};
    /// use las::Read;
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let point = las::Reader::from_path("data/points.las")
    ///     .unwrap()
    ///     .points()
    ///     .next()
    ///     .unwrap()
    ///     .unwrap();
    /// let measurement = Measurement::new(&trajectory, point.clone(), config).unwrap();
    /// assert_eq!(point.z, measurement.z());
    /// ```
    pub fn z(&self) -> f64 {
        self.lidar.z()
    }

    /// Returns the roll of this measurement, from the sbet.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let roll = measurements[0].roll();
    /// ```
    pub fn roll(&self) -> f64 {
        self.sbet.roll
    }

    /// Returns the pitch of this measurement, from the sbet.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let pitch = measurements[0].pitch();
    /// ```
    pub fn pitch(&self) -> f64 {
        self.sbet.pitch
    }

    /// Returns the yaw of this measurement, from the sbet.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let yaw = measurements[0].yaw();
    /// ```
    pub fn yaw(&self) -> f64 {
        self.sbet.yaw
    }

    /// Returns the time of this measurement, from the las point.
    ///
    /// Although not all las points have gps time, we know ours does because we check during measurement creation.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Trajectory, Measurement, Config};
    /// use las::Read;
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let point = las::Reader::from_path("data/points.las")
    ///     .unwrap()
    ///     .points()
    ///     .next()
    ///     .unwrap()
    ///     .unwrap();
    /// let measurement = Measurement::new(&trajectory, point.clone(), config).unwrap();
    /// assert_eq!(point.gps_time.unwrap(), measurement.time());
    /// ```
    pub fn time(&self) -> f64 {
        self.lidar
            .time()
            .expect("time should be something because we check when creating the measurement")
    }

    /// Returns this measurement in the body frame of the aircraft.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Trajectory, Measurement, Config};
    /// use las::Read;
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let point = las::Reader::from_path("data/points.las")
    ///     .unwrap()
    ///     .points()
    ///     .next()
    ///     .unwrap()
    ///     .unwrap();
    /// let measurement = Measurement::new(&trajectory, point.clone(), config).unwrap();
    /// let body_frame = measurement.body_frame();
    /// ```
    pub fn body_frame(&self) -> Point {
        let projected = self.lidar.point();
        let platform = Point::new(self.sbet.longitude, self.sbet.latitude, self.sbet.altitude);
        convert::projected_to_body(
            projected,
            platform,
            RollPitchYaw::new(self.sbet.roll, self.sbet.pitch, self.sbet.yaw),
            self.config.utm_zone,
        )
    }

    /// Calculates body frame coordinates using the lidar equation and this measurement's configuration.
    ///
    /// The argument specifies whether to use the las scan angle (true) or the computed one (false).
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let body_frame = measurements[0].body_frame_from_config(false);
    /// ```
    pub fn body_frame_from_config(&self, lidar_scan_angle: bool) -> Point {
        self.boresight() * self.scanner(lidar_scan_angle) - self.lever_arm()
    }

    /// Returns this measurement's point in the scanner reference frame.
    ///
    /// This is calculated from the las point's scan angle and the computed range from the scanner origin to the target point.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let scanner = measurements[0].scanner(false);
    /// ```
    pub fn scanner(&self, lidar_scan_angle: bool) -> Point {
        let range = self.range();
        let scan_angle = self.scan_angle(lidar_scan_angle);
        Point::new(range * scan_angle.cos(), 0., range * scan_angle.sin())
    }

    /// Returns this measurement's scan range.
    ///
    /// This is the vector distance from the scanner origin to the measured point.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let range = measurements[0].range();
    /// ```
    pub fn range(&self) -> f64 {
        let body_frame = self.body_frame();
        (body_frame - (Point::new(0., 0., 0.) - self.lever_arm())).norm()
    }

    /// Returns this measurement's scan angle in radians.
    ///
    /// If the argument is false, this value is taken from the las point in the body frame of the platform.
    /// If true, the scan angle is from the las file (converted to radians).
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let scan_angle = measurements[0].scan_angle(false);
    /// ```
    pub fn scan_angle(&self, lidar: bool) -> f64 {
        if lidar {
            self.lidar.scan_angle().to_radians()
        } else {
            let body_frame = self.body_frame();
            body_frame.y.signum()
                * (body_frame.x.powi(2) + body_frame.y.powi(2))
                    .sqrt()
                    .atan2(body_frame.z)
        }
    }

    /// Returns this measurement's boresight angles as a rotation matrix.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let boresight = measurements[0].boresight();
    /// ```
    pub fn boresight(&self) -> Matrix {
        RollPitchYaw::new(
            self.config.boresight.roll,
            self.config.boresight.pitch,
            self.config.boresight.yaw,
        )
        .as_matrix()
    }

    /// Returns this measurement's lever arm.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let lever_arm = measurements[0].lever_arm();
    /// ```
    pub fn lever_arm(&self) -> Point {
        self.config.lever_arm
    }

    /// Returns the partial derivative in the body frame for the given dimension and variable.
    ///
    /// The last argument specifies whether to use the computed scan angle (false) or the las scan angle (true).
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Dimension, Variable};
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let partial_derivative = measurements[0].partial_derivative_in_body_frame(Dimension::X, Variable::BoresightRoll, false);
    /// ```
    pub fn partial_derivative_in_body_frame(
        &self,
        dimension: Dimension,
        variable: Variable,
        lidar_scan_angle: bool,
    ) -> f64 {
        let range = self.range();
        let scan_angle = self.scan_angle(lidar_scan_angle);
        let sa = scan_angle.sin();
        let ca = scan_angle.cos();
        let sr = self.config.boresight.roll.sin();
        let cr = self.config.boresight.roll.cos();
        let sp = self.config.boresight.pitch.sin();
        let cp = self.config.boresight.pitch.cos();
        let sy = self.config.boresight.yaw.sin();
        let cy = self.config.boresight.yaw.cos();
        match variable {
            Variable::BoresightRoll => match dimension {
                Dimension::X => range * sa * (cr * sy - cy * sp * sr),
                Dimension::Y => range * sa * (-cr * cy - sp * sr * sy),
                Dimension::Z => -cp * range * sa * sr,
            },
            Variable::BoresightPitch => match dimension {
                Dimension::X => -ca * cy * range * sp + cp * cr * cy * range * sa,
                Dimension::Y => -ca * range * sp * sy + cp * cr * range * sa * sy,
                Dimension::Z => -ca * cp * range - cr * range * sa * sp,
            },
            Variable::BoresightYaw => match dimension {
                Dimension::X => -ca * cp * range * sy + range * sa * (-cr * sp * sy + cy * sr),
                Dimension::Y => ca * cp * cy * range + range * sa * (cr * cy * sp + sr * sy),
                Dimension::Z => 0.,
            },
            Variable::LeverArmX => match dimension {
                Dimension::X => -1.,
                Dimension::Y => 0.,
                Dimension::Z => 0.,
            },
            Variable::LeverArmY => match dimension {
                Dimension::X => 0.,
                Dimension::Y => -1.,
                Dimension::Z => 0.,
            },
            Variable::LeverArmZ => match dimension {
                Dimension::X => 0.,
                Dimension::Y => 0.,
                Dimension::Z => -1.,
            },
            Variable::Range => unimplemented!(),
            Variable::ScanAngle => unimplemented!(),
            _ => 0.,
        }
    }

    /// Returns this measurement's boresight roll.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let boresight_roll = measurements[0].boresight_roll();
    /// ```
    pub fn boresight_roll(&self) -> f64 {
        self.config.boresight.roll
    }

    /// Returns this measurement's boresight pitch.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let boresight_pitch = measurements[0].boresight_pitch();
    /// ```
    pub fn boresight_pitch(&self) -> f64 {
        self.config.boresight.pitch
    }

    /// Returns this measurement's boresight yaw.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let boresight_yaw = measurements[0].boresight_yaw();
    /// ```
    pub fn boresight_yaw(&self) -> f64 {
        self.config.boresight.yaw
    }

    /// Returns this measurement's lever arm x.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let boresight_yaw = measurements[0].lever_arm_x();
    /// ```
    pub fn lever_arm_x(&self) -> f64 {
        self.config.lever_arm.x
    }

    /// Returns this measurement's lever arm y.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let boresight_yaw = measurements[0].lever_arm_y();
    /// ```
    pub fn lever_arm_y(&self) -> f64 {
        self.config.lever_arm.y
    }

    /// Returns this measurement's lever arm z.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let boresight_yaw = measurements[0].lever_arm_z();
    /// ```
    pub fn lever_arm_z(&self) -> f64 {
        self.config.lever_arm.z
    }

    /// Returns this measurement's config.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let config = measurements[0].config();
    /// ```
    pub fn config(&self) -> Config {
        self.config
    }

    /// Creates a new measurement with the provided config.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Config;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let mut config = Config::from_path("data/config.toml").unwrap();
    /// config.lever_arm.x = 1.0;
    /// let measurement = measurements[0].with_config(config);
    /// ```
    pub fn with_config(&self, config: Config) -> Measurement<L> {
        Measurement {
            lidar: self.lidar.clone(),
            sbet: self.sbet,
            config: config,
        }
    }

    /// Returns this measurement's residuals.
    ///
    /// Residuals are computed in the body frame of the aircraft, and are the
    /// difference between the computed body coordinates and the las body
    /// coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let residuals = measurements[0].residuals(false);
    /// ```
    pub fn residuals(&self, lidar_scan_angle: bool) -> Point {
        self.body_frame_from_config(lidar_scan_angle) - self.body_frame()
    }
}

impl Lidar for las::Point {
    fn time(&self) -> Option<f64> {
        self.gps_time
    }

    fn x(&self) -> f64 {
        self.x
    }

    fn y(&self) -> f64 {
        self.y
    }

    fn z(&self) -> f64 {
        self.z
    }

    fn scan_angle(&self) -> f64 {
        f64::from(self.scan_angle)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    #[test]
    fn measurements() {
        let measurements =
            super::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let measurement = &measurements[0];
        assert_eq!(400825.80571932, measurement.time());
        assert_eq!(320000.34, measurement.x());
        assert_eq!(4181319.35, measurement.y());
        assert_eq!(2687.59, measurement.z());
    }

    #[test]
    fn body_frame() {
        let measurements =
            super::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let measurement = &measurements[0];
        let body_frame = measurement.body_frame();
        assert_relative_eq!(-405.710, body_frame.x, max_relative = 1e-3);
        assert_relative_eq!(1780.085, body_frame.y, max_relative = 1e-3);
        assert_relative_eq!(4287.566, body_frame.z, max_relative = 1e-3);
    }

    #[test]
    fn range() {
        let measurements =
            super::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let range = measurements[0].range();
        assert_relative_eq!(4660.10, range, max_relative = 1e-2);
    }
}
