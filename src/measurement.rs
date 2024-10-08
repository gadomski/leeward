use crate::{convert, Config, Dimension, Matrix3, Point, RollPitchYaw, Trajectory, Variable};
use anyhow::{anyhow, Error};
use nalgebra::SMatrix;
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
#[derive(Debug, Clone)]
pub struct Measurement<L: Lasish> {
    las: L,
    sbet: sbet::Point,
    config: Config,
    use_las_scan_angle: bool,
}

/// The total propagated uncertainty for a measurement.
#[derive(Debug)]
pub struct Tpu {
    pub x: f64,
    pub y: f64,
    pub horizontal: f64,
    pub vertical: f64,
    pub total: f64,
    pub incidence_angle: f64,
}

/// A trait implemented by 3D points with ancillary lidar information, e.g. `las::Point`.
pub trait Lasish: Clone {
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

impl<L: Lasish> Measurement<L> {
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
    pub fn new(
        trajectory: &Trajectory,
        lasish: L,
        config: Config,
    ) -> Result<Measurement<L>, Error> {
        let time = lasish.time().ok_or(anyhow!("missing time on point"))?;
        let sbet = trajectory
            .get(time)
            .ok_or(anyhow!("could not find sbet point for time: {}", time))?;
        Ok(Measurement {
            las: lasish,
            sbet: *sbet,
            config,
            use_las_scan_angle: false,
        })
    }

    /// Sets whether this measurement uses the scan angle from the las point, or calculates it itself.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let mut measurement = measurements[0].clone();
    /// measurement.use_las_scan_angle(true);
    /// assert_eq!(measurement.scan_angle().to_degrees(), measurement.scan_angle().to_degrees().round());
    /// ```
    pub fn use_las_scan_angle(&mut self, use_las_scan_angle: bool) {
        self.use_las_scan_angle = use_las_scan_angle;
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
        self.las.x()
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
        self.las.y()
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
        self.las.z()
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
        self.las
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
        let projected = self.las.point();
        convert::projected_to_body(projected, self.platform(), self.rpy(), self.utm_zone())
    }

    fn platform(&self) -> Point {
        Point::new(self.sbet.longitude, self.sbet.latitude, self.sbet.altitude)
    }

    fn rpy(&self) -> RollPitchYaw {
        RollPitchYaw::new(self.sbet.roll, self.sbet.pitch, self.sbet.yaw)
    }

    fn utm_zone(&self) -> u8 {
        self.config.utm_zone
    }

    /// Calculates body frame coordinates using the lidar equation and this measurement's configuration.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let body_frame = measurements[0].modeled_body_frame();
    /// ```
    pub fn modeled_body_frame(&self) -> Point {
        self.boresight() * self.modeled_scan_frame() - self.lever_arm()
    }

    /// Returns this measurement's point in the scanner reference frame.
    ///
    /// This is calculated from the las point's scan angle and the computed range from the scanner origin to the target point.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let scanner = measurements[0].modeled_scan_frame();
    /// ```
    pub fn modeled_scan_frame(&self) -> Point {
        let range = self.range();
        let scan_angle = self.scan_angle();
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
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let scan_angle = measurements[0].scan_angle();
    /// ```
    pub fn scan_angle(&self) -> f64 {
        if self.use_las_scan_angle {
            self.las.scan_angle().to_radians()
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
    pub fn boresight(&self) -> Matrix3 {
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
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Dimension, Variable};
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let partial_derivative = measurements[0].partial_derivative_in_body_frame(Dimension::X, Variable::BoresightRoll);
    /// ```
    pub fn partial_derivative_in_body_frame(
        &self,
        dimension: Dimension,
        variable: Variable,
    ) -> f64 {
        let range = self.range();
        let scan_angle = self.scan_angle();
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
            las: self.las.clone(),
            sbet: self.sbet,
            config,
            use_las_scan_angle: self.use_las_scan_angle,
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
    /// let residuals = measurements[0].residuals();
    /// ```
    pub fn residuals(&self) -> Point {
        self.modeled_body_frame() - self.body_frame()
    }

    /// Returns this measurement's total propagated uncertainty.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Point;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let uncertainty = measurements[0].tpu(Point::new(0., 0., 1.)).unwrap();
    /// ```
    pub fn tpu(&self, normal: Point) -> Result<Tpu, Error> {
        let jacobian = self.jacobian();
        let incidence_angle = self.incidence_angle(normal);
        let covariance =
            jacobian.transpose() * self.uncertainty_covariance(incidence_angle) * jacobian;
        let x = covariance[(0, 0)].sqrt();
        let y = covariance[(1, 1)].sqrt();
        let z = covariance[(2, 2)].sqrt();
        Ok(Tpu {
            x,
            y,
            horizontal: (x.powi(2) + y.powi(2)).sqrt(),
            vertical: z,
            total: (x.powi(2) + y.powi(2) + z.powi(2)).sqrt(),
            incidence_angle,
        })
    }

    fn jacobian(&self) -> SMatrix<f64, 14, 3> {
        let mut jacobian = SMatrix::zeros();
        for (row, variable) in Variable::iter().enumerate() {
            for (col, dimension) in Dimension::iter().enumerate() {
                jacobian[(row, col)] = self.partial_derivative(variable, dimension);
            }
        }
        jacobian
    }

    fn partial_derivative(&self, variable: Variable, dimension: Dimension) -> f64 {
        let cr = self.roll().cos();
        let sr = self.roll().sin();
        let cp = self.pitch().cos();
        let sp = self.pitch().sin();
        let cy = self.yaw().cos();
        let sy = self.yaw().sin();
        let cbr = self.boresight_roll().cos();
        let sbr = self.boresight_roll().sin();
        let cbp = self.boresight_pitch().cos();
        let sbp = self.boresight_pitch().sin();
        let cby = self.boresight_yaw().cos();
        let sby = self.boresight_yaw().sin();
        let lx = self.lever_arm_x();
        let ly = self.lever_arm_y();
        let lz = self.lever_arm_z();
        let ca = self.scan_angle().cos();
        let sa = self.scan_angle().sin();
        let r = self.range();
        match variable {
            Variable::GnssX => match dimension {
                Dimension::X => 1.,
                Dimension::Y => 0.,
                Dimension::Z => 0.,
            },
            Variable::GnssY => match dimension {
                Dimension::X => 0.,
                Dimension::Y => 1.,
                Dimension::Z => 0.,
            },
            Variable::GnssZ => match dimension {
                Dimension::X => 0.,
                Dimension::Y => 0.,
                Dimension::Z => 1.,
            },
            Variable::Roll => match dimension {
                Dimension::X => {
                    (cr * sy - cy * sp * sr) * (-ca * r * sbp + cbp * cbr * r * sa - lz)
                        + (cr * cy * sp + sr * sy)
                            * (ca * cbp * r * sby - ly + r * sa * (cbr * sbp * sby - cby * sbr))
                }
                Dimension::Y => {
                    (-cr * cy - sp * sr * sy) * (-ca * r * sbp + cbp * cbr * r * sa - lz)
                        + (cr * sp * sy - cy * sr)
                            * (ca * cbp * r * sby - ly + r * sa * (cbr * sbp * sby - cby * sbr))
                }
                Dimension::Z => {
                    cp * cr * (ca * cbp * r * sby - ly + r * sa * (cbr * sbp * sby - cby * sbr))
                        - cp * sr * (-ca * r * sbp + cbp * cbr * r * sa - lz)
                }
            },
            Variable::Pitch => match dimension {
                Dimension::X => {
                    cp * cr * cy * (-ca * r * sbp + cbp * cbr * r * sa - lz)
                        + cp * cy
                            * sr
                            * (ca * cbp * r * sby - ly + r * sa * (cbr * sbp * sby - cby * sbr))
                        - cy * sp
                            * (ca * cbp * cby * r - lx + r * sa * (cbr * cby * sbp + sbr * sby))
                }
                Dimension::Y => {
                    cp * cr * sy * (-ca * r * sbp + cbp * cbr * r * sa - lz)
                        + cp * sr
                            * sy
                            * (ca * cbp * r * sby - ly + r * sa * (cbr * sbp * sby - cby * sbr))
                        - sp * sy
                            * (ca * cbp * cby * r - lx + r * sa * (cbr * cby * sbp + sbr * sby))
                }
                Dimension::Z => {
                    -cp * (ca * cbp * cby * r - lx + r * sa * (cbr * cby * sbp + sbr * sby))
                        - cr * sp * (-ca * r * sbp + cbp * cbr * r * sa - lz)
                        - sp * sr
                            * (ca * cbp * r * sby - ly + r * sa * (cbr * sbp * sby - cby * sbr))
                }
            },
            Variable::Yaw => match dimension {
                Dimension::X => {
                    -cp * sy * (ca * cbp * cby * r - lx + r * sa * (cbr * cby * sbp + sbr * sby))
                        + (-cr * cy - sp * sr * sy)
                            * (ca * cbp * r * sby - ly + r * sa * (cbr * sbp * sby - cby * sbr))
                        + (-cr * sp * sy + cy * sr) * (-ca * r * sbp + cbp * cbr * r * sa - lz)
                }
                Dimension::Y => {
                    cp * cy * (ca * cbp * cby * r - lx + r * sa * (cbr * cby * sbp + sbr * sby))
                        + (-cr * sy + cy * sp * sr)
                            * (ca * cbp * r * sby - ly + r * sa * (cbr * sbp * sby - cby * sbr))
                        + (cr * cy * sp + sr * sy) * (-ca * r * sbp + cbp * cbr * r * sa - lz)
                }
                Dimension::Z => 0.,
            },
            Variable::BoresightRoll => match dimension {
                Dimension::X => {
                    -cbp * r * sa * sbr * (cr * cy * sp + sr * sy)
                        + cp * cy * r * sa * (cbr * sby - cby * sbp * sbr)
                        + r * sa * (-cbr * cby - sbp * sbr * sby) * (-cr * sy + cy * sp * sr)
                }
                Dimension::Y => {
                    -cbp * r * sa * sbr * (cr * sp * sy - cy * sr)
                        + cp * r * sa * sy * (cbr * sby - cby * sbp * sbr)
                        + r * sa * (-cbr * cby - sbp * sbr * sby) * (cr * cy + sp * sr * sy)
                }
                Dimension::Z => {
                    -cbp * cp * cr * r * sa * sbr
                        + cp * r * sa * sr * (-cbr * cby - sbp * sbr * sby)
                        - r * sa * sp * (cbr * sby - cby * sbp * sbr)
                }
            },
            Variable::BoresightPitch => match dimension {
                Dimension::X => {
                    cp * cy * (-ca * cby * r * sbp + cbp * cbr * cby * r * sa)
                        + (-cr * sy + cy * sp * sr)
                            * (-ca * r * sbp * sby + cbp * cbr * r * sa * sby)
                        + (-ca * cbp * r - cbr * r * sa * sbp) * (cr * cy * sp + sr * sy)
                }
                Dimension::Y => {
                    cp * sy * (-ca * cby * r * sbp + cbp * cbr * cby * r * sa)
                        + (cr * cy + sp * sr * sy)
                            * (-ca * r * sbp * sby + cbp * cbr * r * sa * sby)
                        + (-ca * cbp * r - cbr * r * sa * sbp) * (cr * sp * sy - cy * sr)
                }
                Dimension::Z => {
                    cp * cr * (-ca * cbp * r - cbr * r * sa * sbp)
                        + cp * sr * (-ca * r * sbp * sby + cbp * cbr * r * sa * sby)
                        - sp * (-ca * cby * r * sbp + cbp * cbr * cby * r * sa)
                }
            },
            Variable::BoresightYaw => match dimension {
                Dimension::X => {
                    cp * cy * (-ca * cbp * r * sby + r * sa * (-cbr * sbp * sby + cby * sbr))
                        + (-cr * sy + cy * sp * sr)
                            * (ca * cbp * cby * r + r * sa * (cbr * cby * sbp + sbr * sby))
                }
                Dimension::Y => {
                    cp * sy * (-ca * cbp * r * sby + r * sa * (-cbr * sbp * sby + cby * sbr))
                        + (cr * cy + sp * sr * sy)
                            * (ca * cbp * cby * r + r * sa * (cbr * cby * sbp + sbr * sby))
                }
                Dimension::Z => {
                    cp * sr * (ca * cbp * cby * r + r * sa * (cbr * cby * sbp + sbr * sby))
                        - sp * (-ca * cbp * r * sby + r * sa * (-cbr * sbp * sby + cby * sbr))
                }
            },
            Variable::Range => match dimension {
                Dimension::X => {
                    cp * cy * (ca * cbp * cby + sa * (cbr * cby * sbp + sbr * sby))
                        + (-ca * sbp + cbp * cbr * sa) * (cr * cy * sp + sr * sy)
                        + (-cr * sy + cy * sp * sr)
                            * (ca * cbp * sby + sa * (cbr * sbp * sby - cby * sbr))
                }
                Dimension::Y => {
                    cp * sy * (ca * cbp * cby + sa * (cbr * cby * sbp + sbr * sby))
                        + (-ca * sbp + cbp * cbr * sa) * (cr * sp * sy - cy * sr)
                        + (cr * cy + sp * sr * sy)
                            * (ca * cbp * sby + sa * (cbr * sbp * sby - cby * sbr))
                }
                Dimension::Z => {
                    cp * cr * (-ca * sbp + cbp * cbr * sa)
                        + cp * sr * (ca * cbp * sby + sa * (cbr * sbp * sby - cby * sbr))
                        - sp * (ca * cbp * cby + sa * (cbr * cby * sbp + sbr * sby))
                }
            },
            Variable::ScanAngle => match dimension {
                Dimension::X => {
                    cp * cy * (ca * r * (cbr * cby * sbp + sbr * sby) - cbp * cby * r * sa)
                        + (-cr * sy + cy * sp * sr)
                            * (ca * r * (cbr * sbp * sby - cby * sbr) - cbp * r * sa * sby)
                        + (cr * cy * sp + sr * sy) * (ca * cbp * cbr * r + r * sa * sbp)
                }
                Dimension::Y => {
                    cp * sy * (ca * r * (cbr * cby * sbp + sbr * sby) - cbp * cby * r * sa)
                        + (cr * cy + sp * sr * sy)
                            * (ca * r * (cbr * sbp * sby - cby * sbr) - cbp * r * sa * sby)
                        + (cr * sp * sy - cy * sr) * (ca * cbp * cbr * r + r * sa * sbp)
                }
                Dimension::Z => {
                    cp * cr * (ca * cbp * cbr * r + r * sa * sbp)
                        + cp * sr * (ca * r * (cbr * sbp * sby - cby * sbr) - cbp * r * sa * sby)
                        - sp * (ca * r * (cbr * cby * sbp + sbr * sby) - cbp * cby * r * sa)
                }
            },
            Variable::LeverArmX => match dimension {
                Dimension::X => -cp * cy,
                Dimension::Y => -cp * sy,
                Dimension::Z => sp,
            },
            Variable::LeverArmY => match dimension {
                Dimension::X => cr * sy - cy * sp * sr,
                Dimension::Y => -cr * cy - sp * sr * sy,
                Dimension::Z => -cp * sr,
            },
            Variable::LeverArmZ => match dimension {
                Dimension::X => -cr * cy * sp - sr * sy,
                Dimension::Y => -cr * sp * sy + cy * sr,
                Dimension::Z => -cp * cr,
            },
        }
    }

    fn incidence_angle(&self, normal: Point) -> f64 {
        let projected_normal_endpoint = self.las.point() + normal;
        let body_normal_endpoint = convert::projected_to_body(
            projected_normal_endpoint,
            self.platform(),
            self.rpy(),
            self.utm_zone(),
        );
        let body_frame = self.body_frame();
        let normal = body_frame - body_normal_endpoint;
        (normal.dot(&body_frame) / (normal.norm() * body_frame.norm())).acos()
    }

    fn uncertainty_covariance(&self, incidence_angle: f64) -> SMatrix<f64, 14, 14> {
        let mut matrix = SMatrix::<f64, 14, 14>::zeros();
        for (i, variable) in Variable::iter().enumerate() {
            matrix[(i, i)] = self.uncertainty(variable, incidence_angle).powi(2);
        }
        matrix
    }

    fn uncertainty(&self, variable: Variable, incidence_angle: f64) -> f64 {
        use Variable::*;
        match variable {
            GnssX => self.config.uncertainty.gnss_x,
            GnssY => self.config.uncertainty.gnss_y,
            GnssZ => self.config.uncertainty.gnss_z,
            Roll => self.config.uncertainty.roll,
            Pitch => self.config.uncertainty.pitch,
            Yaw => self.config.uncertainty.yaw,
            BoresightRoll => self.config.uncertainty.boresight_roll,
            BoresightPitch => self.config.uncertainty.boresight_pitch,
            BoresightYaw => self.config.uncertainty.boresight_yaw,
            LeverArmX => self.config.uncertainty.lever_arm_x,
            LeverArmY => self.config.uncertainty.lever_arm_y,
            LeverArmZ => self.config.uncertainty.lever_arm_z,
            Range => (self.config.uncertainty.range.powi(2)
                + (self.range() * self.config.beam_divergence / 4.0 * incidence_angle.tan()))
            .sqrt(),
            ScanAngle => {
                self.config.uncertainty.scan_angle.powi(2)
                    + (self.config.beam_divergence / 4.0).powi(2)
            }
        }
    }
}

impl Lasish for las::Point {
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
    use crate::Point;
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

    #[test]
    fn scan_angle() {
        let measurements =
            super::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let mut measurement = measurements[0].clone();
        assert_relative_eq!(
            0.402565395893292,
            measurement.scan_angle(),
            max_relative = 1e-6
        );
        measurement.use_las_scan_angle(true);
        assert_eq!(22f64.to_radians(), measurement.scan_angle());
    }

    #[test]
    fn uncertainty() {
        let measurements =
            super::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let _uncertainty = measurements[0].tpu(Point::new(0., 0., 1.)).unwrap();
    }

    #[test]
    fn incidence_angle() {
        let measurements =
            super::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let incidence_angle = measurements[0].incidence_angle(Point::new(0., 0., 1.));
        assert!(incidence_angle < 90f64.to_radians());
        assert!(incidence_angle > 0f64.to_radians());
    }
}
