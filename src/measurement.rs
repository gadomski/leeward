use crate::{convert, Config, Matrix, Point, Trajectory};
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
) -> Result<Vec<Measurement>, Error> {
    use las::Read;
    let trajectory = Trajectory::from_path(sbet)?;
    let config = Config::from_path(config)?;
    let mut measurements = Vec::new();
    for result in las::Reader::from_path(las)?.points() {
        let point = result?;
        measurements.push(Measurement::new(&trajectory, point, config)?);
    }
    Ok(measurements)
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

    /// Returns the x coordinate of this measurement, from the las point.
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
        self.las.x
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
        self.las.y
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
        self.las.z
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
        self.las.gps_time.unwrap()
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
        let projected = Point::new(self.las.x, self.las.y, self.las.z);
        let plane = Point::new(self.sbet.longitude, self.sbet.latitude, self.sbet.altitude);
        convert::projected_to_body(
            projected,
            plane,
            self.sbet.roll,
            self.sbet.pitch,
            self.sbet.yaw,
            self.config.utm_zone,
        )
    }

    /// Calculates body frame coordinates using the lidar equation and this measurement's configuration.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let body_frame = measurements[0].body_frame_from_config();
    /// ```
    pub fn body_frame_from_config(&self) -> Point {
        self.boresight() * self.scanner() - self.lever_arm()
    }

    /// Returns this measurement's point in the scanner reference frame.
    ///
    /// This is calculated from the las point's scan angle and the computed range from the scanner origin to the target point.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let scanner = measurements[0].scanner();
    /// ```
    pub fn scanner(&self) -> Point {
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
    /// This value is taken from the las point.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let scan_angle = measurements[0].scan_angle();
    /// ```
    pub fn scan_angle(&self) -> f64 {
        f64::from(self.las.scan_angle).to_radians()
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
        convert::rotation_matrix(
            self.config.boresight.roll,
            self.config.boresight.pitch,
            self.config.boresight.yaw,
        )
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
