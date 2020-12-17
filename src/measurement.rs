use crate::{Config, Point, Trajectory};
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
        unimplemented!()
    }
}

#[cfg(test)]
mod tests {
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
}
