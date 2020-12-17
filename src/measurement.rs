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
        assert_eq!(400825.80649573973, measurement.time());
        assert_eq!(320000.34, measurement.x());
        assert_eq!(4181319.35, measurement.y());
        assert_eq!(2687.59, measurement.z());
    }
}
