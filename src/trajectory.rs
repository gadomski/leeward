use crate::{Config, Measurement};
use anyhow::Error;
use sbet::Point;
use std::path::Path;

/// Smoothed Best Estimate of a Trajectory (sbet)
///
/// This structure holds a vector of the sbet points as well as an index for
/// quick time-based access to points.
#[derive(Debug)]
pub struct Trajectory {
    points: Vec<Point>,
}

impl Trajectory {
    /// Creates a trajectory from an sbet path.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Trajectory;
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// ```
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Trajectory, Error> {
        use sbet::Reader;
        let reader = Reader::from_path(path)?;
        let points = reader.into_iter().collect::<Result<Vec<Point>, Error>>()?;
        Ok(Trajectory { points })
    }

    /// Returns the number of points in this trajectory.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Trajectory;
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// let number_of_points = trajectory.len();
    /// ```
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Reads a las file and returns measurements.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Config, Trajectory};
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// let measurements = trajectory.read_las("data/points.las", &config).unwrap();
    /// ```
    pub fn read_las<P: AsRef<Path>>(
        &self,
        path: P,
        config: &Config,
    ) -> Result<Vec<Measurement>, Error> {
        use las::{Read, Reader};
        let mut reader = Reader::from_path(path)?;
        reader
            .points()
            .map(|result| {
                result
                    .map_err(Error::from)
                    .and_then(|point| self.measurement(&point, config))
            })
            .collect()
    }

    /// Creates a measurement from a point and a configuration.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Config, Trajectory};
    /// let points = leeward::read_las("data/points.las").unwrap();
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// let measurement = trajectory.measurement(&points[0], &config);
    /// ```
    pub fn measurement(&self, _point: &las::Point, _config: &Config) -> Result<Measurement, Error> {
        Ok(Measurement {})
    }
}

#[cfg(test)]
mod tests {
    use super::Trajectory;

    #[test]
    fn from_path() {
        let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
        assert_eq!(200, trajectory.len());
    }
}
