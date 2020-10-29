use crate::{Config, Measurement};
use anyhow::{anyhow, Error};
use std::{collections::HashMap, path::Path};

/// Smoothed Best Estimate of a Trajectory (sbet)
///
/// This structure holds a vector of the sbet points as well as an index for
/// quick time-based access to points.
#[derive(Debug)]
pub struct Trajectory {
    points: Vec<sbet::Point>,
    scale: f64,
    index: HashMap<i64, usize>,
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
        let points = reader
            .into_iter()
            .collect::<Result<Vec<sbet::Point>, Error>>()?;
        Trajectory::new(points)
    }

    /// Creates a new trajectory.
    ///
    /// This method chooses the "best" index scale based upon the spacing of the sbet points.
    /// To specify your own scale, use `new_with_scale`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Trajectory;
    /// let points = vec![sbet::Point::default()];
    /// let trajectory = Trajectory::new(points);
    /// ```
    pub fn new(points: Vec<sbet::Point>) -> Result<Trajectory, Error> {
        guess_scale(&points).map(|s| Trajectory::new_with_scale(points, s))
    }

    /// Creates a new trajectory with the provided points and index scale.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Trajectory;
    /// let points = vec![sbet::Point::default()];
    /// let trajectory = Trajectory::new_with_scale(points, 1.0);
    /// ```
    pub fn new_with_scale(points: Vec<sbet::Point>, scale: f64) -> Trajectory {
        Trajectory {
            index: points
                .iter()
                .enumerate()
                .map(|(i, p)| (scaled_time(p.time, scale), i))
                .collect(),
            scale: scale,
            points: points,
        }
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

    /// Returns this trajectory's scale factor.
    ///
    /// The scale factor is used by the internal trajectory index.
    /// A good scale that allows access to all points via the index (provided the points are evenly spaced) is the time spacing of the points, e.g. if each point is 10 milliseconds apart, the scale should be 0.01.
    pub fn scale(&self) -> f64 {
        self.scale
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
    /// let measurement = trajectory.measurement(&points[0], &config).unwrap();
    /// ```
    pub fn measurement(&self, las: &las::Point, config: &Config) -> Result<Measurement, Error> {
        let time = las
            .gps_time
            .ok_or_else(|| anyhow!("missing gps_time on las point"))?;
        let sbet = self
            .point(time)
            .ok_or_else(|| anyhow!("could not find sbet point for las gps_time: {}", time))?;
        Ok(Measurement::new(sbet, las, config))
    }

    /// Returns the sbet point for the provided timestamp.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Trajectory;
    /// let points = vec![sbet::Point { time: 1., ..Default::default() }];
    /// let trajectory = Trajectory::new_with_scale(points.clone(), 1.);
    /// assert_eq!(&points[0], trajectory.point(1.).unwrap());
    /// ```
    pub fn point(&self, time: f64) -> Option<&sbet::Point> {
        let time = self.scaled_time(time);
        self.index
            .get(&time)
            .and_then(|index| self.points.get(*index))
    }

    /// Returns a reference to a slice of all this trajectory's points.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Trajectory;
    /// let points = vec![sbet::Point { time: 1., ..Default::default() }];
    /// let trajectory = Trajectory::new_with_scale(points.clone(), 1.);
    /// let points = trajectory.points();
    /// ```
    pub fn points(&self) -> &[sbet::Point] {
        &self.points
    }

    /// Returns the provided time as a scaled integer, as deteremined by this trajectory's index scale.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Trajectory;
    /// let trajectory = Trajectory::new_with_scale(vec![], 0.1);
    /// assert_eq!(10, trajectory.scaled_time(1.0));
    /// ```
    pub fn scaled_time(&self, time: f64) -> i64 {
        scaled_time(time, self.scale)
    }
}

fn scaled_time(time: f64, scale: f64) -> i64 {
    (time / scale).round() as i64
}

fn guess_scale(points: &[sbet::Point]) -> Result<f64, Error> {
    if points.len() < 2 {
        Err(anyhow!(
            "cannot guess scale with {} sbet points",
            points.len()
        ))
    } else {
        let mut sum = 0.;
        for (a, b) in points.iter().zip(points.iter().skip(1)) {
            if b.time <= a.time {
                return Err(anyhow!(
                    "sbet times do not increase monatonically: first={}, second={}",
                    a.time,
                    b.time
                ));
            }
            sum += b.time - a.time;
        }
        Ok(sum / (points.len() - 1) as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::Trajectory;

    #[test]
    fn from_path() {
        let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
        assert_eq!(200, trajectory.len());
        assert!((0.005 - trajectory.scale()) < 0.0001);
    }

    #[test]
    fn time_within_bounds() {
        let points = [1f64, 2.]
            .iter()
            .map(|&t| sbet::Point {
                time: t,
                ..Default::default()
            })
            .collect::<Vec<_>>();
        let trajectory = Trajectory::new_with_scale(points.clone(), 1.0);
        assert!(trajectory.point(0.49).is_none());
        assert_eq!(&points[0], trajectory.point(0.51).unwrap());
        assert_eq!(&points[0], trajectory.point(1.).unwrap());
        assert_eq!(&points[0], trajectory.point(1.49).unwrap());
        assert_eq!(&points[1], trajectory.point(1.51).unwrap());
        assert_eq!(&points[1], trajectory.point(2.).unwrap());
        assert_eq!(&points[1], trajectory.point(2.49).unwrap());
        assert!(trajectory.point(2.51).is_none());
    }
}
