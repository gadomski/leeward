use crate::{Config, Measurement};
use anyhow::Error;
use sbet::Point;
use std::collections::HashMap;
use std::path::Path;

#[derive(Debug)]
pub struct Trajectory(Vec<Point>);

#[derive(Debug)]
pub struct QuantizedTrajectory {
    points: HashMap<i64, Point>,
    level: u32,
}

impl Trajectory {
    /// Reads a trajectory from an sbet file at the provided path.
    ///
    /// # Examples
    ///
    /// ```
    /// use leeward::Trajectory;
    /// let trajectory = Trajectory::from_path("examples/sbet.out").unwrap();
    /// ```
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Trajectory, Error> {
        let reader = sbet::Reader::from_path(path.as_ref())?;
        let vec = reader.into_iter().collect::<Result<Vec<Point>, _>>()?;
        Ok(Trajectory(vec))
    }

    /// Converts this trajectory into an interator of its points.
    ///
    /// # Examples
    ///
    /// ```
    /// use leeward::Trajectory;
    /// let trajectory = Trajectory::from_path("examples/sbet.out").unwrap();
    /// let points: Vec<sbet::Point> = trajectory.into_iter().collect();
    /// ```
    pub fn into_iter(self) -> std::vec::IntoIter<sbet::Point> {
        self.0.into_iter()
    }

    /// Returns the length of this trajectory in number of points.
    /// Converts this trajectory into an interator of its points.
    ///
    /// # Examples
    ///
    /// ```
    /// use leeward::Trajectory;
    /// let trajectory = Trajectory::from_path("examples/sbet.out").unwrap();
    /// assert_eq!(trajectory.len(), 2);
    /// ```
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Quantizes this trajectory to the provided level.
    ///
    /// E.g. a level of 100 means that the trajectory will be quantized to every 10 millisecond.
    ///
    /// ```
    /// use leeward::Trajectory;
    /// let trajectory = Trajectory::from_path("examples/sbet.out").unwrap();
    /// let quantized_trajectory = trajectory.quantize(100);
    /// ```
    pub fn quantize(self, level: u32) -> QuantizedTrajectory {
        let mut points = HashMap::new();
        for point in self.into_iter() {
            points.insert(quantize(point.time, level), point);
        }
        QuantizedTrajectory {
            points: points,
            level: level,
        }
    }
}

impl From<Vec<Point>> for Trajectory {
    fn from(vec: Vec<Point>) -> Trajectory {
        Trajectory(vec)
    }
}

impl QuantizedTrajectory {
    pub fn measurement(&self, las: las::Point, config: Config) -> Result<Measurement, Error> {
        unimplemented!()
    }
}

fn quantize(time: f64, level: u32) -> i64 {
    (time * f64::from(level)).round() as i64
}
