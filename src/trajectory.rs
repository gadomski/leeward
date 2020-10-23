use crate::{Config, Measurement};
use anyhow::{anyhow, Error};
use sbet::Point;
use std::collections::HashMap;
use std::path::Path;

/// An indexed SBET trajectory.
#[derive(Debug)]
pub struct Trajectory {
    points: Vec<Point>,
    index: HashMap<i64, usize>,
    level: f64,
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
        Ok(Trajectory::new(vec, None))
    }

    /// Creates a new trajectory from a vector of points.
    ///
    /// Optionally, provide a quanization level for the trajectory index. The quantization level is used like this: `quantized_time = (time * level).round() as i64;`
    ///
    /// Pass `None` to intuit the quantization level from the average spacing of the time between points.
    ///
    /// # Examples
    ///
    /// ```
    /// use leeward::Trajectory;
    /// let points = vec![sbet::Point { time: 42.0, ..Default::default() }];
    /// let trajectory = Trajectory::new(points, None);
    /// ```
    pub fn new(points: Vec<Point>, level: Option<f64>) -> Trajectory {
        let level = level.unwrap_or_else(|| calculate_level(&points));
        let mut index = HashMap::new();
        for (i, point) in points.iter().enumerate() {
            index.insert(quantize(point.time, level), i);
        }
        Trajectory {
            index: index,
            level: level,
            points: points,
        }
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
        self.points.into_iter()
    }

    /// Returns the length of this trajectory in number of points.
    /// Converts this trajectory into an interator of its points.
    ///
    /// # Examples
    ///
    /// ```
    /// use leeward::Trajectory;
    /// let trajectory = Trajectory::from_path("examples/sbet.out").unwrap();
    /// assert_eq!(trajectory.len(), 200);
    /// ```
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Create a measurement that can be used to calculate TPU.
    ///
    /// # Examples
    ///
    /// ```
    /// use leeward::{Trajectory, Config};
    /// let trajectory: Trajectory = vec![sbet::Point { time: 42.0, ..Default::default() }].into();
    /// let measurement = trajectory.measurement(las::Point { gps_time: Some(42.0), ..Default::default() }, Config::default()).unwrap();
    /// ```
    pub fn measurement(&self, las: las::Point, config: Config) -> Result<Measurement, Error> {
        let time = las
            .gps_time
            .ok_or_else(|| anyhow!("Missing GPSTime on las point"))?;
        let sbet = self
            .point(time)
            .ok_or_else(|| anyhow!("Could not find sbet point for time: {}", time))?;
        Ok(Measurement::new(las, *sbet, config))
    }

    /// Returns the sbet point for the provided time.
    ///
    /// # Examples
    ///
    /// ```
    /// let trajectory = leeward::Trajectory::from_path("examples/sbet.out").unwrap();
    /// let point = trajectory.point(400824.9102).unwrap();
    /// ```
    pub fn point(&self, time: f64) -> Option<&Point> {
        let quantized_time = self.quantize(time);
        self.index
            .get(&quantized_time)
            .and_then(|index| self.points.get(*index))
    }

    fn quantize(&self, time: f64) -> i64 {
        quantize(time, self.level)
    }
}

impl From<Vec<Point>> for Trajectory {
    fn from(vec: Vec<Point>) -> Trajectory {
        Trajectory::new(vec, None)
    }
}

fn quantize(time: f64, level: f64) -> i64 {
    (time * level).round() as i64
}

fn calculate_level(points: &[Point]) -> f64 {
    let mut delta = 0.0;
    for (a, b) in points.iter().zip(points.iter().skip(1)) {
        delta += b.time - a.time;
    }
    (points.len() - 1) as f64 / delta
}
