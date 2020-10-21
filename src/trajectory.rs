use crate::{Config, Measurement};
use anyhow::{anyhow, Error};
use sbet::Point;
use std::collections::HashMap;
use std::path::Path;

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
        Trajectory {
            index: build_index(&points, level),
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
        let quantized_time = self.quantize(time);
        let index = self
            .index
            .get(&quantized_time)
            .ok_or_else(|| anyhow!("Could not find SBET point for time {}", time))?;
        let sbet = self
            .points
            .get(*index)
            .ok_or_else(|| anyhow!("Invalid index, no sbet point for index value {}", index))?;
        Ok(Measurement::new(las, *sbet, config))
    }

    /// Rebuilds the index with the given quantization level.
    ///
    /// # Examples
    ///
    /// ```
    /// use leeward::Trajectory;
    /// let mut trajectory = Trajectory::from_path("examples/sbet.out").unwrap();
    /// trajectory.rebuild_index(100.0);
    /// ```
    pub fn rebuild_index(&mut self, level: f64) {
        self.index = build_index(&self.points, level);
        self.level = level;
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

fn build_index(points: &[Point], level: f64) -> HashMap<i64, usize> {
    let mut index = HashMap::new();
    for (i, point) in points.iter().enumerate() {
        index.insert(quantize(point.time, level), i);
    }
    index
}

fn calculate_level(points: &[Point]) -> f64 {
    let mut delta = 0.0;
    for (a, b) in points.iter().zip(points.iter().next()) {
        delta += b.time - a.time;
    }
    delta / (points.len() - 1) as f64
}
