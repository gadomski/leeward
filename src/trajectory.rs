use anyhow::Error;
use sbet::{Point, Reader};
use std::{collections::HashMap, path::Path};

/// A plane's trajectory.
#[derive(Debug)]
pub struct Trajectory {
    points: HashMap<i64, Point>,
    scale: f64,
}

impl Trajectory {
    /// Reads a trajectory from a path.
    ///
    /// # Examples
    ///
    /// ```
    /// # pub use leeward::Trajectory;
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// ```
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Trajectory, Error> {
        let reader = Reader::from_path(path)?;
        let mut scale = 0.;
        let mut last_time: Option<f64> = None;
        let mut points = vec![];
        for result in reader {
            let point = result?;
            let time = point.time;
            if let Some(last_time) = last_time {
                scale = (time - last_time).max(scale);
            }
            last_time = Some(time);
            points.push(point);
        }
        let mut map = HashMap::new();
        for point in points {
            let index = index(point.time, scale);
            map.insert(index, point);
        }
        Ok(Trajectory { points: map, scale })
    }

    /// Gets an sbet point for the given time.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Trajectory;
    /// let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
    /// assert!(trajectory.get(400825.80571932).is_some());
    /// assert!(trajectory.get(600825.80571932).is_none());
    /// ```
    pub fn get(&self, time: f64) -> Option<&Point> {
        let index = index(time, self.scale);
        self.points.get(&index)
    }
}

fn index(time: f64, scale: f64) -> i64 {
    (time / scale).round() as i64
}
