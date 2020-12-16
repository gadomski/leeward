use anyhow::{anyhow, Error};
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
        let mut scale = std::f64::MAX;
        let mut last_time = 0.;
        let mut points = vec![];
        for result in reader {
            let point = result?;
            let time = point.time;
            scale = (time - last_time).min(scale);
            last_time = time;
            points.push(point);
        }
        let mut map = HashMap::new();
        for point in points {
            let index = index(point.time, scale);
            if let Some(previous) = map.insert(index, point) {
                return Err(anyhow!(
                    "duplicate sbet point for index={}, previous.time={}, point.time={}",
                    index,
                    previous.time,
                    point.time
                ));
            }
        }
        Ok(Trajectory { points: map, scale })
    }

    /// Gets an sbet point for the given time.
    ///
    /// # Examples
    ///
    /// TODO
    pub fn get(&self, time: f64) -> Option<&Point> {
        let index = index(time, self.scale);
        self.points.get(&index)
    }
}

fn index(time: f64, scale: f64) -> i64 {
    (time / scale).round() as i64
}
