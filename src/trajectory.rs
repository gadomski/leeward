use anyhow::Error;
use sbet::Point;
use std::{collections::HashMap, path::Path};

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
