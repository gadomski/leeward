use anyhow::Error;
use sbet::{Point, Reader};
use std::path::Path;

/// A plane's trajectory.
#[derive(Debug)]
pub struct Trajectory {
    points: Vec<Point>,
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
        Ok(Trajectory {
            points: reader.collect::<Result<_, Error>>()?,
        })
    }
}
