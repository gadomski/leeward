use crate::{Config, Point, Uncertainty};

/// A lidar measurement.
///
/// More than just a point, a `Measurement` contains system configuration and platform orientation information as well.
#[derive(Debug)]
pub struct Measurement {
    las: Point,
}

impl Measurement {
    /// Creates a new measurement from the sbet, las, and config.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Config, Measurement};
    /// let measurement = Measurement::new(&sbet::Point::default(), &las::Point::default(), &Config::default());
    /// ```
    pub fn new(_sbet: &sbet::Point, las: &las::Point, _config: &Config) -> Measurement {
        Measurement {
            las: Point {
                x: las.x,
                y: las.y,
                z: las.z,
            },
        }
    }

    /// Returns the las point.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].las_point();
    /// ```
    pub fn las_point(&self) -> Point {
        self.las
    }

    /// Returns the uncertainty structure for this measurement.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let uncertainty = measurements[0].uncertainty();
    /// ```
    pub fn uncertainty(&self) -> Uncertainty {
        Uncertainty {}
    }
}
