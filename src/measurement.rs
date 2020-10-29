use crate::{Config, Point, Uncertainty};

/// A lidar measurement.
///
/// More than just a point, a `Measurement` contains system configuration and platform orientation information as well.
#[derive(Debug)]
pub struct Measurement {
    las: Point,
    gnss: Point,
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
    pub fn new(sbet: &sbet::Point, las: &las::Point, config: &Config) -> Measurement {
        let (northing, easting, _) =
            utm::radians_to_utm_wgs84(sbet.latitude, sbet.longitude, config.utm_zone);
        Measurement {
            las: Point {
                x: las.x,
                y: las.y,
                z: las.z,
            },
            gnss: Point {
                x: easting,
                y: northing,
                z: sbet.altitude,
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

    /// Returns this measurement's gnss point in projected coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].gnss_point();
    /// ```
    pub fn gnss_point(&self) -> Point {
        self.gnss
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
