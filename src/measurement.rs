use crate::{utils, Config, Uncertainty};
use nalgebra::{Matrix3, Vector3};

/// A lidar measurement.
///
/// More than just a point, a `Measurement` contains system configuration and platform orientation information as well.
#[derive(Debug)]
pub struct Measurement {
    las: Vector3<f64>,
    gnss: Vector3<f64>,
    imu: Matrix3<f64>,
    ned_to_enu: Matrix3<f64>,
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
            las: Vector3::new(las.x, las.y, las.z),
            gnss: Vector3::new(easting, northing, sbet.altitude),
            imu: utils::rotation_matrix(sbet.roll, sbet.pitch, sbet.yaw),
            ned_to_enu: Matrix3::new(0., 1., 0., 1., 0., 0., 0., 0., -1.),
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
    pub fn las_point(&self) -> Vector3<f64> {
        self.las
    }

    /// Returns the las point in platform coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].las_platform();
    /// ```
    pub fn las_platform(&self) -> Vector3<f64> {
        self.imu.transpose() * self.ned_to_enu.transpose() * (self.las - self.gnss)
    }

    /// Returns this measurement's gnss point in projected coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].gnss_point();
    /// ```
    pub fn gnss_point(&self) -> Vector3<f64> {
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
