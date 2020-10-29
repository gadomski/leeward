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
    boresight: Matrix3<f64>,
    lever_arm: Vector3<f64>,
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
        let boresight = config.boresight.to_rotation_matrix();
        Measurement {
            las: Vector3::new(las.x, las.y, las.z),
            gnss: Vector3::new(easting, northing, sbet.altitude),
            imu: utils::rotation_matrix(sbet.roll, sbet.pitch, sbet.yaw),
            ned_to_enu: Matrix3::new(0., 1., 0., 1., 0., 0., 0., 0., -1.),
            boresight,
            lever_arm: config.lever_arm,
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

    /// Returns the measured point as calculated through the lidar equation.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].gnss_point();
    /// ```
    pub fn calculated(&self) -> Vector3<f64> {
        self.gnss + self.ned_to_enu * self.imu * (self.boresight * self.scanner() - self.lever_arm)
    }

    /// Returns the measured point in the scanner's coordinate system.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let point = measurements[0].scanner();
    /// ```
    pub fn scanner(&self) -> Vector3<f64> {
        let range = self.range();
        let scan_angle = self.scan_angle();
        Vector3::new(range * scan_angle.cos(), 0., range * scan_angle.sin())
    }

    /// Returns this measurement's range.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let range = measurements[0].range();
    /// ```
    pub fn range(&self) -> f64 {
        (self.las - self.scanner_origin()).norm()
    }

    /// Returns this measurement's scan angle.
    ///
    /// # Examples
    ///
    /// ```
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let scan_angle = measurements[0].scan_angle();
    /// ```
    pub fn scan_angle(&self) -> f64 {
        ((self.boresight.transpose() * (self.lever_arm + self.las_platform())).z / self.range())
            .asin()
    }

    /// Returns this measurement's scanner origin in global coordinates.
    ///
    /// Lever arm.
    pub fn scanner_origin(&self) -> Vector3<f64> {
        self.gnss + self.ned_to_enu * self.imu * (-self.lever_arm)
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
