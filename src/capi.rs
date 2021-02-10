//! Leeward's C API.
//!
//! Used to interaction w/ PDAL via https://github.com/gadomski/leeward-pdal.

use crate::{Config, Lasish, Measurement, Point, Trajectory};
use anyhow::Error;
use libc::c_char;
use std::{ffi::CStr, ptr};

/// Creates a new opaque leeward structure for the given trajectory and configuration.
///
/// # Examples
///
/// ```
/// # use leeward::capi;
/// # use std::ffi::CString;
/// let sbet = CString::new("data/sbet.out").unwrap();
/// let config = CString::new("data/config.toml").unwrap();
/// let leeward = capi::leeward_new(sbet.as_ptr(), config.as_ptr());
/// assert!(!leeward.is_null());
/// capi::leeward_free(leeward);
/// ```
#[no_mangle]
pub extern "C" fn leeward_new(sbet: *const c_char, config: *const c_char) -> *mut Leeward {
    let config = unsafe { CStr::from_ptr(config) }.to_string_lossy();
    let config = match Config::from_path(&*config) {
        Ok(config) => config,
        Err(e) => {
            eprintln!(
                "leeward c api error while reading config file {}: {}",
                config, e
            );
            return ptr::null_mut();
        }
    };
    let sbet = unsafe { CStr::from_ptr(sbet) }.to_string_lossy();
    let trajectory = match Trajectory::from_path(&*sbet) {
        Ok(config) => config,
        Err(e) => {
            eprintln!(
                "leeward c api error while reading sbet file {}: {}",
                sbet, e
            );
            return ptr::null_mut();
        }
    };
    let leeward = Leeward { trajectory, config };
    Box::into_raw(Box::new(leeward))
}

/// Returns a measurement, which contains a slew of information about the provided las point.
///
/// # Examples
///
/// ```
/// # use leeward::capi;
/// # use std::ffi::CString;
/// let sbet = CString::new("data/sbet.out").unwrap();
/// let config = CString::new("data/config.toml").unwrap();
/// let leeward = capi::leeward_new(sbet.as_ptr(), config.as_ptr());
/// let point = capi::LeewardPoint {
///     x: 320000.34,
///     y: 4181319.35,
///     z: 2687.58,
///     scan_angle: 22.,
///     time: 400825.8057,
/// };
/// let measurement = capi::leeward_measurement(leeward, point);
/// assert!(!measurement.is_null());
/// capi::leeward_measurement_free(measurement);
/// capi::leeward_free(leeward);
/// ```
#[no_mangle]
pub extern "C" fn leeward_measurement(
    leeward: *mut Leeward,
    point: LeewardPoint,
    normal: LeewardNormal,
) -> *mut LeewardMeasurement {
    if leeward.is_null() {
        eprintln!("leeward c api error: leeward pointer is null");
        return ptr::null_mut();
    }
    let leeward = match unsafe { leeward.as_ref() } {
        Some(leeward) => leeward,
        None => {
            eprintln!("leeward c api error: could not get reference to leeward object");
            return ptr::null_mut();
        }
    };
    let measurement = match leeward.measurement(point, normal) {
        Ok(measurement) => measurement,
        Err(err) => {
            eprintln!("leeward c api error: {}", err);
            return ptr::null_mut();
        }
    };
    Box::into_raw(Box::new(measurement))
}

/// Free an allocated `LeewardMeasurement` structure.
#[no_mangle]
pub extern "C" fn leeward_measurement_free(measurement: *mut LeewardMeasurement) {
    if measurement.is_null() {
        // pass
    } else {
        drop(unsafe { Box::from_raw(measurement) })
    }
}

/// Free an allocated `Leeward` structure.
#[no_mangle]
pub extern "C" fn leeward_free(leeward: *mut Leeward) {
    if leeward.is_null() {
        // pass
    } else {
        drop(unsafe { Box::from_raw(leeward) });
    }
}

/// An opaque structure for performing leeward operations from C.
#[derive(Debug)]
pub struct Leeward {
    config: Config,
    trajectory: Trajectory,
}

/// A structure to contain only the essential bits of a lidar point.
///
/// "Essential" only goes as far as this application, of course.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct LeewardPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub scan_angle: f64,
    pub time: f64,
}

/// A unit normal.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct LeewardNormal {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// An structure that contains all the information that leeward can calculate for a point.
#[derive(Debug)]
#[repr(C)]
pub struct LeewardMeasurement {
    horizontal_uncertainty: f64,
    vertical_uncertainty: f64,
    total_uncertainty: f64,
    incidence_angle: f64,
}

impl Leeward {
    fn measurement(
        &self,
        point: LeewardPoint,
        normal: LeewardNormal,
    ) -> Result<LeewardMeasurement, Error> {
        Measurement::new(&self.trajectory, point, self.config)
            .and_then(|m| LeewardMeasurement::new(m, normal))
    }
}

impl Lasish for LeewardPoint {
    fn time(&self) -> Option<f64> {
        Some(self.time)
    }

    fn x(&self) -> f64 {
        self.x
    }

    fn y(&self) -> f64 {
        self.y
    }

    fn z(&self) -> f64 {
        self.z
    }

    fn scan_angle(&self) -> f64 {
        self.scan_angle
    }
}

impl LeewardMeasurement {
    fn new(
        measurement: Measurement<LeewardPoint>,
        normal: LeewardNormal,
    ) -> Result<LeewardMeasurement, Error> {
        let tpu = measurement.tpu(normal.into())?;
        Ok(LeewardMeasurement {
            horizontal_uncertainty: tpu.horizontal,
            vertical_uncertainty: tpu.vertical,
            total_uncertainty: tpu.total,
            incidence_angle: tpu.incidence_angle,
        })
    }
}

impl From<LeewardNormal> for Point {
    fn from(normal: LeewardNormal) -> Point {
        Point::new(normal.x, normal.y, normal.z)
    }
}

#[cfg(test)]
mod tests {
    use std::ffi::CString;

    #[test]
    fn new() {
        let sbet = CString::new("data/sbet.out").unwrap();
        let config = CString::new("data/config.toml").unwrap();
        let leeward = super::leeward_new(sbet.as_ptr(), config.as_ptr());
        assert!(!leeward.is_null());
        super::leeward_free(leeward);
    }

    #[test]
    fn measurement() {
        let sbet = CString::new("data/sbet.out").unwrap();
        let config = CString::new("data/config.toml").unwrap();
        let leeward = super::leeward_new(sbet.as_ptr(), config.as_ptr());
        let point = super::LeewardPoint {
            x: 320000.34,
            y: 4181319.35,
            z: 2687.58,
            scan_angle: 22.,
            time: 400825.8057,
        };
        let normal = super::LeewardNormal {
            x: 0.,
            y: 0.,
            z: 0.,
        };
        let measurement = super::leeward_measurement(leeward, point, normal);
        assert!(!measurement.is_null());
        super::leeward_measurement_free(measurement);
        super::leeward_free(leeward);
    }
}
