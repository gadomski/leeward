//! The C API for leeward.
//!
//! # Examples
//!
//! Create an opaque `Leeward` structure, use it to calculate TPU, then clean up after yourself:
//!
//! ```
//! use std::ffi::CString;
//! use leeward::capi;
//! use las::Read;
//!
//! let sbet_path = CString::new("examples/sbet.out").unwrap();
//! let config_path = CString::new("examples/config.toml").unwrap();
//! let leeward = capi::leeward_new(sbet_path.as_ptr(), config_path.as_ptr());
//! let point = las::Reader::from_path("examples/one-point.las").unwrap().points().next().unwrap().unwrap();
//! let tpu = capi::leeward_tpu(leeward, point.x, point.y, point.z, point.scan_angle, point.gps_time.unwrap());
//! unsafe { println!("sigma_magnitude={}", (*tpu).sigma_magnitude) };
//! capi::leeward_tpu_delete(tpu);
//! capi::leeward_delete(leeward);
//! ```

use crate::{Config, Measurement, Trajectory};
use anyhow::Error;
use libc::{c_char, c_double};
use std::ffi::CStr;
use std::ptr;

/// An opaque structure for computing TPU from C.
pub struct Leeward {
    config: Config,
    trajectory: Trajectory,
}

impl Leeward {
    fn measurement(&self, point: las::Point) -> Result<Measurement, Error> {
        self.trajectory.measurement(point, self.config)
    }
}

#[repr(C)]
#[allow(missing_docs)]
/// A collection of the TPU uncertainties, as standard deviations (not variances).
pub struct LeewardTpu {
    pub sigma_x: f64,
    pub sigma_y: f64,
    pub sigma_horizontal: f64,
    pub sigma_vertical: f64,
    pub sigma_magnitude: f64,
}

/// Create a new `leeward::capi::Leeward` opaque structure.
///
/// Returns null if it cannot be created.
///
/// # Examples
///
/// Make sure to clean up after yourself:
///
/// ```
/// use std::ffi::CString;
/// use leeward::capi;
///
/// let sbet_path = CString::new("examples/sbet.out").unwrap();
/// let config_path = CString::new("examples/config.toml").unwrap();
/// let leeward = capi::leeward_new(sbet_path.as_ptr(), config_path.as_ptr());
/// assert!(!leeward.is_null());
/// capi::leeward_delete(leeward);
/// ```
///
/// An error means we return a null pointer, no need to clean up:
///
/// ```
/// # use std::ffi::CString;
/// # use leeward::capi;
/// let sbet_path = CString::new("this/dosnt/exist").unwrap();
/// # let config_path = CString::new("examples/config.toml").unwrap();
/// let leeward = capi::leeward_new(sbet_path.as_ptr(), config_path.as_ptr());
/// assert!(leeward.is_null());
/// ```
#[no_mangle]
pub extern "C" fn leeward_new(
    sbet_path: *const c_char,
    config_path: *const c_char,
) -> *mut Leeward {
    let sbet_path = unsafe { CStr::from_ptr(sbet_path) };
    let trajectory = match Trajectory::from_path(sbet_path.to_string_lossy().into_owned()) {
        Ok(trajectory) => trajectory,
        Err(err) => {
            eprintln!("{}", err);
            return ptr::null_mut();
        }
    };
    let config_path = unsafe { CStr::from_ptr(config_path) };
    let config = match Config::from_path(config_path.to_string_lossy().into_owned()) {
        Ok(config) => config,
        Err(err) => {
            eprintln!("{}", err);
            return ptr::null_mut();
        }
    };
    Box::into_raw(Box::new(Leeward {
        config: config,
        trajectory: trajectory,
    }))
}

/// Returns the TPU for a given las point.
#[no_mangle]
pub extern "C" fn leeward_tpu(
    leeward: *mut Leeward,
    x: c_double,
    y: c_double,
    z: c_double,
    scan_angle: f32,
    gps_time: f64,
) -> *mut LeewardTpu {
    if leeward.is_null() {
        return ptr::null_mut();
    }
    let point = las::Point {
        x,
        y,
        z,
        scan_angle,
        gps_time: Some(gps_time),
        ..Default::default()
    };
    let leeward = match unsafe { leeward.as_ref() } {
        Some(leeward) => leeward,
        None => {
            eprintln!("Unable to get reference to leeward object...");
            return ptr::null_mut();
        }
    };
    match leeward.measurement(point) {
        Ok(measurement) => {
            let covariance = measurement.tpu();
            let tpu = LeewardTpu {
                sigma_x: covariance[(0, 0)].sqrt(),
                sigma_y: covariance[(1, 1)].sqrt(),
                sigma_horizontal: (covariance[(0, 0)] + covariance[(1, 1)]).sqrt(),
                sigma_vertical: covariance[(2, 2)].sqrt(),
                sigma_magnitude: (covariance[(0, 0)] + covariance[(1, 1)] + covariance[(2, 2)])
                    .sqrt(),
            };
            Box::into_raw(Box::new(tpu))
        }
        Err(err) => {
            eprintln!("{}", err);
            return ptr::null_mut();
        }
    }
}

/// Deletes a TPU structure.
#[no_mangle]
pub extern "C" fn leeward_tpu_delete(tpu: *mut LeewardTpu) {
    if tpu.is_null() {
        // pass
    } else {
        let _deleted = unsafe { Box::from_raw(tpu) };
    }
}

/// Deletes a leeward structure.
#[no_mangle]
pub extern "C" fn leeward_delete(leeward: *mut Leeward) {
    if leeward.is_null() {
        // pass
    } else {
        let _deleted = unsafe { Box::from_raw(leeward) };
    }
}
