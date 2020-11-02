//! Leeward's C API.

use crate::{Config, Measurement, Trajectory};
use anyhow::Error;
use libc::c_char;
use nalgebra::Vector3;
use std::{ffi::CStr, ptr};

/// An opaque structure for computing TPU.
#[derive(Debug)]
pub struct Leeward {
    config: Config,
    trajectory: Trajectory,
}

/// A point structure.
#[repr(C)]
#[derive(Debug)]
pub struct LasPoint {
    x: f64,
    y: f64,
    z: f64,
    gps_time: f64,
}

/// A vector.
#[repr(C)]
#[derive(Debug)]
pub struct Normal {
    x: f64,
    y: f64,
    z: f64,
}

/// Structure containting information about the uncertainty calculation.
#[repr(C)]
#[derive(Debug)]
pub struct Uncertainty {}

impl Leeward {
    fn measurement(&self, las: &LasPoint) -> Result<Measurement, Error> {
        let _point = Vector3::new(las.x, las.y, las.z);
        let _time = las.gps_time;
        unimplemented!()
    }
}

impl Uncertainty {
    fn new(_measurement: Measurement) -> Uncertainty {
        Uncertainty {}
    }
}

/// Creates a new opaque leeward structure.
///
/// Make sure to delete the structure after use.
///
/// # Examples
///
/// ```c
/// let leeward = leeward_new("data/sbet.out", "data/config.toml");
/// assert(leeward);
/// leeward_free(leeward);
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

/// Calculates TPU for a las point.
#[no_mangle]
pub extern "C" fn leeward_uncertainty(
    leeward: *const Leeward,
    las: *const LasPoint,
) -> *mut Uncertainty {
    uncertainty(leeward, las, None)
}

/// Calculates TPU for a las point with the provided normal.
#[no_mangle]
pub extern "C" fn leeward_uncertainty_with_normal(
    leeward: *const Leeward,
    las: *const LasPoint,
    normal: *const Normal,
) -> *mut Uncertainty {
    uncertainty(leeward, las, Some(normal))
}

fn uncertainty(
    leeward: *const Leeward,
    las: *const LasPoint,
    normal: Option<*const Normal>,
) -> *mut Uncertainty {
    if leeward.is_null() {
        eprintln!("leeward c api error while computing uncertainty: leeward structure is null");
        return ptr::null_mut();
    }
    let leeward = match unsafe { leeward.as_ref() } {
        Some(leeward) => leeward,
        None => {
            eprintln!("leeward c api error while computing uncertainty: unable to get reference to leeward structure");
            return ptr::null_mut();
        }
    };
    if las.is_null() {
        eprintln!("leeward c api error while computing uncertainty: las point is null");
        return ptr::null_mut();
    }
    let las = match unsafe { las.as_ref() } {
        Some(las) => las,
        None => {
            eprintln!("leeward c api error while computing uncertainty: unable to get reference to las structure");
            return ptr::null_mut();
        }
    };
    let mut measurement = match leeward.measurement(las) {
        Ok(measurement) => measurement,
        Err(e) => {
            eprintln!("leeward c api error while computing uncertainty: {}", e);
            return ptr::null_mut();
        }
    };
    if let Some(normal) = normal {
        if normal.is_null() {
            eprintln!("leeward c api error while computing uncertainty: normal is null");
            return ptr::null_mut();
        }
        let normal = match unsafe { normal.as_ref() } {
            Some(normal) => normal,
            None => {
                eprintln!("leeward c api error while computing uncertainty: unable to get reference to normal structure");
                return ptr::null_mut();
            }
        };
        let normal = Vector3::new(normal.x, normal.y, normal.z);
        measurement.set_normal(normal);
    }
    let uncertainty = Uncertainty::new(measurement);
    Box::into_raw(Box::new(uncertainty))
}

/// Deletes an leeward uncertainty structure.
#[no_mangle]
pub extern "C" fn leeward_uncertainty_free(uncertainty: *mut Uncertainty) {
    if uncertainty.is_null() {
        // pass
    } else {
        let _ = unsafe { Box::from_raw(uncertainty) };
    }
}

/// Deletes an opaque leeward structure.
#[no_mangle]
pub extern "C" fn leeward_free(leeward: *mut Leeward) {
    if leeward.is_null() {
        // pass
    } else {
        let _ = unsafe { Box::from_raw(leeward) };
    }
}

#[cfg(test)]
mod tests {
    use crate::capi;
    use std::ffi::CString;

    #[test]
    fn basic_usage() {
        let sbet = CString::new("data/sbet.out").unwrap();
        let config = CString::new("data/config.toml").unwrap();
        let leeward = capi::leeward_new(sbet.as_ptr(), config.as_ptr());
        assert!(!leeward.is_null());

        let las = capi::LasPoint {
            x: 320000.34,
            y: 4181319.35,
            z: 2687.58,
            gps_time: 400825.8057,
        };
        let uncertainty = capi::leeward_uncertainty(leeward, &las);
        assert!(!uncertainty.is_null());
        capi::leeward_uncertainty_free(uncertainty);

        let normal = capi::Normal {
            x: 0.,
            y: 0.,
            z: 1.,
        };
        let uncertainty = capi::leeward_uncertainty_with_normal(leeward, &las, &normal);
        assert!(!uncertainty.is_null());
        capi::leeward_uncertainty_free(uncertainty);

        capi::leeward_free(leeward);
    }

    #[test]
    fn invalid_paths() {
        let sbet = CString::new("data/notafile.out").unwrap();
        let config = CString::new("data/config.toml").unwrap();
        let leeward = capi::leeward_new(sbet.as_ptr(), config.as_ptr());
        assert!(leeward.is_null());

        let sbet = CString::new("data/sbet.out").unwrap();
        let config = CString::new("data/notafile.toml").unwrap();
        let leeward = capi::leeward_new(sbet.as_ptr(), config.as_ptr());
        assert!(leeward.is_null());
    }
}
