//! Leeward's C API.

use crate::{Config, Lidar, Measurement, Trajectory};
use anyhow::{anyhow, Error};
use libc::c_char;
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
pub struct LeewardLidar {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub scan_angle: f64,
    pub time: f64,
}

/// A vector.
#[repr(C)]
#[derive(Debug)]
pub struct LeewardNormal {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Structure containting information about the uncertainty calculation.
#[repr(C)]
#[derive(Debug)]
pub struct LeewardUncertainty {
    x: f64,
    y: f64,
    horizontal: f64,
    vertical: f64,
    total: f64,
    incidence_angle: f64,
}

impl Leeward {
    fn measurement(&self, lidar: &LeewardLidar) -> Result<Measurement, Error> {
        let sbet = self
            .trajectory
            .point(lidar.time)
            .ok_or_else(|| anyhow!("no sbet point found for time: {}", lidar.time))?;
        Ok(Measurement::new(lidar, sbet, self.config))
    }
}

impl LeewardUncertainty {
    fn new(measurement: Measurement) -> Result<LeewardUncertainty, Error> {
        let uncertainty = measurement.uncertainty()?;
        Ok(LeewardUncertainty {
            x: uncertainty.x,
            y: uncertainty.y,
            horizontal: uncertainty.horizontal,
            vertical: uncertainty.vertical,
            total: uncertainty.total,
            incidence_angle: uncertainty.incidence_angle.unwrap_or_else(|| std::f64::NAN),
        })
    }
}

impl From<&LeewardLidar> for Lidar {
    fn from(lidar: &LeewardLidar) -> Lidar {
        Lidar {
            x: lidar.x,
            y: lidar.y,
            z: lidar.z,
            scan_angle: Some(lidar.scan_angle),
        }
    }
}

/// Creates a new opaque leeward structure.
///
/// Make sure to delete the structure after use.
///
/// # Examples
///
/// ```c
/// Leeward leeward = leeward_new("data/sbet.out", "data/config.toml");
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
    lidar: *const LeewardLidar,
) -> *mut LeewardUncertainty {
    uncertainty(leeward, lidar, None)
}

/// Calculates TPU for a las point with the provided normal.
#[no_mangle]
pub extern "C" fn leeward_uncertainty_with_normal(
    leeward: *const Leeward,
    lidar: *const LeewardLidar,
    normal: *const LeewardNormal,
) -> *mut LeewardUncertainty {
    uncertainty(leeward, lidar, Some(normal))
}

fn uncertainty(
    leeward: *const Leeward,
    lidar: *const LeewardLidar,
    normal: Option<*const LeewardNormal>,
) -> *mut LeewardUncertainty {
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
    if lidar.is_null() {
        eprintln!("leeward c api error while computing uncertainty: lidar point is null");
        return ptr::null_mut();
    }
    let lidar = match unsafe { lidar.as_ref() } {
        Some(lidar) => lidar,
        None => {
            eprintln!("leeward c api error while computing uncertainty: unable to get reference to lidar structure");
            return ptr::null_mut();
        }
    };
    let mut measurement = match leeward.measurement(lidar) {
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
        measurement.set_normal(normal.x, normal.y, normal.z);
    }
    let uncertainty = match LeewardUncertainty::new(measurement) {
        Ok(uncertainty) => uncertainty,
        Err(err) => {
            eprintln!("leeward c api error while computing uncertainty: {}", err);
            return ptr::null_mut();
        }
    };
    Box::into_raw(Box::new(uncertainty))
}

/// Deletes an leeward uncertainty structure.
#[no_mangle]
pub extern "C" fn leeward_uncertainty_free(uncertainty: *mut LeewardUncertainty) {
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
    use crate::capi::{self, LeewardLidar, LeewardNormal};
    use std::ffi::CString;

    #[test]
    fn basic_usage() {
        let sbet = CString::new("data/sbet.out").unwrap();
        let config = CString::new("data/config.toml").unwrap();
        let leeward = capi::leeward_new(sbet.as_ptr(), config.as_ptr());
        assert!(!leeward.is_null());

        let lidar = LeewardLidar {
            x: 320000.34,
            y: 4181319.35,
            z: 2687.58,
            scan_angle: 22.,
            time: 400825.8057,
        };
        let uncertainty = capi::leeward_uncertainty(leeward, &lidar);
        assert!(!uncertainty.is_null());
        capi::leeward_uncertainty_free(uncertainty);

        let normal = LeewardNormal {
            x: 0.,
            y: 0.,
            z: 1.,
        };
        let uncertainty = capi::leeward_uncertainty_with_normal(leeward, &lidar, &normal);
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
