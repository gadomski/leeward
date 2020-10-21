use crate::{Config, Measurement, Trajectory};
use anyhow::Error;
use libc::{c_char, c_double};
use std::ffi::CStr;
use std::ptr;

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
pub struct LeewardTpu {
    sigma_x: f64,
    sigma_y: f64,
    sigma_horizontal: f64,
    sigma_vertical: f64,
    sigma_magnitude: f64,
}

#[no_mangle]
pub extern "C" fn leeward_new(
    sbet_path: *const c_char,
    config_path: *const c_char,
    level: f64,
) -> *mut Leeward {
    let sbet_path = unsafe { CStr::from_ptr(sbet_path) };
    let trajectory = match Trajectory::from_path(sbet_path.to_string_lossy().into_owned()) {
        Ok(mut trajectory) => {
            trajectory.rebuild_index(level);
            trajectory
        }
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

#[no_mangle]
pub extern "C" fn leeward_tpu_delete(tpu: *mut LeewardTpu) {
    if tpu.is_null() {
        // pass
    } else {
        let _deleted = unsafe { Box::from_raw(tpu) };
    }
}

#[no_mangle]
pub extern "C" fn leeward_delete(leeward: *mut Leeward) {
    if leeward.is_null() {
        // pass
    } else {
        let _deleted = unsafe { Box::from_raw(leeward) };
    }
}
