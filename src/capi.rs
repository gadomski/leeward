//! Leeward's C API.

use crate::{Config, Trajectory};
use libc::c_char;
use std::{ffi::CStr, ptr};

/// An opaque structure for computing TPU.
#[derive(Debug)]
pub struct Leeward {
    config: Config,
    trajectory: Trajectory,
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
