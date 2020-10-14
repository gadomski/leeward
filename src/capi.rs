use crate::Trajectory;
use libc::c_char;
use std::ffi::CStr;
use std::ptr;

pub struct Leeward {
    trajectory: Trajectory,
}

#[no_mangle]
pub extern "C" fn leeward_new(path: *const c_char, quantization: u32) -> *mut Leeward {
    let path = unsafe { CStr::from_ptr(path) };
    let trajectory = match Trajectory::new(path.to_string_lossy().into_owned(), Some(quantization))
    {
        Ok(trajectory) => trajectory,
        Err(err) => {
            eprintln!("{}", err);
            return ptr::null_mut();
        }
    };
    Box::into_raw(Box::new(Leeward {
        trajectory: trajectory,
    }))
}

#[no_mangle]
pub extern "C" fn leeward_delete(leeward: *mut Leeward) {
    if leeward.is_null() {
        // pass
    } else {
        let _deleted = unsafe { Box::from_raw(leeward) };
    }
}
