//! Lidar Equation Engine With Already Racked Derivatives
//!
//! # Step 1: Read a trajectory file
//!
//! **leeward** currently supports Smoothed Best Estimate of Trajectory (sbet)
//! files, which usually have a .out extension.
//!
//! ```
//! use leeward::Trajectory;
//! let trajectory = Trajectory::from_path("data/sbet.out").unwrap();
//! ```

mod trajectory;

pub use trajectory::Trajectory;
