//! Utility functions.

use crate::{Lasish, Measurement, Point};

/// Fits a bunch of measurements to a plane in the platform's body frame.
///
/// Returns each measurement projected onto the plane, with the z value being the distance from the plane.
///
/// # Examples
///
/// ```
/// # use leeward::utils;
/// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
/// let points = utils::fit_to_plane_in_body_frame(measurements);
/// assert_eq!(measurements.len(), points.len());
/// ```
pub fn fit_to_plane_in_body_frame<L: Lasish>(_measurements: &[Measurement<L>]) -> Vec<Point> {
    unimplemented!()
}
