//! Utility functions.

use crate::{Lasish, Matrix3, Measurement, Point};
use nalgebra::{Dynamic, MatrixMN, U3};

/// Fits a bunch of measurements to a plane in the platform's body frame.
///
// Returns each measurement projected onto the plane, with the z value being the distance from the plane.
///
/// # Examples
///
/// ```
/// # use leeward::utils;
/// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
/// let points = utils::fit_to_plane_in_body_frame(measurements);
/// assert_eq!(measurements.len(), points.len());
/// ```
pub fn fit_to_plane_in_body_frame<L: Lasish>(measurements: &[Measurement<L>]) -> Vec<Point> {
    let mut points = MatrixMN::<f64, Dynamic, U3>::zeros(measurements.len());
    for (i, measurement) in measurements.iter().enumerate() {
        let body_frame = measurement.body_frame();
        points[(i, 0)] = body_frame.x;
        points[(i, 1)] = body_frame.y;
        points[(i, 2)] = body_frame.z;
    }
    let centroid = points.row_mean();
    for (i, mean) in centroid.iter().enumerate() {
        points.set_column(i, &points.column(i).add_scalar(-mean));
    }
    let svd = points.transpose().svd(true, false);
    let u = svd.u.unwrap();
    let angle_to_x_axis = u.column(0).dot(&Point::new(1., 0., 0.)).acos();
    let rotation_to_zy_plane = Matrix3::new(
        angle_to_x_axis.cos(),
        -angle_to_x_axis.sin(),
        0.,
        angle_to_x_axis.sin(),
        angle_to_x_axis.cos(),
        0.,
        0.,
        0.,
        1.,
    );
    let points_as_matrix = points * rotation_to_zy_plane;
    let mut points = Vec::new();
    for i in 0..points_as_matrix.nrows() {
        points.push(Point::new(
            points_as_matrix[(i, 0)],
            points_as_matrix[(i, 1)],
            points_as_matrix[(i, 2)],
        ));
    }
    points
}

#[cfg(test)]
mod tests {
    #[test]
    fn best_fitting_plane() {
        let measurements =
            crate::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let _points = super::fit_to_plane_in_body_frame(&measurements);
    }
}
