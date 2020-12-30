//! Adjust configuration variables to align computed points with the actual points.
use crate::Measurement;
use nalgebra::DVector;

/// Computes the residuals for all measurements.
///
/// Residuals are computed for all three dimensions, so the residual's vector length is three times the number of measurements.
///
/// # Examples
///
/// ```
/// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
/// let residuals = leeward::adjust::residuals(&measurements);
/// assert_eq!(residuals.len(), measurements.len() * 3);
/// ```
pub fn residuals(measurements: &[Measurement]) -> DVector<f64> {
    let mut residuals = DVector::zeros(measurements.len() * 3);
    for (i, measurement) in measurements.iter().enumerate() {
        let rs = measurement.residuals();
        for (j, residual) in rs.iter().enumerate() {
            residuals[i * 3 + j] = *residual;
        }
    }
    residuals
}
