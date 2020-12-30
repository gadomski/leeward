//! Adjust configuration variables to align computed points with the actual points.
use crate::{Config, Dimension, Measurement, Variable};
use anyhow::{anyhow, Error};
use nalgebra::{DMatrix, DVector};

const DEFAULT_TOLERANCE: f64 = 1e-6;
const DEFAULT_VARIABLES: [Variable; 3] = [
    Variable::BoresightRoll,
    Variable::BoresightPitch,
    Variable::BoresightYaw,
];

/// Adjustment structure.
#[derive(Debug)]
pub struct Adjustment {
    config: Config,
    measurements: Vec<Measurement>,
    rmse: f64,
    residuals: DVector<f64>,
    tolerance: f64,
    variables: Vec<Variable>,
}

impl Adjustment {
    /// Creates a new adjustment for the provided measurements.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Adjustment;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let adjustment = Adjustment::new(measurements).unwrap();
    /// ```
    pub fn new(measurements: Vec<Measurement>) -> Result<Adjustment, Error> {
        if measurements.is_empty() {
            return Err(anyhow!("cannot create adjustment with no measurements"));
        }
        let config = measurements[0].config();
        let mut residuals = DVector::zeros(measurements.len() * 3);
        for (i, measurement) in measurements.iter().enumerate() {
            if measurement.config() != config {
                return Err(anyhow!("not all measurements have the same config"));
            }
            let rs = measurement.residuals();
            for (j, &residual) in rs.iter().enumerate() {
                residuals[i * 3 + j] = residual;
            }
        }
        let rmse = residuals.norm();
        Ok(Adjustment {
            config,
            rmse,
            residuals,
            measurements,
            tolerance: DEFAULT_TOLERANCE,
            variables: DEFAULT_VARIABLES.to_vec(),
        })
    }

    /// Adjusts these measurements' configuration to optimally align the points.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Adjustment;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let adjustment = Adjustment::new(measurements).unwrap();
    /// let config = adjustment.adjust().unwrap();
    /// ```
    pub fn adjust(&self) -> Result<Config, Error> {
        let next_adjustment = self.next_adjustment()?;
        let delta = self.rmse - next_adjustment.rmse;
        if delta > self.tolerance {
            Ok(self.config)
        } else {
            next_adjustment.adjust()
        }
    }

    fn next_adjustment(&self) -> Result<Adjustment, Error> {
        let mut jacobian = DMatrix::zeros(self.residuals.len(), self.variables.len());
        for (i, measurement) in self.measurements.iter().enumerate() {
            for (j, dimension) in Dimension::iter().enumerate() {
                for (k, &variable) in self.variables.iter().enumerate() {
                    jacobian[(i * 3 + j, k)] =
                        measurement.partial_derivative_in_body_frame(dimension, variable);
                }
            }
        }
        let values = self.config.values(&self.variables)?;
        unimplemented!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_measurements() {
        assert!(Adjustment::new(vec![]).is_err());
    }

    #[test]
    fn different_config() {
        let mut measurements =
            crate::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let mut new_config = measurements[0].config();
        new_config.lever_arm.x = new_config.lever_arm.x + 1.;
        measurements[0] = measurements[0].with_config(new_config);
        assert!(Adjustment::new(measurements).is_err());
    }
}
