//! Adjust configuration variables to align computed points with the actual points.
use crate::{Config, Dimension, Measurement, Variable};
use anyhow::{anyhow, Error};
use nalgebra::{DMatrix, DVector};

const DEFAULT_TOLERANCE: f64 = 1e-6;
const BORESIGHT_VARIABLES: [Variable; 3] = [
    Variable::BoresightRoll,
    Variable::BoresightPitch,
    Variable::BoresightYaw,
];
const LEVER_ARM_VARIABLES: [Variable; 3] = [
    Variable::LeverArmX,
    Variable::LeverArmY,
    Variable::LeverArmZ,
];

/// Adjustor structure.
#[derive(Debug)]
pub struct Adjustor {
    measurements: Vec<Measurement>,
    rmse: f64,
    residuals: DVector<f64>,
    tolerance: f64,
    variables: Vec<Variable>,
    config: Config,
    history: Vec<Record>,
}

/// A record of a single iteration.
#[derive(Clone, Debug)]
pub struct Record {
    pub rmse: f64,
    pub variables: Vec<Variable>,
    pub values: Vec<f64>,
}

impl Adjustor {
    /// Creates a new adjustor for the provided measurements.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Adjustor;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let adjustor = Adjustor::new(measurements).unwrap();
    /// ```
    pub fn new(measurements: Vec<Measurement>) -> Result<Adjustor, Error> {
        Adjustor::new_iteration(measurements, BORESIGHT_VARIABLES.to_vec(), vec![])
    }

    /// Switch this adjustor to adjust the lever arm.
    ///
    /// By default, adjusts boresight.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Adjustor;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let mut adjustor = Adjustor::new(measurements).unwrap();
    /// adjustor.adjust_lever_arm(true);
    /// ```
    pub fn adjust_lever_arm(&mut self, adjust_lever_arm: bool) {
        if adjust_lever_arm {
            self.variables = LEVER_ARM_VARIABLES.to_vec();
        } else {
            self.variables = BORESIGHT_VARIABLES.to_vec();
        }
    }

    fn new_iteration(
        measurements: Vec<Measurement>,
        variables: Vec<Variable>,
        mut history: Vec<Record>,
    ) -> Result<Adjustor, Error> {
        if measurements.is_empty() {
            return Err(anyhow!("cannot create adjustor with no measurements"));
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
        let values = config.values(&variables)?;
        history.push(Record {
            rmse,
            variables: variables.clone(),
            values: values.iter().map(|&v| v).collect(),
        });
        Ok(Adjustor {
            rmse,
            residuals,
            measurements,
            variables,
            tolerance: DEFAULT_TOLERANCE,
            history,
            config,
        })
    }

    pub fn rmse(&self) -> f64 {
        self.rmse
    }

    pub fn config(&self) -> Config {
        self.config
    }

    /// Adjusts these measurements' configuration to optimally align the points.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Adjustor;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let mut adjustor = Adjustor::new(measurements).unwrap();
    /// let adjustment = adjustor.adjust().unwrap();
    /// ```
    pub fn adjust(self) -> Result<Adjustor, Error> {
        let next = self.next()?;
        let delta = self.rmse - next.rmse;
        if delta < self.tolerance {
            Ok(self)
        } else {
            next.adjust()
        }
    }

    fn next(&self) -> Result<Adjustor, Error> {
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
        let values = (jacobian.transpose() * &jacobian)
            .try_inverse()
            .ok_or(anyhow!("no inverse found"))?
            * jacobian.transpose()
            * (&jacobian * values - &self.residuals);
        let config = self
            .config
            .with_values(&self.variables, values.as_slice())?;
        let measurements = self
            .measurements
            .iter()
            .map(|m| m.with_config(config))
            .collect();
        Adjustor::new_iteration(measurements, self.variables.clone(), self.history.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_measurements() {
        assert!(Adjustor::new(vec![]).is_err());
    }

    #[test]
    fn different_config() {
        let mut measurements =
            crate::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let mut new_config = measurements[0].config();
        new_config.lever_arm.x = new_config.lever_arm.x + 1.;
        measurements[0] = measurements[0].with_config(new_config);
        assert!(Adjustor::new(measurements).is_err());
    }

    #[test]
    fn adjust() {
        let measurements =
            crate::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let adjustor = Adjustor::new(measurements).unwrap().adjust().unwrap();
        assert!(adjustor.rmse < 14.);
    }
}
