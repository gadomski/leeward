//! Adjust configuration variables to align computed points with the actual points.
//!
//! # Examples
//!
//! The `adjust` method runs the adjustment. It returns an adjustment structure,
//! in case you want to modify something and continue adjusting:
//!
//! ```
//! # use leeward::Adjust;
//! let measurements = leeward::measurements(
//!     "data/sbet.out",
//!     "data/points.las",
//!     "data/config.toml"
//! ).unwrap();
//! let adjust = Adjust::new(measurements).unwrap();
//! let adjust = adjust.adjust().unwrap();
//! ```
//!
//! The adjust structure includes its final configuration, and also has a history
//! of all iterations:
//!
//! ```
//! # use leeward::Adjust;
//! # let measurements = leeward::measurements(
//! #     "data/sbet.out",
//! #     "data/points.las",
//! #     "data/config.toml"
//! # ).unwrap();
//! # let adjust = Adjust::new(measurements).unwrap();
//! # let adjust = adjust.adjust().unwrap();
//! let config = adjust.config();
//! let history = adjust.history();
//! let last_iteration = history.last().unwrap();
//! let final_rmse = last_iteration.rmse;
//! let final_config = last_iteration.config;
//! assert_eq!(final_config, config);
//! ```
use crate::{Config, Dimension, Lasish, Measurement, Variable};
use anyhow::{anyhow, Error};
use nalgebra::{DMatrix, DVector};

const DEFAULT_TOLERANCE: f64 = 1e-6;
const DEFAULT_LAS_SCAN_ANGLE: bool = false;
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

/// Adjust structure.
#[derive(Debug)]
pub struct Adjust<L: Lasish> {
    measurements: Vec<Measurement<L>>,
    rmse: f64,
    residuals: DVector<f64>,
    tolerance: f64,
    variables: Vec<Variable>,
    config: Config,
    history: Vec<Record>,
    las_scan_angle: bool,
}

/// A record of a single iteration.
#[derive(Clone, Debug)]
pub struct Record {
    pub rmse: f64,
    pub variables: Vec<Variable>,
    pub values: Vec<f64>,
    pub config: Config,
    pub las_scan_angle: bool,
}

impl<L: Lasish> Adjust<L> {
    /// Creates a new adjust for the provided measurements.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Adjust;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let adjust = Adjust::new(measurements).unwrap();
    /// ```
    pub fn new(measurements: Vec<Measurement<L>>) -> Result<Adjust<L>, Error> {
        Adjust::new_iteration(
            measurements,
            BORESIGHT_VARIABLES.to_vec(),
            vec![],
            DEFAULT_LAS_SCAN_ANGLE,
        )
    }

    /// Switch this adjust to adjust the lever arm.
    ///
    /// By default, adjusts boresight.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Adjust;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let mut adjust = Adjust::new(measurements).unwrap();
    /// adjust.adjust_lever_arm(true);
    /// ```
    pub fn adjust_lever_arm(&mut self, adjust_lever_arm: bool) {
        if adjust_lever_arm {
            self.variables = LEVER_ARM_VARIABLES.to_vec();
        } else {
            self.variables = BORESIGHT_VARIABLES.to_vec();
        }
    }

    fn new_iteration(
        measurements: Vec<Measurement<L>>,
        variables: Vec<Variable>,
        mut history: Vec<Record>,
        las_scan_angle: bool,
    ) -> Result<Adjust<L>, Error> {
        if measurements.is_empty() {
            return Err(anyhow!("cannot create adjust with no measurements"));
        }
        let config = measurements[0].config();
        let mut residuals = DVector::zeros(measurements.len() * 3);
        for (i, measurement) in measurements.iter().enumerate() {
            if measurement.config() != config {
                return Err(anyhow!("not all measurements have the same config"));
            }
            let rs = measurement.residuals(las_scan_angle);
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
            config,
            las_scan_angle,
        });
        Ok(Adjust {
            rmse,
            residuals,
            measurements,
            variables,
            tolerance: DEFAULT_TOLERANCE,
            history,
            config,
            las_scan_angle,
        })
    }

    /// Returns the root mean squared error for all the variables.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Adjust;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let adjust = Adjust::new(measurements).unwrap();
    /// let rmse = adjust.rmse();
    /// ```
    pub fn rmse(&self) -> f64 {
        self.rmse
    }

    /// Returns the configuration structure for this adjust.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Adjust;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let adjust = Adjust::new(measurements).unwrap();
    /// let config = adjust.config();
    /// ```
    pub fn config(&self) -> Config {
        self.config
    }

    /// Adjusts these measurements' configuration to optimally align the points.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Adjust;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let adjust = Adjust::new(measurements).unwrap();
    /// let adjust = adjust.adjust().unwrap();
    /// ```
    pub fn adjust(self) -> Result<Adjust<L>, Error> {
        let next = self.next()?;
        let delta = self.rmse - next.rmse;
        if delta < self.tolerance {
            Ok(self)
        } else {
            next.adjust()
        }
    }

    /// Returns this adjustment's history.
    ///
    /// Starts with one entry, the initial setup.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Adjust;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let adjust = Adjust::new(measurements).unwrap();
    /// assert_eq!(1, adjust.history().len());
    /// ```
    pub fn history(&self) -> &Vec<Record> {
        &self.history
    }

    fn next(&self) -> Result<Adjust<L>, Error> {
        let mut jacobian = DMatrix::zeros(self.residuals.len(), self.variables.len());
        for (i, measurement) in self.measurements.iter().enumerate() {
            for (j, dimension) in Dimension::iter().enumerate() {
                for (k, &variable) in self.variables.iter().enumerate() {
                    jacobian[(i * 3 + j, k)] = measurement.partial_derivative_in_body_frame(
                        dimension,
                        variable,
                        self.las_scan_angle,
                    );
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
        Adjust::new_iteration(
            measurements,
            self.variables.clone(),
            self.history.clone(),
            self.las_scan_angle,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_measurements() {
        assert!(Adjust::<las::Point>::new(vec![]).is_err());
    }

    #[test]
    fn different_config() {
        let mut measurements =
            crate::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let mut new_config = measurements[0].config();
        new_config.lever_arm.x = new_config.lever_arm.x + 1.;
        measurements[0] = measurements[0].with_config(new_config);
        assert!(Adjust::new(measurements).is_err());
    }

    #[test]
    fn adjust() {
        let measurements =
            crate::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
        let adjust = Adjust::new(measurements).unwrap().adjust().unwrap();
        assert!(adjust.rmse < 14.);
    }
}
