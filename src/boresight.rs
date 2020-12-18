use crate::{Dimension, Measurement, RollPitchYaw, Variable};
use anyhow::{anyhow, Error};
use nalgebra::{DVector, Dynamic, MatrixMN, Vector3, U3};

const VARIABLES: [Variable; 3] = [
    Variable::BoresightRoll,
    Variable::BoresightPitch,
    Variable::BoresightYaw,
];

/// A structure for computing boresight and lever arm adjustments from data.
#[derive(Debug)]
pub struct Boresight {
    measurements: Vec<Measurement>,
    residuals: DVector<f64>,
    rmse: f64,
}

/// A boresight adjustment.
#[derive(Debug)]
pub struct Adjustment {
    boresight: RollPitchYaw,
}

impl Boresight {
    /// Creates a new boresight adjuster.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Boresight;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let boresight = Boresight::new(measurements);
    /// ```
    pub fn new(measurements: Vec<Measurement>) -> Result<Boresight, Error> {
        if measurements.len() < 4 {
            Err(anyhow!("too few measurements: {}", measurements.len()))
        } else {
            let residuals = residuals(&measurements);
            Ok(Boresight {
                measurements,
                rmse: residuals.norm(),
                residuals,
            })
        }
    }

    /// Computes new boresight angles and lever arm displacements for these measurements.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Boresight;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let boresight = Boresight::new(measurements);
    /// let adjustment = boresight.compute();
    /// ```
    pub fn compute(&self) -> Result<Adjustment, Error> {
        println!("rmse={}", self.rmse);
        let jacobian = self.jacobian();
        let new_values = (jacobian.transpose() * &jacobian)
            .try_inverse()
            .ok_or(anyhow!("no inverse found"))?
            * jacobian.transpose()
            * (&jacobian * self.values() - self.residuals.clone());
        let boresight = self.boresight(new_values)?;
        let delta = self.rmse - boresight.rmse;
        if delta < 0.1 {
            Ok(Adjustment {
                boresight: self.measurements[0].config().boresight,
            })
        } else {
            boresight.compute()
        }
    }

    fn jacobian(&self) -> MatrixMN<f64, Dynamic, U3> {
        let mut jacobian = MatrixMN::<f64, Dynamic, U3>::zeros(self.measurements.len() * 3);
        for (i, measurement) in self.measurements.iter().enumerate() {
            for (j, dimension) in Dimension::iter().enumerate() {
                for (k, &variable) in VARIABLES.iter().enumerate() {
                    jacobian[(i * 3 + j, k)] =
                        measurement.partial_derivative_in_body_frame(dimension, variable);
                }
            }
        }
        jacobian
    }

    fn values(&self) -> Vector3<f64> {
        let measurement = &self.measurements[0];
        Vector3::new(
            measurement.boresight_roll(),
            measurement.boresight_pitch(),
            measurement.boresight_yaw(),
        )
    }

    fn boresight(&self, values: Vector3<f64>) -> Result<Boresight, Error> {
        let mut config = self.measurements[0].config();
        config.boresight.roll = values[0];
        config.boresight.pitch = values[1];
        config.boresight.yaw = values[2];
        let measurements = self
            .measurements
            .iter()
            .map(|m| m.with_config(config))
            .collect();
        Boresight::new(measurements)
    }
}

fn residuals(measurements: &[Measurement]) -> DVector<f64> {
    let mut residuals = DVector::zeros(measurements.len() * 3);
    for (i, measurement) in measurements.iter().enumerate() {
        let residual = measurement.body_frame_from_config() - measurement.body_frame();
        for (j, &n) in residual.iter().enumerate() {
            residuals[i * 2 + j] = n;
        }
    }
    residuals
}
