use crate::{Dimension, Measurement, Point, RollPitchYaw, Variable};
use nalgebra::{DVector, Dynamic, MatrixMN, U6};

const VARIABLES: [Variable; 6] = [
    Variable::BoresightRoll,
    Variable::BoresightPitch,
    Variable::BoresightYaw,
    Variable::LeverArmX,
    Variable::LeverArmY,
    Variable::LeverArmZ,
];

/// A structure for computing boresight and lever arm adjustments from data.
#[derive(Debug)]
pub struct Boresight {
    measurements: Vec<Measurement>,
}

/// A boresight adjustment.
#[derive(Debug)]
pub struct Adjustment {
    boresight: RollPitchYaw,
    lever_arm: Point,
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
    pub fn new(measurements: Vec<Measurement>) -> Boresight {
        Boresight { measurements }
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
    pub fn compute(&self) -> Adjustment {
        let jacobian = self.jacobian();
        let residuals = self.residuals();
        unimplemented!()
    }

    fn jacobian(&self) -> MatrixMN<f64, Dynamic, U6> {
        let mut jacobian = MatrixMN::<f64, Dynamic, U6>::zeros(self.measurements.len() * 3);
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

    fn residuals(&self) -> DVector<f64> {
        let mut residuals = DVector::zeros(self.measurements.len() * 3);
        for (i, measurement) in self.measurements.iter().enumerate() {
            let residual = measurement.body_frame_from_config() - measurement.body_frame();
            for (j, &n) in residual.iter().enumerate() {
                residuals[i * 2 + j] = n;
            }
        }
        residuals
    }
}
