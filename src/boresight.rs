use crate::{Config, Measurement, Variable};
use anyhow::{anyhow, Error};
use nalgebra::{DMatrix, DVector};
use std::io::{Sink, Write};

const DEFAULT_TOLERANCE: f64 = 1e-6;
const DEFAULT_USE_NUMERICAL_DIFFERENTIATION: bool = false;
const DEFAULT_VARIABLES: [Variable; 3] = [
    Variable::BoresightRoll,
    Variable::BoresightPitch,
    Variable::BoresightYaw,
];

/// A boresight alignment structure.
///
/// Iteratively refines the lidar parameters to determine the optimal system configuration.
#[derive(Debug)]
pub struct Boresight<W: Write> {
    config: Config,
    measurements: Vec<Measurement>,
    tolerance: f64,
    use_numerical_differentiation: bool,
    variables: Vec<Variable>,
    output: W,
}

/// The results of a boresight adjustment.
#[derive(Debug)]
pub struct Adjustment {
    pub config: Config,
    pub rmse: f64,
    pub residuals: DVector<f64>,
}

impl Boresight<Sink> {
    /// Creates a new boresight adjustment.
    ///
    /// The adjustment is initialized with boresight roll, pitch, and yaw as default adjustment variables.
    /// All output will be sent to /dev/null, to print output use `Boresight::with_output`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Boresight, Config};
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let boresight = Boresight::new(measurements, config);
    /// ```
    pub fn new(measurements: Vec<Measurement>, config: Config) -> Boresight<Sink> {
        Boresight {
            measurements,
            tolerance: DEFAULT_TOLERANCE,
            config,
            use_numerical_differentiation: DEFAULT_USE_NUMERICAL_DIFFERENTIATION,
            variables: DEFAULT_VARIABLES.to_vec(),
            output: std::io::sink(),
        }
    }
}

impl<W: Write> Boresight<W> {
    /// Creates a new boresight adjustment with the specified output.
    ///
    /// The adjustment is initialized with boresight roll, pitch, and yaw as default adjustment variables.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Boresight, Config};
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let boresight = Boresight::with_output(measurements, config, std::io::sink());
    /// ```
    pub fn with_output(measurements: Vec<Measurement>, config: Config, output: W) -> Boresight<W> {
        Boresight {
            measurements,
            tolerance: DEFAULT_TOLERANCE,
            config,
            use_numerical_differentiation: DEFAULT_USE_NUMERICAL_DIFFERENTIATION,
            variables: DEFAULT_VARIABLES.to_vec(),
            output,
        }
    }

    /// Sets whether this boresight uses numerical differentation.
    ///
    /// Defaults to false (analyticial partials).
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Boresight;
    /// let mut boresight = Boresight::new(vec![], Default::default());
    /// boresight.use_numerical_differentiation(true);
    /// ```
    pub fn use_numerical_differentiation(&mut self, use_numerical_differentiation: bool) {
        self.use_numerical_differentiation = use_numerical_differentiation;
    }

    /// Sets this boresight's variables.
    ///
    /// Use this to control which varaibles are modified during the boresight adjustment.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Boresight, Config, Variable};
    /// let mut boresight = Boresight::new(vec![], Config::default());
    /// boresight.set_variables(vec![Variable::BoresightRoll, Variable::BoresightPitch, Variable::BoresightYaw]);
    /// ```
    pub fn set_variables(&mut self, variables: Vec<Variable>) {
        self.variables = variables;
    }

    /// Runs this boresight adjustment.
    ///
    /// Modifies the internal configuration and measurements, in case you want to run multiple boresights with the same structure.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::{Boresight, Config};
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let config = Config::from_path("data/config.toml").unwrap();
    /// let mut boresight = Boresight::new(measurements, config);
    /// let adjustment = boresight.run();
    /// ```
    pub fn run(&mut self) -> Result<Adjustment, Error> {
        let mut adjustment = Adjustment::new(&self.measurements, self.config);
        let mut iteration = 0;
        loop {
            write!(self.output, "Iter #{}, rmse={}", iteration, adjustment.rmse)?;
            let jacobian = self.jacobian()?;
            let values = self.values()?;
            for (variable, value) in self.variables.iter().zip(&values) {
                write!(self.output, ", {}={}", variable, value)?;
            }
            let new_values = (jacobian.transpose() * &jacobian)
                .try_inverse()
                .ok_or_else(|| anyhow!("no inverse found"))?
                * jacobian.transpose()
                * (&jacobian * values - &adjustment.residuals);
            let new_config = self.update_config(&new_values)?;
            let new_measurements = self.update_measurements(&new_config)?;
            let new_adjustment = Adjustment::new(&new_measurements, new_config);
            if new_adjustment.rmse > adjustment.rmse {
                writeln!(self.output, ": new_rmse={}, done...", new_adjustment.rmse)?;
                return Ok(adjustment);
            } else if (adjustment.rmse - new_adjustment.rmse) / adjustment.rmse < self.tolerance {
                writeln!(self.output, ": new_rmse={}, done...", new_adjustment.rmse)?;
                return Ok(adjustment);
            } else {
                writeln!(self.output, "")?;
                self.measurements = new_measurements;
                self.config = new_config;
                adjustment = new_adjustment;
                iteration += 1;
            }
        }
    }

    fn jacobian(&self) -> Result<DMatrix<f64>, Error> {
        use crate::Dimension;
        let mut jacobian = DMatrix::zeros(self.measurements.len() * 3, self.variables.len());
        let dimensions = Dimension::all();
        assert_eq!(dimensions.len(), 3);
        for (i, measurement) in self.measurements.iter().enumerate() {
            for (j, &dimension) in dimensions.iter().enumerate() {
                let row = i * dimensions.len() + j;
                for (col, &variable) in self.variables.iter().enumerate() {
                    jacobian[(row, col)] = if self.use_numerical_differentiation {
                        measurement
                            .finite_difference((dimension, variable))
                            .ok_or_else(|| {
                                anyhow!(
                                    "could not use numerical differentiation for variable: {}",
                                    variable
                                )
                            })?
                    } else {
                        measurement.partial((dimension, variable))
                    };
                }
            }
        }
        Ok(jacobian)
    }

    fn values(&self) -> Result<DVector<f64>, Error> {
        let mut values = DVector::zeros(self.variables.len());
        for (i, variable) in self.variables.iter().enumerate() {
            values[i] = match *variable {
                Variable::BoresightRoll => self.config.boresight.roll,
                Variable::BoresightPitch => self.config.boresight.pitch,
                Variable::BoresightYaw => self.config.boresight.yaw,
                Variable::LeverArmX => self.config.lever_arm.x,
                Variable::LeverArmY => self.config.lever_arm.y,
                Variable::LeverArmZ => self.config.lever_arm.z,
                _ => {
                    return Err(anyhow!(
                        "unsupported boresight adjustment variable: {}",
                        variable
                    ))
                }
            };
        }
        Ok(values)
    }

    fn update_config(&self, values: &DVector<f64>) -> Result<Config, Error> {
        let mut new_config = self.config.clone();
        for (&variable, &value) in self.variables.iter().zip(values.iter()) {
            match variable {
                Variable::BoresightRoll => new_config.boresight.roll = value,
                Variable::BoresightPitch => new_config.boresight.pitch = value,
                Variable::BoresightYaw => new_config.boresight.yaw = value,
                Variable::LeverArmX => new_config.lever_arm.x = value,
                Variable::LeverArmY => new_config.lever_arm.y = value,
                Variable::LeverArmZ => new_config.lever_arm.z = value,
                _ => {
                    return Err(anyhow!(
                        "unsupported boresight adjustment variable: {}",
                        variable
                    ))
                }
            };
        }
        Ok(new_config)
    }

    fn update_measurements(&self, config: &Config) -> Result<Vec<Measurement>, Error> {
        let mut new_measurements = vec![];
        for measurement in &self.measurements {
            new_measurements.push(measurement.with_new_config(config)?);
        }
        Ok(new_measurements)
    }
}

impl Adjustment {
    fn new(measurements: &[Measurement], config: Config) -> Adjustment {
        let mut residuals = DVector::zeros(measurements.len() * 3);
        for (i, measurement) in measurements.iter().enumerate() {
            let misalignment = measurement.calculated() - measurement.las_point();
            for (j, &value) in misalignment.iter().enumerate() {
                let row = i * 3 + j;
                residuals[row] = value;
            }
        }
        Adjustment {
            config: config,
            rmse: residuals.norm(),
            residuals,
        }
    }
}
