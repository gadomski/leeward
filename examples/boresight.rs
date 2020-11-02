//! Computes a boresight adjustment for the provided data.

use anyhow::Error;
use clap::{App, Arg};
use las::{Read, Reader};
use leeward::{Config, Dimension, Measurement, Trajectory, Variable};
use nalgebra::{DMatrix, DVector};

fn main() {
    let matches = App::new("boresight")
        .arg(
            Arg::with_name("sbet")
                .help("sbet file")
                .required(true)
                .index(1),
        )
        .arg(
            Arg::with_name("las")
                .help("las file")
                .required(true)
                .index(2),
        )
        .arg(
            Arg::with_name("config")
                .help("config file")
                .required(true)
                .index(3),
        )
        .arg(
            Arg::with_name("decimate")
                .help("decimate")
                .takes_value(true)
                .short("d")
                .long("decimate"),
        )
        .get_matches();
    let trajectory = Trajectory::from_path(matches.value_of("sbet").unwrap()).unwrap();
    let mut config = Config::from_path(matches.value_of("config").unwrap()).unwrap();
    let step_by = matches
        .value_of("decimate")
        .unwrap_or_else(|| "1")
        .parse()
        .unwrap();

    let mut reader = Reader::from_path(matches.value_of("las").unwrap()).unwrap();
    let mut measurements = Vec::new();
    for result in reader.points().step_by(step_by) {
        let las = result.unwrap();
        measurements.push(trajectory.measurement(&las, &config).unwrap());
    }
    let variables = vec![
        Variable::BoresightRoll,
        Variable::BoresightPitch,
        Variable::BoresightYaw,
    ];

    let mut residuals = calculate_residuals(&measurements);
    let mut rmse = residuals.norm();
    let mut iter = 0;
    loop {
        print!("Iter #{}, rmse={}", iter, rmse);
        let jacobian = calculate_jacobian(&measurements, &variables);
        let values = extract_values(&config, &variables);
        for (variable, value) in variables.iter().zip(values.iter()) {
            print!(", {}={}", variable, value);
        }
        let new_values = (jacobian.transpose() * &jacobian).try_inverse().unwrap()
            * jacobian.transpose()
            * (&jacobian * values - &residuals);
        let new_config = update_config(&config, &variables, &new_values);
        let new_measurements = update_measurements(&measurements, &new_config).unwrap();
        let new_residuals = calculate_residuals(&new_measurements);
        let new_rmse = new_residuals.norm();
        if new_rmse > rmse {
            println!("\nrmse increased to {}, done", new_rmse);
            break;
        } else if (rmse - new_rmse) / rmse < 1e-6 {
            println!("\nrmse change less than {}, done", 1e-6);
            break;
        } else {
            residuals = new_residuals;
            rmse = new_rmse;
            measurements = new_measurements;
            config = new_config;
            iter += 1;
            println!("");
        }
    }
    println!("{}", toml::to_string_pretty(&config).unwrap());
}

fn calculate_jacobian(measurements: &[Measurement], variables: &[Variable]) -> DMatrix<f64> {
    let mut jacobian = DMatrix::zeros(measurements.len() * 3, variables.len());
    let dimensions = Dimension::all();
    for (i, measurement) in measurements.iter().enumerate() {
        for (j, &dimension) in dimensions.iter().enumerate() {
            let row = i * dimensions.len() + j;
            for (col, &variable) in variables.iter().enumerate() {
                jacobian[(row, col)] = measurement.partial((dimension, variable));
            }
        }
    }
    jacobian
}

fn calculate_residuals(measurements: &[Measurement]) -> DVector<f64> {
    let mut residuals = DVector::zeros(measurements.len() * 3);
    for (i, measurement) in measurements.iter().enumerate() {
        let misalignment = measurement.calculated() - measurement.las_point();
        for (j, &value) in misalignment.iter().enumerate() {
            let row = i * 3 + j;
            residuals[row] = value;
        }
    }
    residuals
}

fn extract_values(config: &Config, variables: &[Variable]) -> DVector<f64> {
    let mut values = DVector::zeros(variables.len());
    for (i, variable) in variables.iter().enumerate() {
        values[i] = match *variable {
            Variable::BoresightRoll => config.boresight.roll,
            Variable::BoresightPitch => config.boresight.pitch,
            Variable::BoresightYaw => config.boresight.yaw,
            Variable::LeverArmX => config.lever_arm.x,
            Variable::LeverArmY => config.lever_arm.y,
            Variable::LeverArmZ => config.lever_arm.z,
            _ => panic!("unsupported variable: {}", variable),
        };
    }
    values
}

fn update_config(config: &Config, variables: &[Variable], values: &DVector<f64>) -> Config {
    let mut new_config = config.clone();
    for (&variable, &value) in variables.iter().zip(values.iter()) {
        match variable {
            Variable::BoresightRoll => new_config.boresight.roll = value,
            Variable::BoresightPitch => new_config.boresight.pitch = value,
            Variable::BoresightYaw => new_config.boresight.yaw = value,
            Variable::LeverArmX => new_config.lever_arm.x = value,
            Variable::LeverArmY => new_config.lever_arm.y = value,
            Variable::LeverArmZ => new_config.lever_arm.z = value,
            _ => panic!("unsupported variable: {}", variable),
        };
    }
    new_config
}

fn update_measurements(
    measurements: &[Measurement],
    config: &Config,
) -> Result<Vec<Measurement>, Error> {
    let mut new_measurements = vec![];
    for measurement in measurements {
        new_measurements.push(measurement.with_new_config(config)?);
    }
    Ok(new_measurements)
}
