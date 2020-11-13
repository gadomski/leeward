use clap::{App, Arg};
use las::{Point, Read, Reader};
use leeward::{Config, Measurement, Platform};
use nalgebra::{DMatrix, DVector, Vector6};
use std::collections::HashMap;

fn main() {
    let matches = App::new("leeward")
        .arg(
            Arg::with_name("las")
                .help("las file")
                .required(true)
                .index(1),
        )
        .arg(
            Arg::with_name("config")
                .help("config file")
                .required(true)
                .index(2),
        )
        .arg(
            Arg::with_name("scale")
                .short("s")
                .long("scale")
                .help("time scale factor")
                .takes_value(true),
        )
        .get_matches();
    let mut reader = Reader::from_path(matches.value_of("las").unwrap()).unwrap();
    let mut config = Config::from_path(matches.value_of("config").unwrap()).unwrap();
    config.derive_scan_angle = false;
    let scale: f64 = matches.value_of("scale").unwrap_or("0.02").parse().unwrap();
    let mut buckets = HashMap::new();
    for point in reader.points() {
        let point = point.unwrap();
        let time = scaled_time(point.gps_time.unwrap(), scale);
        buckets.entry(time).or_insert_with(|| vec![]).push(point);
    }
    println!("Time,X,Y,Z,Roll,Pitch,Yaw");
    for (&time, points) in &buckets {
        let platform = locate_platform(points, config);
        let time = time as f64 * scale;
        println!(
            "{},{},{},{},{},{},{}",
            time, platform.x, platform.y, platform.z, platform.roll, platform.pitch, platform.yaw
        );
    }
}

fn scaled_time(time: f64, scale: f64) -> i64 {
    (time / scale).round() as i64
}

fn locate_platform(points: &[Point], config: Config) -> Platform {
    let mut x = 0.;
    let mut y = 0.;
    let mut z = 0.;
    for point in points {
        x += point.x;
        y += point.y;
        z += point.z;
    }
    x = x / points.len() as f64;
    y = y / points.len() as f64;
    z = z / points.len() as f64;
    let mut platform = Platform {
        x,
        y,
        z,
        roll: 0.,
        pitch: 0.,
        yaw: 0.,
    };
    let (mut measurements, mut residuals) = make_measurements(points, platform, config);
    loop {
        let jacobian = jacobian(&measurements);
        let values = Vector6::new(
            platform.x,
            platform.y,
            platform.z,
            platform.roll,
            platform.pitch,
            platform.yaw,
        );
        let new_values = (jacobian.transpose() * &jacobian).try_inverse().unwrap()
            * jacobian.transpose()
            * (&jacobian * values - &residuals);
        let new_platform = Platform {
            x: new_values[0],
            y: new_values[1],
            z: new_values[2],
            roll: new_values[3],
            pitch: new_values[4],
            yaw: new_values[5],
        };
        let (new_measurements, new_residuals) = make_measurements(points, new_platform, config);
        if new_residuals.norm() > residuals.norm() {
            break;
        } else {
            measurements = new_measurements;
            residuals = new_residuals;
            platform = new_platform;
        }
    }
    platform
}

fn make_measurements(
    points: &[Point],
    location: Platform,
    config: Config,
) -> (Vec<Measurement>, DVector<f64>) {
    let mut measurements = vec![];
    let mut residuals = DVector::zeros(points.len() * 3);
    for (i, point) in points.iter().enumerate() {
        let measurement = Measurement::new(point, location, config);
        let misalignment = measurement.calculated_point().unwrap() - measurement.measured_point();
        measurements.push(measurement);
        residuals[3 * i] = misalignment.x;
        residuals[3 * i + 1] = misalignment.y;
        residuals[3 * i + 2] = misalignment.z;
    }
    (measurements, residuals)
}

fn jacobian(measurements: &[Measurement]) -> DMatrix<f64> {
    use leeward::{Dimension, Variable};
    let variables = [
        Variable::GnssX,
        Variable::GnssY,
        Variable::GnssZ,
        Variable::ImuRoll,
        Variable::ImuPitch,
        Variable::ImuYaw,
    ];

    let mut jacobian = DMatrix::zeros(measurements.len() * 3, 6);
    for (i, measurement) in measurements.iter().enumerate() {
        for (j, dimension) in Dimension::all().into_iter().enumerate() {
            let row = 3 * i + j;
            for (col, &variable) in variables.iter().enumerate() {
                jacobian[(row, col)] = measurement.partial((dimension, variable)).unwrap();
            }
        }
    }
    jacobian
}
