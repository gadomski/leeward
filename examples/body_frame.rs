//! Prints measurements in body frame to stdout.

use clap::{App, Arg};
use csv::Writer;
use las::{Read, Reader};
use leeward::{Config, Measurement, Trajectory};
use serde::Serialize;
use std::io;

#[derive(Debug, Serialize)]
struct CsvMeasurement {
    x: f64,
    y: f64,
    z: f64,
    scan_angle: f64,
    scaled_time: Option<i64>,
    gps_time: Option<f64>,
}

impl From<&Measurement> for CsvMeasurement {
    fn from(measurement: &Measurement) -> CsvMeasurement {
        let point = measurement.measured_point_in_body_frame();
        CsvMeasurement {
            x: point.x,
            y: point.y,
            z: point.z,
            scan_angle: measurement.scan_angle().unwrap(),
            scaled_time: None,
            gps_time: None,
        }
    }
}

fn main() {
    let matches = App::new("measurements")
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
        .arg(
            Arg::with_name("offset")
                .help("offset")
                .takes_value(true)
                .short("o")
                .long("offset")
                .allow_hyphen_values(true),
        )
        .arg(
            Arg::with_name("take")
                .help("take")
                .takes_value(true)
                .short("t")
                .long("take"),
        )
        .get_matches();
    let mut trajectory = Trajectory::from_path(matches.value_of("sbet").unwrap()).unwrap();
    if let Some(offset) = matches.value_of("offset") {
        let offset: f64 = offset.parse().unwrap();
        trajectory = trajectory.with_offset(offset).unwrap();
    }
    let config = Config::from_path(matches.value_of("config").unwrap()).unwrap();
    let step = matches
        .value_of("decimate")
        .unwrap_or_else(|| "1")
        .parse()
        .unwrap();
    let mut writer = Writer::from_writer(io::stdout());
    let mut reader = Reader::from_path(matches.value_of("las").unwrap()).unwrap();
    let iter = reader.points();
    let iter = if let Some(take) = matches.value_of("take") {
        Box::new(iter.take(take.parse().unwrap()))
            as Box<dyn Iterator<Item = Result<las::Point, las::Error>>>
    } else {
        Box::new(iter) as Box<dyn Iterator<Item = Result<las::Point, las::Error>>>
    };
    for result in iter.step_by(step) {
        let las = result.unwrap();
        match trajectory.measurement(&las, config) {
            Ok(measurement) => {
                let mut csv_measurement = CsvMeasurement::from(&measurement);
                csv_measurement.gps_time = Some(las.gps_time.unwrap());
                csv_measurement.scaled_time = Some(trajectory.scaled_time(las.gps_time.unwrap()));
                writer.serialize(csv_measurement).unwrap();
            }
            Err(err) => eprintln!("{}", err),
        }
    }
}
