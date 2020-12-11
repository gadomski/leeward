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
}

impl From<&Measurement> for CsvMeasurement {
    fn from(measurement: &Measurement) -> CsvMeasurement {
        let point = measurement.measured_point_in_body_frame();
        CsvMeasurement {
            x: point.x,
            y: point.y,
            z: point.z,
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
        .get_matches();
    let mut trajectory = Trajectory::from_path(matches.value_of("sbet").unwrap()).unwrap();
    if let Some(offset) = matches.value_of("offset") {
        let offset: f64 = offset.parse().unwrap();
        trajectory = trajectory.with_offset(offset).unwrap();
    }
    let config = Config::from_path(matches.value_of("config").unwrap()).unwrap();
    let step_by = matches
        .value_of("decimate")
        .unwrap_or_else(|| "1")
        .parse()
        .unwrap();
    let mut writer = Writer::from_writer(io::stdout());
    let mut reader = Reader::from_path(matches.value_of("las").unwrap()).unwrap();
    for result in reader.points().step_by(step_by) {
        let las = result.unwrap();
        match trajectory.measurement(&las, config) {
            Ok(measurement) => {
                let csv_measurement = CsvMeasurement::from(&measurement);
                writer.serialize(csv_measurement).unwrap();
            }
            Err(err) => eprintln!("{}", err),
        }
    }
}
