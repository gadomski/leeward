//! Prints misalignment information to stdout in csv format.

use clap::{App, Arg};
use csv::Writer;
use leeward::Measurement;
use serde::Serialize;
use std::io;

#[derive(Debug, Serialize)]
struct CsvMeasurement {
    las_x: f64,
    las_y: f64,
    las_z: f64,
    misalignment_body_x: f64,
    misalignment_body_y: f64,
    misalignment_body_z: f64,
}

impl From<&Measurement> for CsvMeasurement {
    fn from(measurement: &Measurement) -> CsvMeasurement {
        let las = measurement.measured_point();
        let las_body = measurement.measured_point_in_body_frame();
        let calculated_body = measurement.calculated_point_in_body_frame().unwrap();
        let misalignment_body = calculated_body - las_body;
        CsvMeasurement {
            las_x: las.x,
            las_y: las.y,
            las_z: las.z,
            misalignment_body_x: misalignment_body.x,
            misalignment_body_y: misalignment_body.y,
            misalignment_body_z: misalignment_body.z,
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
        .get_matches();
    let measurements = leeward::measurements(
        matches.value_of("sbet").unwrap(),
        matches.value_of("las").unwrap(),
        matches.value_of("config").unwrap(),
    )
    .unwrap();
    let step_by = matches
        .value_of("decimate")
        .unwrap_or_else(|| "1")
        .parse()
        .unwrap();

    let mut writer = Writer::from_writer(io::stdout());
    for measurement in measurements.iter().step_by(step_by) {
        let measurement = CsvMeasurement::from(measurement);
        writer.serialize(measurement).unwrap();
    }
}
