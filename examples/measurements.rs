//! Prints measurements to stdout in csv format.

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
    las_body_x: f64,
    las_body_y: f64,
    las_body_z: f64,
    gnss_x: f64,
    gnss_y: f64,
    gnss_z: f64,
    calculated_x: f64,
    calculated_y: f64,
    calculated_z: f64,
    calculated_body_x: f64,
    calculated_body_y: f64,
    calculated_body_z: f64,
    scanner_x: f64,
    scanner_y: f64,
    scanner_z: f64,
    range: f64,
    scan_angle: f64,
    misalignment_x: f64,
    misalignment_y: f64,
    misalignment_z: f64,
    misalignment_body_x: f64,
    misalignment_body_y: f64,
    misalignment_body_z: f64,
}

impl From<&Measurement> for CsvMeasurement {
    fn from(measurement: &Measurement) -> CsvMeasurement {
        let las = measurement.measured_point();
        let las_body = measurement.measured_point_in_body_frame();
        let gnss = measurement.gnss();
        let calculated = measurement.calculated_point().unwrap();
        let calculated_body = measurement.calculated_point_in_body_frame();
        let scanner = measurement.scanner_point().unwrap();
        let range = measurement.range();
        let scan_angle = measurement.scan_angle().unwrap().to_degrees();
        let misalignment = calculated - las;
        let misalignment_body = calculated_body - las_body;
        CsvMeasurement {
            las_x: las.x,
            las_y: las.y,
            las_z: las.z,
            las_body_x: las_body.x,
            las_body_y: las_body.y,
            las_body_z: las_body.z,
            gnss_x: gnss.x,
            gnss_y: gnss.y,
            gnss_z: gnss.z,
            calculated_x: calculated.x,
            calculated_y: calculated.y,
            calculated_z: calculated.z,
            calculated_body_x: calculated_body.x,
            calculated_body_y: calculated_body.y,
            calculated_body_z: calculated_body.z,
            scanner_x: scanner.x,
            scanner_y: scanner.y,
            scanner_z: scanner.z,
            range,
            scan_angle,
            misalignment_x: misalignment.x,
            misalignment_y: misalignment.y,
            misalignment_z: misalignment.z,
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
