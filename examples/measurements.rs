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
    las_platform_x: f64,
    las_platform_y: f64,
    las_platform_z: f64,
    gnss_x: f64,
    gnss_y: f64,
    gnss_z: f64,
}

impl From<&Measurement> for CsvMeasurement {
    fn from(measurement: &Measurement) -> CsvMeasurement {
        let las = measurement.las_point();
        let las_platform = measurement.las_platform();
        let gnss = measurement.gnss_point();
        CsvMeasurement {
            las_x: las.x,
            las_y: las.y,
            las_z: las.z,
            las_platform_x: las_platform.x,
            las_platform_y: las_platform.y,
            las_platform_z: las_platform.z,
            gnss_x: gnss.x,
            gnss_y: gnss.y,
            gnss_z: gnss.z,
        }
    }
}

fn main() {
    let matches = App::new("leeward-measurements")
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
        .get_matches();
    let measurements = leeward::measurements(
        matches.value_of("sbet").unwrap(),
        matches.value_of("las").unwrap(),
        matches.value_of("config").unwrap(),
    )
    .unwrap();

    let mut writer = Writer::from_writer(io::stdout());
    for measurement in &measurements {
        let measurement = CsvMeasurement::from(measurement);
        writer.serialize(measurement).unwrap();
    }
}
