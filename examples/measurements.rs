//! Prints measurements to stdout in csv format.

use clap::{App, Arg};
use csv::Writer;
use leeward::Measurement;
use serde::Serialize;
use std::io;

#[derive(Debug, Serialize)]
struct CsvMeasurement {}

impl From<&Measurement> for CsvMeasurement {
    fn from(_measurement: &Measurement) -> CsvMeasurement {
        CsvMeasurement {}
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
