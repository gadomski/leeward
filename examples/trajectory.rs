//! Prints out a trajectory as a csv file.
//!
//! Useful for inspecting the quantization values.

use clap::{App, Arg};
use csv::Writer;
use leeward::Trajectory;
use serde::Serialize;
use std::io;

#[derive(Debug, Serialize)]
struct CsvPoint {
    latitude: f64,
    longitude: f64,
    altitude: f64,
    time: f64,
    scaled_time: i64,
}

fn main() {
    let matches = App::new("trajectory")
        .arg(Arg::with_name("sbet").help("sbet file").index(1))
        .get_matches();
    let trajectory = Trajectory::from_path(matches.value_of("sbet").unwrap()).unwrap();
    let mut writer = Writer::from_writer(io::stdout());
    for point in trajectory.points() {
        let csv_point = CsvPoint {
            latitude: point.latitude,
            longitude: point.longitude,
            altitude: point.altitude,
            time: point.time,
            scaled_time: trajectory.scaled_time(point.time),
        };
        writer.serialize(csv_point).unwrap();
    }
}
