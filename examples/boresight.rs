//! Computes a boresight adjustment for the provided data.

use clap::{App, Arg};
use las::{Read, Reader};
use leeward::{Boresight, Config, Trajectory, Variable};
use serde::Serialize;
use std::{fs::File, io::Write};

#[derive(Debug, Serialize)]
struct BoresightResult {
    rmse: f64,
    offset: f64,
    iterations: usize,
}

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
        .arg(
            Arg::with_name("offset")
                .help("time offset")
                .takes_value(true)
                .short("o")
                .long("offset")
                .allow_hyphen_values(true),
        )
        .arg(
            Arg::with_name("result")
                .help("result file to write")
                .takes_value(true)
                .short("r")
                .long("result"),
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

    let mut reader = Reader::from_path(matches.value_of("las").unwrap()).unwrap();
    let mut measurements = Vec::new();
    for result in reader.points().step_by(step_by) {
        let las = result.unwrap();
        match trajectory.measurement(&las, config) {
            Ok(measurement) => measurements.push(measurement),
            Err(err) => eprintln!("{}", err),
        }
    }
    let mut boresight = Boresight::with_output(measurements, config, std::io::stderr());
    let adjustment = boresight
        .run(
            &[
                Variable::BoresightRoll,
                Variable::BoresightPitch,
                Variable::BoresightYaw,
            ],
            false,
        )
        .unwrap();
    println!("{}", toml::to_string_pretty(&adjustment.config).unwrap());
    if let Some(result) = matches.value_of("result") {
        let mut file = File::create(result).unwrap();
        let result = BoresightResult {
            offset: matches.value_of("offset").unwrap_or("0").parse().unwrap(),
            iterations: adjustment.iterations,
            rmse: adjustment.rmse,
        };
        writeln!(file, "{}", toml::to_string_pretty(&result).unwrap()).unwrap();
    }
}
