//! Computes a boresight adjustment for the provided data.

use clap::{App, Arg};
use las::{Read, Reader};
use leeward::{Config, Trajectory};

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
        measurements.push(trajectory.measurement(&las, &config).unwrap());
    }
}
