//! Runs an adjustment

use clap::{App, Arg};
use las::{Read, Reader};
use leeward::{Adjustor, Config, Measurement, Trajectory};

fn main() {
    let matches = App::new("leeward-adjust")
        .arg(
            Arg::with_name("trajectory")
                .takes_value(true)
                .required(true),
        )
        .arg(Arg::with_name("points").takes_value(true).required(true))
        .arg(Arg::with_name("config").takes_value(true).required(true))
        .arg(
            Arg::with_name("decimate")
                .takes_value(true)
                .short("d")
                .long("decimate"),
        )
        .get_matches();
    let trajectory = Trajectory::from_path(matches.value_of("trajectory").unwrap()).unwrap();
    let config = Config::from_path(matches.value_of("config").unwrap()).unwrap();
    let step = matches.value_of("decimate").unwrap_or("1").parse().unwrap();
    let measurements = Reader::from_path(matches.value_of("points").unwrap())
        .unwrap()
        .points()
        .step_by(step)
        .map(|r| Measurement::new(&trajectory, r.unwrap(), config).unwrap())
        .collect();
    let adjustor = Adjustor::new(measurements).unwrap();
    let adjustment = adjustor.adjust().unwrap();
    for (i, record) in adjustment.history.iter().enumerate() {
        eprint!("Iter #{}: rmse={}", i, record.rmse);
        for (variable, value) in record.variables.iter().zip(&record.values) {
            eprint!(", {:?}={}", variable, value);
        }
        eprintln!("");
    }
    println!("{}", toml::to_string_pretty(&adjustment.config).unwrap());
}
