use clap::{App, Arg};
use las::{Read, Reader};
use leeward::{Boresight, Config, Measurement, Trajectory};

fn main() {
    let matches = App::new("leeward-boresight")
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
    let measurements: Vec<Measurement> = Reader::from_path(matches.value_of("points").unwrap())
        .unwrap()
        .points()
        .step_by(step)
        .map(|r| Measurement::new(&trajectory, r.unwrap(), config).unwrap())
        .collect();
    let boresight = Boresight::new(measurements).unwrap();
    println!("{:?}", boresight.compute());
}
