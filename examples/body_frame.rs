//! Converts the points in a las file into the body frame of the aircraft (ned).

use clap::{App, Arg};
use las::{Read, Reader};
use leeward::{Measurement, Trajectory};

fn main() {
    let matches = App::new("leeward: body frame")
        .arg(
            Arg::with_name("trajectory")
                .takes_value(true)
                .required(true),
        )
        .arg(Arg::with_name("points").takes_value(true).required(true))
        .get_matches();
    let trajectory = Trajectory::from_path(matches.value_of("trajectory").unwrap()).unwrap();
    for point in Reader::from_path(matches.value_of("points").unwrap())
        .unwrap()
        .points()
        .map(|r| r.unwrap())
    {
        let measurement = Measurement::new(&trajectory, point);
        println!("{:?}", measurement);
        break;
    }
}
