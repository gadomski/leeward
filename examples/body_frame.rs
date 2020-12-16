//! Converts the points in a las file into the body frame of the aircraft (ned).

use clap::{App, Arg};
use las::{Read, Reader};
use leeward::{Config, Measurement, Trajectory};

fn main() {
    let matches = App::new("leeward: body frame")
        .arg(
            Arg::with_name("trajectory")
                .takes_value(true)
                .required(true),
        )
        .arg(Arg::with_name("points").takes_value(true).required(true))
        .arg(Arg::with_name("config").takes_value(true).required(true))
        .get_matches();
    let trajectory = Trajectory::from_path(matches.value_of("trajectory").unwrap()).unwrap();
    let config = Config::from_path(matches.value_of("config").unwrap()).unwrap();
    println!("Time,X,Y,Z,BodyFrameX,BodyFrameY,BodyFrameZ");
    for point in Reader::from_path(matches.value_of("points").unwrap())
        .unwrap()
        .points()
        .map(|r| r.unwrap())
    {
        let measurement = Measurement::new(&trajectory, point, config).unwrap();
        let las = measurement.las();
        let body_frame = measurement.body_frame();
        println!(
            "{},{},{},{},{},{},{}",
            measurement.time(),
            las.x,
            las.y,
            las.z,
            body_frame.x,
            body_frame.y,
            body_frame.z
        );
    }
}
