//! Write a csv of all of the partial derivatives for each point.

use clap::{App, Arg};
use csv::Writer;
use las::{Read, Reader};
use leeward::{Config, Partial, Trajectory};
use std::io;

fn main() {
    let matches = App::new("partials")
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
    let mut writer = Writer::from_writer(io::stdout());
    let mut header = vec!["X".to_string(), "Y".to_string(), "Z".to_string()];
    header.extend(Partial::all().into_iter().map(|p| p.to_string()));
    writer.write_record(header).unwrap();
    for result in reader.points().step_by(step_by) {
        let las = result.unwrap();
        let measurement = trajectory.measurement(&las, &config).unwrap();
        let las_point = measurement.las_point();
        let mut record = vec![las_point.x, las_point.y, las_point.z];
        record.extend(Partial::all().into_iter().map(|p| measurement.partial(p)));
        writer
            .write_record(
                record
                    .into_iter()
                    .map(|n| n.to_string())
                    .collect::<Vec<String>>(),
            )
            .unwrap();
    }
}
