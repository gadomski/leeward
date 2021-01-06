//! Runs an adjustment

use clap::{App, Arg};
use las::{Read, Reader};
use leeward::{Adjustor, Config, Measurement, Trajectory};

fn main() {
    let matches = App::new("leeward-adjust")
        .arg(
            Arg::with_name("trajectory")
                .takes_value(true)
                .required(true)
                .help("The input sbet file"),
        )
        .arg(
            Arg::with_name("points")
                .takes_value(true)
                .required(true)
                .help("The input las file"),
        )
        .arg(
            Arg::with_name("config")
                .takes_value(true)
                .required(true)
                .help("The configuration file"),
        )
        .arg(
            Arg::with_name("decimate")
                .takes_value(true)
                .short("d")
                .long("decimate")
                .help("Decimate the input points by this amount before doing the adjustment"),
        )
        .arg(
            Arg::with_name("lever-arm")
                .short("l")
                .long("lever-arm")
                .help(
                "Pass this option to include the lever arm in the adjustment, takes much longer",
            ),
        )
        .arg(
            Arg::with_name("outfile")
                .takes_value(true)
                .long("outfile")
                .short("o")
                .help("The output config file, prints to stdout if not provided"),
        )
        .get_matches();
    let mut output: Box<dyn std::io::Write> = if let Some(outfile) = matches.value_of("outfile") {
        Box::new(std::fs::File::create(outfile).unwrap())
    } else {
        Box::new(std::io::stdout())
    };
    let trajectory = Trajectory::from_path(matches.value_of("trajectory").unwrap()).unwrap();
    let config = Config::from_path(matches.value_of("config").unwrap()).unwrap();
    let step = matches.value_of("decimate").unwrap_or("1").parse().unwrap();
    let boresight_only = !matches.is_present("lever-arm");
    let measurements = Reader::from_path(matches.value_of("points").unwrap())
        .unwrap()
        .points()
        .step_by(step)
        .map(|r| Measurement::new(&trajectory, r.unwrap(), config).unwrap())
        .collect();
    let mut adjustor = Adjustor::new(measurements).unwrap();
    let mut last_rmse = adjustor.rmse();
    let mut adjust_lever_arm = false;
    for i in 0.. {
        adjustor.adjust_lever_arm(adjust_lever_arm);
        adjustor = adjustor.adjust().unwrap();
        let new_rmse = adjustor.rmse();
        println!("Iter #{}: rmse={}", i, new_rmse);
        if boresight_only {
            break;
        } else if last_rmse - new_rmse < 1e-6 {
            break;
        } else {
            last_rmse = new_rmse;
            adjust_lever_arm = !adjust_lever_arm;
        }
    }
    adjustor.adjust_lever_arm(false);
    adjustor = adjustor.adjust().unwrap();

    writeln!(
        *output,
        "{}",
        toml::to_string_pretty(&adjustor.config()).unwrap()
    )
    .unwrap();
}
