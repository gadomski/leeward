#[macro_use]
extern crate clap;

use anyhow::Error;
use leeward::app::{App, BackconvertOptions};
use leeward::Config;

fn main() -> Result<(), Error> {
    let yaml = load_yaml!("cli.yml");
    let matches = clap::App::from_yaml(yaml).get_matches();

    let mut app = App::new();
    app.quiet(matches.is_present("quiet"));

    if let Some(m) = matches.subcommand_matches("project-trajectory") {
        app.project_trajectory(
            m.value_of("TRAJECTORY").unwrap(),
            m.value_of("UTM_ZONE").unwrap().parse()?,
            m.value_of("OUTFILE").unwrap(),
            m.value_of("decimation").unwrap_or("1").parse()?,
        )?;
    } else if let Some(m) = matches.subcommand_matches("tpu") {
        app.tpu(
            m.value_of("TRAJECTORY").unwrap(),
            m.value_of("LASFILE").unwrap(),
            Config::from_path(m.value_of("CONFIG").unwrap())?,
            m.value_of("OUTFILE").unwrap(),
            m.value_of("decimation").unwrap_or("1").parse()?,
            m.is_present("all"),
        )?;
    } else if let Some(m) = matches.subcommand_matches("backconvert") {
        let backconverter = BackconvertOptions {
            original: m.is_present("original"),
            vector: m.is_present("vector"),
            gps_time: m.is_present("gps-time"),
            range: m.is_present("range"),
            scan_angle: m.is_present("scan-angle"),
            las_scan_angle: m.is_present("las-scan-angle"),
            orientation: m.is_present("orientation"),
            platform: m.is_present("platform"),
            partials: m.is_present("partials"),
        };
        app.backconvert(
            m.value_of("TRAJECTORY").unwrap(),
            m.value_of("LASFILE").unwrap(),
            Config::from_path(m.value_of("CONFIG").unwrap())?,
            backconverter,
            m.value_of("OUTFILE").unwrap(),
            m.value_of("decimation").unwrap_or("1").parse()?,
        )?;
    } else if let Some(m) = matches.subcommand_matches("boresight") {
        app.boresight(
            m.value_of("TRAJECTORY").unwrap(),
            m.value_of("LASFILE").unwrap(),
            Config::from_path(m.value_of("CONFIG").unwrap())?,
            m.value_of("decimation").unwrap_or("1").parse()?,
        )?;
    } else if let Some(m) = matches.subcommand_matches("measurement") {
        app.measurement(
            m.value_of("TRAJECTORY").unwrap(),
            m.value_of("LASFILE").unwrap(),
            Config::from_path(m.value_of("CONFIG").unwrap())?,
            m.value_of("INDEX").unwrap().parse()?,
        )?;
    } else if let Some(m) = matches.subcommand_matches("partials") {
        app.partials(
            m.value_of("TRAJECTORY").unwrap(),
            m.value_of("LASFILE").unwrap(),
            Config::from_path(m.value_of("CONFIG").unwrap())?,
            m.value_of("OUTFILE").unwrap(),
            m.value_of("decimation").unwrap_or("1").parse()?,
            m.value_of("delta").unwrap_or("0.01").parse()?,
        )?;
    }

    Ok(())
}
