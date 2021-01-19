use anyhow::Error;
use clap::{load_yaml, App, AppSettings};
use csv::Writer;
use leeward::{utils, Adjust, Config, Lasish, Measurement};
use serde::Serialize;
use std::{
    fs::File,
    io::{self, Write},
};

fn main() -> Result<(), Error> {
    let yaml = load_yaml!("cli.yml");
    let matches = App::from_yaml(yaml)
        .setting(AppSettings::DisableHelpSubcommand)
        .get_matches();
    let measurements = leeward::decimated_measurements(
        matches.value_of("SBET").unwrap(),
        matches.value_of("LAS").unwrap(),
        matches.value_of("CONFIG").unwrap(),
        matches
            .value_of("decimation")
            .map(|v| v.parse())
            .transpose()?
            .unwrap_or(1),
    )?;
    if let Some(_matches) = matches.subcommand_matches("adjust") {
        let adjust = Adjust::new(measurements)?.adjust()?;
        let mut write: Box<dyn Write> = if let Some(outfile) = matches.value_of("outfile") {
            Box::new(File::create(outfile)?)
        } else {
            Box::new(io::stdout())
        };
        writeln!(write, "{}", toml::to_string_pretty(&adjust.config())?)?;

        if let Some(history) = matches.value_of("history") {
            let mut writer = File::create(history).map(|f| Writer::from_writer(f))?;
            for (iteration, record) in adjust.history().iter().enumerate() {
                writer.serialize(Record::new(iteration, record))?;
            }
        }
    } else if let Some(matches) = matches.subcommand_matches("body_frame") {
        let write: Box<dyn Write> = if let Some(outfile) = matches.value_of("outfile") {
            Box::new(File::create(outfile)?)
        } else {
            Box::new(io::stdout())
        };
        let mut writer = Writer::from_writer(write);
        for result in measurements.into_iter().map(|m| BodyFrame::new(&m)) {
            let body_frame = result?;
            writer.serialize(body_frame)?;
        }
    } else if let Some(matches) = matches.subcommand_matches("best_fit_plane") {
        let write: Box<dyn Write> = if let Some(outfile) = matches.value_of("outfile") {
            Box::new(File::create(outfile)?)
        } else {
            Box::new(io::stdout())
        };
        let mut writer = Writer::from_writer(write);
        for point in utils::fit_to_plane_in_body_frame(&measurements) {
            writer.serialize(point)?;
        }
    }
    Ok(())
}

#[derive(Debug, Serialize)]
struct BodyFrame {
    time: f64,
    x: f64,
    y: f64,
    z: f64,
    roll: f64,
    pitch: f64,
    yaw: f64,
    body_frame_x: f64,
    body_frame_y: f64,
    body_frame_z: f64,
    body_frame_config_computed_x: f64,
    body_frame_config_computed_y: f64,
    body_frame_config_computed_z: f64,
    body_frame_config_las_x: f64,
    body_frame_config_las_y: f64,
    body_frame_config_las_z: f64,
    range: f64,
    scan_angle_computed: f64,
    scan_angle_las: f64,
}

#[derive(Debug, Serialize)]
struct Record {
    iteration: usize,
    rmse: f64,
    config: Config,
}

impl BodyFrame {
    fn new<L: Lasish>(measurement: &Measurement<L>) -> Result<BodyFrame, Error> {
        let body_frame = measurement.body_frame();
        let body_frame_config_computed = measurement.body_frame_from_config(false);
        let body_frame_config_las = measurement.body_frame_from_config(true);
        Ok(BodyFrame {
            time: measurement.time(),
            x: measurement.x(),
            y: measurement.y(),
            z: measurement.z(),
            roll: measurement.roll(),
            pitch: measurement.pitch(),
            yaw: measurement.yaw(),
            body_frame_x: body_frame.x,
            body_frame_y: body_frame.y,
            body_frame_z: body_frame.z,
            body_frame_config_computed_x: body_frame_config_computed.x,
            body_frame_config_computed_y: body_frame_config_computed.y,
            body_frame_config_computed_z: body_frame_config_computed.z,
            body_frame_config_las_x: body_frame_config_las.x,
            body_frame_config_las_y: body_frame_config_las.y,
            body_frame_config_las_z: body_frame_config_las.z,
            range: measurement.range(),
            scan_angle_computed: measurement.scan_angle(false),
            scan_angle_las: measurement.scan_angle(true),
        })
    }
}

impl Record {
    fn new(iteration: usize, record: &leeward::adjust::Record) -> Record {
        Record {
            iteration,
            rmse: record.rmse,
            config: record.config,
        }
    }
}
