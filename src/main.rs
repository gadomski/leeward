use anyhow::Error;
use clap::{load_yaml, App, AppSettings};
use csv::Writer;
use leeward::Measurement;
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
    let measurements = leeward::measurements_with_decimation(
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
        unimplemented!()
    } else if let Some(_matches) = matches.subcommand_matches("body_frame") {
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
        unimplemented!()
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
    body_frame_config_x: f64,
    body_frame_config_y: f64,
    body_frame_config_z: f64,
}

impl BodyFrame {
    fn new(measurement: &Measurement) -> Result<BodyFrame, Error> {
        let body_frame = measurement.body_frame();
        let body_frame_config = measurement.body_frame_from_config();
        Ok(BodyFrame {
            time: measurement.time(),
            x: measurement.x(),
            y: measurement.x(),
            z: measurement.y(),
            roll: measurement.roll(),
            pitch: measurement.pitch(),
            yaw: measurement.yaw(),
            body_frame_x: body_frame.x,
            body_frame_y: body_frame.y,
            body_frame_z: body_frame.z,
            body_frame_config_x: body_frame_config.x,
            body_frame_config_y: body_frame_config.y,
            body_frame_config_z: body_frame_config.z,
        })
    }
}
