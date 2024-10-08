use anyhow::Error;
use clap::{Parser, Subcommand};
use csv::Writer;
use leeward::{utils, Adjust, Config, Lasish, Measurement, Point};
use serde::Serialize;
use std::{fs::File, io::Write, path::PathBuf};

#[derive(Debug, Parser)]
struct Args {
    /// The SBET file
    sbet: PathBuf,

    /// The LAS file holding the points
    las: PathBuf,

    /// The config TOML file
    config: PathBuf,

    /// The amount to decimate the incoming points
    #[arg(short, long, default_value = "1")]
    decimation: usize,

    /// The output file.
    ///
    /// If not provided, the output will be printed to standard output.
    #[arg(short, long)]
    outfile: Option<PathBuf>,

    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Computes the boresight adjustment.
    Adjust {
        /// The file to write the history information.
        history: Option<PathBuf>,
    },

    /// Computes the best fit plane for the points in the body frame of the platform
    BestFitPlane,

    /// Computes the points in the body frame of the aircraft
    BodyFrame,

    /// Computes total propagated uncertainty
    Tpu,
}

fn main() -> Result<(), Error> {
    let args = Args::parse();
    let measurements =
        leeward::decimated_measurements(args.sbet, args.las, args.config, args.decimation)?;
    let mut write: Box<dyn Write> = if let Some(outfile) = args.outfile {
        Box::new(File::create(outfile)?)
    } else {
        Box::new(std::io::stdout())
    };
    match args.command {
        Command::Adjust { history } => {
            let adjust = Adjust::new(measurements)?.adjust()?;
            writeln!(write, "{}", toml::to_string_pretty(&adjust.config())?)?;
            if let Some(history) = history {
                let mut writer = File::create(history).map(Writer::from_writer)?;
                for (iteration, record) in adjust.history().iter().enumerate() {
                    writer.serialize(Record::new(iteration, record))?;
                }
            }
        }
        Command::BestFitPlane {} => {
            let mut writer = Writer::from_writer(write);
            for result in measurements.into_iter().map(|m| BodyFrame::new(&m)) {
                let body_frame = result?;
                writer.serialize(body_frame)?;
            }
        }
        Command::BodyFrame {} => {
            let mut writer = Writer::from_writer(write);
            for point in utils::fit_to_plane_in_body_frame(&measurements) {
                writer.serialize(point)?;
            }
        }
        Command::Tpu {} => {
            let mut writer = Writer::from_writer(write);
            for tpu in measurements.into_iter().flat_map(Tpu::new) {
                writer.serialize(tpu)?;
            }
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

#[derive(Debug, Serialize)]
struct Tpu {
    x: f64,
    y: f64,
    z: f64,
    range: f64,
    horizontal: f64,
    vertical: f64,
    total: f64,
    incidence_angle: f64,
}

impl BodyFrame {
    fn new<L: Lasish>(measurement: &Measurement<L>) -> Result<BodyFrame, Error> {
        let body_frame = measurement.body_frame();
        let modeled_body_frame_computed = measurement.modeled_body_frame();
        let scan_angle_computed = measurement.scan_angle();
        let mut measurement = measurement.clone();
        measurement.use_las_scan_angle(true);
        let modeled_body_frame_las = measurement.modeled_body_frame();
        let scan_angle_las = measurement.scan_angle();
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
            body_frame_config_computed_x: modeled_body_frame_computed.x,
            body_frame_config_computed_y: modeled_body_frame_computed.y,
            body_frame_config_computed_z: modeled_body_frame_computed.z,
            body_frame_config_las_x: modeled_body_frame_las.x,
            body_frame_config_las_y: modeled_body_frame_las.y,
            body_frame_config_las_z: modeled_body_frame_las.z,
            range: measurement.range(),
            scan_angle_computed,
            scan_angle_las,
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

impl Tpu {
    fn new<L: Lasish>(measurement: Measurement<L>) -> Result<Tpu, Error> {
        let tpu = measurement.tpu(Point::new(0., 0., 1.))?;
        Ok(Tpu {
            x: measurement.x(),
            y: measurement.y(),
            z: measurement.z(),
            range: measurement.range(),
            horizontal: tpu.horizontal,
            vertical: tpu.vertical,
            total: tpu.total,
            incidence_angle: tpu.incidence_angle,
        })
    }
}
