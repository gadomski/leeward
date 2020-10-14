use crate::lidar::Measurement;
use crate::partials::{Dimension, Partial, Variable};
use crate::trajectory::Trajectory;
use crate::Config;
use anyhow::Error;
use csv::Writer;
use indicatif::ProgressBar;
use las::Read;
use nalgebra::{DMatrix, DVector};
use std::collections::HashMap;
use std::fs::File;
use std::io::Write;
use std::path::Path;

/// Global structure to manage total propogated error calculations.
///
/// Includes both the data and configuration for reading the data (e.g. whether progress should be reported).
#[derive(Debug)]
pub struct App {
    quiet: bool,
}

/// A collection of fields that can be written out from backconversion.
///
/// We might want to turn these on or off at the command line to test stuff.
#[derive(Debug)]
pub struct BackconvertOptions {
    pub original: bool,
    pub vector: bool,
    pub gps_time: bool,
    pub range: bool,
    pub scan_angle: bool,
    pub las_scan_angle: bool,
    pub orientation: bool,
    pub platform: bool,
    pub partials: bool,
}

struct OptionalProgressBar(Option<ProgressBar>);

impl App {
    /// Creates a new app.
    ///
    /// # Examples
    ///
    /// ```
    /// let app = leeward::App::new();
    /// ```
    pub fn new() -> App {
        App::default()
    }

    /// If quiet, no progress reporting. Otherwise, report progress.
    pub fn quiet(&mut self, quiet: bool) {
        self.quiet = quiet;
    }

    /// Projects a trajectory into UTM coordinates.
    pub fn project_trajectory<P0: AsRef<Path>, P1: AsRef<Path>>(
        &self,
        trajectory: P0,
        utm_zone: u8,
        outfile: P1,
        decimation: usize,
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory, None)?;
        let progress = self.optional_progress_bar((trajectory.len() / decimation) as u64);
        progress.set_message(&format!("writing csv {}", outfile.as_ref().display()));
        let mut file = File::create(&outfile)?;
        writeln!(file, "Northing,Easting,Elevation,Roll,Pitch,Yaw")?;
        for point in trajectory.into_iter().step_by(decimation) {
            let (northing, easting, _) =
                utm::radians_to_utm_wgs84(point.latitude, point.longitude, utm_zone);
            writeln!(
                file,
                "{},{},{},{},{},{}",
                northing,
                easting,
                point.altitude,
                point.roll.to_degrees(),
                point.pitch.to_degrees(),
                point.yaw.to_degrees()
            )?;
            progress.inc(1);
        }
        progress.finish_with_message(&format!("done writing {}", outfile.as_ref().display()));
        Ok(())
    }

    pub fn tpu<P0: AsRef<Path>, P1: AsRef<Path>, P2: AsRef<Path>>(
        &self,
        trajectory: P0,
        lasfile: P1,
        config: Config,
        outfile: P2,
        decimation: usize,
        quantization: u32,
        all: bool,
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory, Some(quantization))?;
        let mut reader = las::Reader::from_path(lasfile.as_ref())?;
        let progress =
            self.optional_progress_bar(reader.header().number_of_points() / decimation as u64);
        progress.set_message(&format!(
            "calculating tpu for {}",
            lasfile.as_ref().display()
        ));
        let mut outfile = File::create(outfile)?;
        write!(outfile, "X,Y,Z,sigmaX,sigmaY,sigmaHorizontal,sigmaVertical")?;
        if all {
            unimplemented!()
        }
        writeln!(outfile, "")?;
        for result in reader.points().step_by(decimation) {
            let point = result?;
            let measurement = trajectory.measurement(point, config)?;
            let covariance = measurement.tpu();
            let point = measurement.las_point();
            write!(
                outfile,
                "{},{},{},{},{},{},{}",
                point.x,
                point.y,
                point.z,
                covariance[(0, 0)].sqrt(),
                covariance[(1, 1)].sqrt(),
                (covariance[(0, 0)] + covariance[(1, 1)]).sqrt(),
                covariance[(2, 2)].sqrt(),
            )?;
            if all {
                unimplemented!()
            }
            writeln!(outfile, "")?;
            progress.inc(1);
        }
        progress.finish_with_message(&format!(
            "calculating tpu for {}",
            lasfile.as_ref().display(),
        ));
        Ok(())
    }

    /// Backconverts a las file using the lever arm and boresight.
    ///
    /// This can be a check to make sure the lidar equation is set up correctly.
    pub fn backconvert<P0: AsRef<Path>, P1: AsRef<Path>, P2: AsRef<Path>>(
        &self,
        trajectory: P0,
        lasfile: P1,
        config: Config,
        options: BackconvertOptions,
        outfile: P2,
        decimation: usize,
        quantization: u32,
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory, Some(quantization))?;
        let mut reader = las::Reader::from_path(lasfile.as_ref())?;
        let progress =
            self.optional_progress_bar(reader.header().number_of_points() / decimation as u64);
        progress.set_message(&format!(
            "backconverting {} to {}",
            lasfile.as_ref().display(),
            outfile.as_ref().display()
        ));
        let mut writer = Writer::from_path(outfile.as_ref())?;
        writer.write_record(options.header())?;
        for result in reader.points().step_by(decimation) {
            let point = result?;
            let measurement = trajectory.measurement(point, config)?;
            let record = options.record(&measurement)?;
            writer.write_record(record)?;
            progress.inc(1);
        }
        progress.finish_with_message(&format!(
            "done backconverting {} to {}",
            lasfile.as_ref().display(),
            outfile.as_ref().display()
        ));
        Ok(())
    }

    /// Calculates a boresight alignment.
    pub fn boresight<P0: AsRef<Path>, P1: AsRef<Path>>(
        &self,
        trajectory: P0,
        lasfile: P1,
        config: Config,
        decimation: usize,
        quantization: u32,
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory, Some(quantization))?;
        let mut reader = las::Reader::from_path(lasfile.as_ref())?;
        let progress =
            self.optional_progress_bar(reader.header().number_of_points() / decimation as u64);
        progress.set_message(&format!("reading {}", lasfile.as_ref().display(),));
        let mut measurements = vec![];
        for result in reader.points().step_by(decimation) {
            let point = result?;
            let measurement = trajectory.measurement(point, config)?;
            measurements.push(measurement);
            progress.inc(1);
        }
        progress.finish_with_message(&format!("done reading {}", lasfile.as_ref().display(),));

        let variables = [
            Variable::BoresightRoll,
            Variable::BoresightPitch,
            Variable::BoresightYaw,
        ];
        let last_error = std::f64::INFINITY;
        loop {
            let residuals = DVector::from_iterator(
                measurements.len() * 3,
                measurements
                    .iter()
                    .flat_map(|m| m.misalignment().into_iter()),
            );
            let error = residuals.norm();
            let relative_error = ((error - last_error) / error).abs();
            println!(
                "last_error={}, error={}, relative_error={}",
                last_error, error, relative_error,
            );
            if relative_error < 1e-6 {
                break;
            } else {
                let _last_error = error;
            }
            let mut jacobian = DMatrix::<f64>::zeros(measurements.len() * 3, variables.len());
            for (i, measurement) in measurements.iter().enumerate() {
                for (j, dimension) in Dimension::iter().enumerate() {
                    let row = i * 3 + j;
                    for (col, variable) in variables.iter().enumerate() {
                        jacobian[(row, col)] = measurement.partial(Partial(dimension, *variable));
                    }
                }
            }
            let adjustments = (jacobian.transpose() * jacobian.clone())
                .try_inverse()
                .ok_or_else(|| anyhow!("No inverse found"))?
                * jacobian.transpose()
                * residuals;
            let mut new_config = config.clone();
            for (&variable, &adjustment) in variables.iter().zip(adjustments.iter()) {
                new_config.adjust(variable, adjustment)?;
            }
            unimplemented!()
        }
        println!("last_error={}", last_error);

        Ok(())
    }

    /// Prints a single measurement.
    pub fn measurement<P0: AsRef<Path>, P1: AsRef<Path>>(
        &self,
        trajectory: P0,
        lasfile: P1,
        config: Config,
        index: usize,
        quantization: u32,
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory, Some(quantization))?;
        let mut reader = las::Reader::from_path(lasfile.as_ref())?;
        let point = reader
            .points()
            .nth(index)
            .ok_or_else(|| anyhow!("Not enough points for index: {}", index))??;
        let measurement = trajectory.measurement(point, config)?;
        println!("{:?}", measurement);
        println!("{:?}", measurement.gnss());
        Ok(())
    }

    /// Asses the partial derivatives.
    pub fn partials<P0: AsRef<Path>, P1: AsRef<Path>, P2: AsRef<Path>>(
        &self,
        trajectory: P0,
        lasfile: P1,
        config: Config,
        outfile: P2,
        decimation: usize,
        quantization: u32,
        delta: f64,
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory, Some(quantization))?;
        let mut reader = las::Reader::from_path(lasfile.as_ref())?;
        let mut outfile = File::create(outfile)?;
        write!(outfile, "X,Y,Z")?;
        for partial in Partial::iter() {
            write!(
                outfile,
                ",{} expected,{},{} adjustment,{} actual,{} error",
                partial, partial, partial, partial, partial
            )?;
        }
        writeln!(outfile, "")?;
        let progress =
            self.optional_progress_bar(reader.header().number_of_points() / decimation as u64);
        progress.set_message(&format!("reading {}", lasfile.as_ref().display(),));
        for result in reader.points().step_by(decimation) {
            let point = result?;
            let measurement = trajectory.measurement(point, config)?;
            let point = measurement.backconverted_point();
            write!(outfile, "{},{},{}", point.x, point.y, point.z)?;
            for partial in Partial::iter() {
                let check = measurement.partial_check(partial, delta);
                write!(
                    outfile,
                    ",{},{},{},{},{}",
                    check.expected, check.partial, check.adjustment, check.actual, check.error,
                )?;
            }
            writeln!(outfile, "")?;
            progress.inc(1);
        }
        progress.finish_with_message(&format!("done reading {}", lasfile.as_ref().display(),));
        Ok(())
    }

    fn read_trajectory<P: AsRef<Path>>(
        &self,
        path: P,
        quantization: Option<u32>,
    ) -> Result<Trajectory, Error> {
        let reader = sbet::Reader::from_path(path.as_ref())?;
        let mut vec = vec![];
        let mut hash_map = HashMap::new();
        let progress = self.optional_progress_bar(sbet::estimate_number_of_points(path.as_ref())?);
        progress.set_message(&format!("reading sbet {}", path.as_ref().display()));
        for result in reader {
            let point = result?;
            if let Some(quantization) = quantization {
                let time = quantize(point.time, quantization);
                hash_map.insert(time, point.clone());
            }
            vec.push(point);
            progress.inc(1);
        }
        progress.finish_with_message(&format!("done reading sbet {}", path.as_ref().display()));
        Ok(Trajectory {
            quantanization: quantization, // FIXME should be a constructor
            vec: vec,
            hash_map: hash_map,
        })
    }

    fn optional_progress_bar(&self, len: u64) -> OptionalProgressBar {
        OptionalProgressBar::new(!self.quiet, len)
    }
}

impl Default for App {
    fn default() -> App {
        App { quiet: false }
    }
}

impl OptionalProgressBar {
    fn new(enabled: bool, len: u64) -> OptionalProgressBar {
        let progress_bar = if enabled {
            use indicatif::ProgressStyle;
            let progress_bar = ProgressBar::new(len);
            progress_bar.set_style(
                ProgressStyle::default_bar()
                    .template("[{elapsed_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7} {msg}"),
            );
            progress_bar.set_draw_delta(len / 1000);
            Some(progress_bar)
        } else {
            None
        };
        OptionalProgressBar(progress_bar)
    }

    fn set_message(&self, msg: &str) {
        if let Some(progress_bar) = &self.0 {
            progress_bar.set_message(msg);
        }
    }

    fn inc(&self, delta: u64) {
        if let Some(progress_bar) = &self.0 {
            progress_bar.inc(delta);
        }
    }

    fn finish_with_message(&self, msg: &str) {
        if let Some(progress_bar) = &self.0 {
            progress_bar.finish_with_message(msg);
        }
    }
}

impl Config {
    /// Creates a configuration from TOML in a file path.
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Config, Error> {
        toml::from_str(&std::fs::read_to_string(path)?).map_err(Into::into)
    }
}

impl BackconvertOptions {
    fn header(&self) -> Vec<String> {
        let mut header = vec!["X".to_string(), "Y".to_string(), "Z".to_string()];
        if self.vector {
            header.extend(vec![
                "VectorX".to_string(),
                "VectorY".to_string(),
                "VectorZ".to_string(),
            ]);
        }
        if self.gps_time {
            header.push("GpsTime".to_string());
        }
        if self.range {
            header.push("Range".to_string());
        }
        if self.scan_angle {
            header.push("ScanAngle".to_string());
        }
        if self.las_scan_angle {
            header.push("LasScanAngle".to_string());
        }
        if self.orientation {
            header.extend(vec![
                "Roll".to_string(),
                "Pitch".to_string(),
                "Yaw".to_string(),
            ]);
        }
        if self.platform {
            header.extend(vec![
                "PlatformX".to_string(),
                "PlatformY".to_string(),
                "PlatformZ".to_string(),
            ]);
        }
        if self.partials {
            for partial in Partial::iter() {
                header.push(format!("{}", partial));
            }
        }
        header
    }

    fn record(&self, measurement: &Measurement) -> Result<Vec<String>, Error> {
        let mut record = vec![];
        let point = if self.original {
            measurement.las_point()
        } else {
            measurement.backconverted_point()
        };
        record.push(point.x.to_string());
        record.push(point.y.to_string());
        record.push(point.z.to_string());
        if self.gps_time {
            record.push(
                measurement
                    .gps_time()
                    .ok_or_else(|| anyhow!("No GPS time"))?
                    .to_string(),
            );
        }
        if self.range {
            record.push(measurement.range().to_string());
        }
        if self.scan_angle {
            record.push(measurement.scan_angle().to_degrees().to_string());
        }
        if self.las_scan_angle {
            record.push(measurement.las_scan_angle().to_degrees().to_string());
        }
        if self.orientation {
            let imu = measurement.imu();
            record.push(imu.roll.to_degrees().to_string());
            record.push(imu.pitch.to_degrees().to_string());
            record.push(imu.yaw.to_degrees().to_string());
        }
        if self.platform {
            let platform = measurement.platform();
            record.push(platform.x.to_string());
            record.push(platform.y.to_string());
            record.push(platform.z.to_string());
        }
        if self.partials {
            for partial in Partial::iter() {
                record.push(measurement.partial(partial).to_string());
            }
        }
        Ok(record)
    }
}

pub fn quantize(time: f64, level: u32) -> i64 {
    (time * f64::from(level)).round() as i64
}
