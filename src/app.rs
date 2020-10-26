//! Command line application, used by main.rs.

use crate::partials::Partial;
use crate::{Config, Measurement, Trajectory};
use anyhow::{anyhow, Error};
use csv::Writer;
use indicatif::ProgressBar;
use las::Read;
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
#[allow(missing_docs)]
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
    /// let app = leeward::app::App::new();
    /// ```
    pub fn new() -> App {
        App::default()
    }

    /// If quiet, no progress reporting. Otherwise, report progress.
    ///
    /// # Examples
    ///
    /// ```
    /// let mut app = leeward::app::App::new();
    /// app.quiet(true);
    /// ```
    pub fn quiet(&mut self, quiet: bool) {
        self.quiet = quiet;
    }

    /// Projects a trajectory into UTM coordinates.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// let app = leeward::app::App::new();
    /// app.project_trajectory("examples/sbet.out", 11, "examples/sbet-projected.csv", 100).unwrap();
    /// ```
    pub fn project_trajectory<P0: AsRef<Path>, P1: AsRef<Path>>(
        &self,
        trajectory: P0,
        utm_zone: u8,
        outfile: P1,
        decimation: usize,
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory)?;
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

    /// Calculates TPU for a dataset and writes the results to a csv file.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// let app = leeward::app::App::new();
    /// let config = leeward::Config::from_path("examples/config.toml").unwrap();
    /// app.tpu("examples/sbet.out", "examples/one-point.las", config, "examples/tpu.csv", 100);
    /// ```
    pub fn tpu<P0: AsRef<Path>, P1: AsRef<Path>, P2: AsRef<Path>>(
        &self,
        trajectory: P0,
        lasfile: P1,
        config: Config,
        outfile: P2,
        decimation: usize,
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory)?;
        let mut reader = las::Reader::from_path(lasfile.as_ref())?;
        let progress =
            self.optional_progress_bar(reader.header().number_of_points() / decimation as u64);
        progress.set_message(&format!(
            "calculating tpu for {}",
            lasfile.as_ref().display()
        ));
        let mut outfile = File::create(outfile)?;
        writeln!(
            outfile,
            "X,Y,Z,sigmaX,sigmaY,sigmaHorizontal,sigmaVertical,sigmaMagnitude"
        )?;
        for result in reader.points().step_by(decimation) {
            let point = result?;
            let measurement = trajectory.measurement(point, config)?;
            let covariance = measurement.tpu();
            let point = measurement.las_point();
            writeln!(
                outfile,
                "{},{},{},{},{},{},{},{}",
                point.x,
                point.y,
                point.z,
                covariance[(0, 0)].sqrt(),
                covariance[(1, 1)].sqrt(),
                (covariance[(0, 0)] + covariance[(1, 1)]).sqrt(),
                covariance[(2, 2)].sqrt(),
                (covariance[(0, 0)] + covariance[(1, 1)] + covariance[(2, 2)]).sqrt(),
            )?;
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
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory)?;
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
        mut config: Config,
        decimation: usize,
    ) -> Result<(), Error> {
        use crate::partials::{Dimension, Variable};
        use nalgebra::{DVector, Dynamic, MatrixMN, Vector6, U6};

        let trajectory = self.read_trajectory(trajectory)?;
        let mut reader = las::Reader::from_path(lasfile)?;
        let mut measurements = Vec::new();
        for result in reader.points().step_by(decimation) {
            let point = result?;
            measurements.push(trajectory.measurement(point, config)?);
        }
        let variables = [
            Variable::BoresightRoll,
            Variable::BoresightPitch,
            Variable::BoresightYaw,
            Variable::LeverArmX,
            Variable::LeverArmY,
            Variable::LeverArmZ,
        ];
        let calculate_residuals = |measurements: &[Measurement]| -> DVector<f64> {
            use nalgebra::Vector3;
            let mut residuals = DVector::zeros(measurements.len() * 3);
            for (i, measurement) in measurements.iter().enumerate() {
                let misalignment = Vector3::from(measurement.misalignment());
                for (j, _) in Dimension::iter().enumerate() {
                    let row = i * 3 + j;
                    residuals[row] = misalignment[j];
                }
            }
            residuals
        };
        let calculate_jacobian = |measurements: &[Measurement]| -> MatrixMN<f64, Dynamic, U6> {
            let mut jacobian = MatrixMN::<f64, Dynamic, U6>::zeros(measurements.len() * 3);
            for (i, measurement) in measurements.iter().enumerate() {
                for (j, dimension) in Dimension::iter().enumerate() {
                    let row = i * 3 + j;
                    for (col, &variable) in variables.iter().enumerate() {
                        jacobian[(row, col)] = measurement.partial((dimension, variable));
                    }
                }
            }
            jacobian
        };
        let update_config = |config: &Config, values: Vector6<f64>| -> Config {
            let mut new_config = config.clone();
            new_config.boresight.roll = values[0];
            new_config.boresight.pitch = values[1];
            new_config.boresight.yaw = values[2];
            new_config.lever_arm.x = values[3];
            new_config.lever_arm.y = values[4];
            new_config.lever_arm.z = values[5];
            new_config
        };
        let update_measurements =
            |measurements: &[Measurement], config: Config| -> Vec<Measurement> {
                let mut new_measurements = Vec::new();
                for measurement in measurements {
                    new_measurements.push(measurement.with_config(config))
                }
                new_measurements
            };
        let mut iter = 0;
        let mut residuals = calculate_residuals(&measurements);
        let mut rmse = residuals.norm();
        println!(
            "Initial setup: rmse={}, roll={}, pitch={}, yaw={}, x={}, y={}, z={}",
            rmse,
            config.boresight.roll,
            config.boresight.pitch,
            config.boresight.yaw,
            config.lever_arm.x,
            config.lever_arm.y,
            config.lever_arm.z,
        );
        loop {
            let jacobian = calculate_jacobian(&measurements);
            let values = Vector6::new(
                config.boresight.roll,
                config.boresight.pitch,
                config.boresight.yaw,
                config.lever_arm.x,
                config.lever_arm.y,
                config.lever_arm.z,
            );
            let new_boresight = (jacobian.transpose() * &jacobian).try_inverse().unwrap()
                * jacobian.transpose()
                * (&jacobian * values - &residuals);
            let new_config = update_config(&config, new_boresight);
            let new_measurements = update_measurements(&measurements, new_config);
            let new_residuals = calculate_residuals(&new_measurements);
            let new_rmse = new_residuals.norm();
            if new_rmse < rmse {
                measurements = new_measurements;
                config = new_config;
                residuals = new_residuals;
                rmse = new_rmse;
                println!(
                    "Iter #{}, rmse reduced, updating config: rmse={}, roll={}, pitch={}, yaw={}, x={}, y={}, z={}",
                    iter,
                    rmse,
                    config.boresight.roll,
                    config.boresight.pitch,
                    config.boresight.yaw,
                    config.lever_arm.x,
                    config.lever_arm.y,
                    config.lever_arm.z,
                );
            } else {
                println!(
                    "Iter #{}, rmse increased, not updating config: rmse={}, bad_rmse={}, roll={}, pitch={}, yaw={}, x={}, y={}, z={}",
                    iter,
                    rmse,
                    new_rmse,
                    config.boresight.roll,
                    config.boresight.pitch,
                    config.boresight.yaw,
                    config.lever_arm.x,
                    config.lever_arm.y,
                    config.lever_arm.z
                );
                break;
            }
            iter += 1;
        }
        println!("{}", toml::to_string_pretty(&config)?);
        Ok(())
    }

    /// Prints a single measurement.
    pub fn measurement<P0: AsRef<Path>, P1: AsRef<Path>>(
        &self,
        trajectory: P0,
        lasfile: P1,
        config: Config,
        index: usize,
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory)?;
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

    /// Converts the data to the platform's coordinate system.
    pub fn platform<P0: AsRef<Path>, P1: AsRef<Path>, P2: AsRef<Path>>(
        &self,
        trajectory: P0,
        lasfile: P1,
        config: Config,
        outfile: P2,
        decimation: usize,
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory)?;
        let mut reader = las::Reader::from_path(lasfile.as_ref())?;
        let mut outfile = File::create(outfile)?;
        write!(
            outfile,
            "LasX,LasY,LasZ,LeewardX,LeewardY,LeewardZ,ScanAngle,Range"
        )?;
        for partial in Partial::iter() {
            write!(outfile, ",{}", partial)?;
        }
        writeln!(outfile, "")?;
        for result in reader.points().step_by(decimation) {
            let point = result?;
            let measurement = trajectory.measurement(point, config)?;
            let las = measurement.platform();
            let leeward = measurement.configured_point_in_platform();
            let measurement = measurement.to_platform();
            let scan_angle = measurement.scan_angle().to_degrees();
            let range = measurement.range();
            write!(
                outfile,
                "{},{},{},{},{},{},{},{}",
                las.x, las.y, las.z, leeward.x, leeward.y, leeward.z, scan_angle, range,
            )?;
            for partial in Partial::iter() {
                write!(outfile, ",{}", measurement.partial(partial))?;
            }
            writeln!(outfile, "")?;
        }
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
        delta: f64,
    ) -> Result<(), Error> {
        let trajectory = self.read_trajectory(trajectory)?;
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

    fn read_trajectory<P: AsRef<Path>>(&self, path: P) -> Result<Trajectory, Error> {
        let reader = sbet::Reader::from_path(path.as_ref())?;
        let mut vec = vec![];
        let progress = self.optional_progress_bar(sbet::estimate_number_of_points(path.as_ref())?);
        progress.set_message(&format!("reading sbet {}", path.as_ref().display()));
        for result in reader {
            let point = result?;
            vec.push(point);
            progress.inc(1);
        }
        progress.finish_with_message(&format!("done reading sbet {}", path.as_ref().display()));
        Ok(Trajectory::new(vec, None))
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
