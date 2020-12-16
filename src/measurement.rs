use crate::Trajectory;
use anyhow::{anyhow, Error};

/// A measurement combines trajectory information with the lidar point.
#[derive(Debug)]
pub struct Measurement {
    las: las::Point,
    sbet: sbet::Point,
}

impl Measurement {
    /// Creates a new measurement.
    ///
    /// # Examples
    ///
    /// TODO
    pub fn new(trajectory: &Trajectory, las: las::Point) -> Result<Measurement, Error> {
        let gps_time = las.gps_time.ok_or(anyhow!("missing gps time on point"))?;
        let sbet = trajectory.get(gps_time).ok_or(anyhow!(
            "could not find sbet point for gps time: {}",
            gps_time
        ))?;
        Ok(Measurement { las, sbet: *sbet })
    }
}
