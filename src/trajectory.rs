use crate::lidar::Measurement;
use crate::Config;
use anyhow::{anyhow, Error};
use std::collections::HashMap;
use std::path::Path;

#[derive(Debug)]
pub struct Trajectory {
    pub quantanization: Option<u32>,
    pub hash_map: HashMap<i64, sbet::Point>,
    pub vec: Vec<sbet::Point>,
}

impl Trajectory {
    pub fn new<P: AsRef<Path>>(path: P, quantization: Option<u32>) -> Result<Trajectory, Error> {
        let reader = sbet::Reader::from_path(path.as_ref())?;
        let mut vec = vec![];
        let mut hash_map = HashMap::new();
        for result in reader {
            let point = result?;
            if let Some(quantization) = quantization {
                let time = quantize(point.time, quantization);
                hash_map.insert(time, point.clone());
            }
            vec.push(point);
        }
        Ok(Trajectory {
            quantanization: quantization,
            vec: vec,
            hash_map: hash_map,
        })
    }

    pub fn into_iter(self) -> std::vec::IntoIter<sbet::Point> {
        self.vec.into_iter()
    }

    pub fn len(&self) -> usize {
        self.vec.len()
    }

    pub fn measurement(&self, las: las::Point, config: Config) -> Result<Measurement, Error> {
        let gps_time = las
            .gps_time
            .ok_or_else(|| anyhow!("Missing GPSTime field on las point"))?;
        let quantanization = self
            .quantanization
            .ok_or_else(|| anyhow!("Trajectory is not quantized"))?;
        let gps_time_quantized = crate::trajectory::quantize(gps_time, quantanization); // FIXME hack
        let sbet = self
            .hash_map
            .get(&gps_time_quantized)
            .ok_or_else(|| anyhow!("Could not find sbet point for gps time: {}", gps_time))?;
        Ok(Measurement::new(las, *sbet, config))
    }
}

fn quantize(time: f64, level: u32) -> i64 {
    (time * f64::from(level)).round() as i64
}
