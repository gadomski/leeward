use crate::Measurement;

/// A structure for computing boresight and lever arm adjustments from data.
#[derive(Debug)]
pub struct Boresight {
    measurements: Vec<Measurement>,
}

/// A boresight adjustment.
#[derive(Debug)]
pub struct Adjustment {
    roll: f64,
    pitch: f64,
    yaw: f64,
    x: f64,
    y: f64,
    z: f64,
}

impl Boresight {
    /// Creates a new boresight adjuster.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Boresight;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let boresight = Boresight::new(measurements);
    /// ```
    pub fn new(measurements: Vec<Measurement>) -> Boresight {
        Boresight { measurements }
    }

    /// Computes new boresight angles and lever arm displacements for these measurements.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Boresight;
    /// let measurements = leeward::measurements("data/sbet.out", "data/points.las", "data/config.toml").unwrap();
    /// let boresight = Boresight::new(measurements);
    /// let adjustment = boresight.compute();
    /// ```
    pub fn compute(&self) -> Adjustment {
        unimplemented!()
    }
}
