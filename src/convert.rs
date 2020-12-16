use anyhow::{anyhow, Error};
use proj::Proj;

/// Used to convert coordinates between UTM, Lat/Lon, ECEF, and body frame.
pub struct Converter {
    proj: Proj,
}

/// A 3D point.
pub struct Point {
    x: f64,
    y: f64,
    z: f64,
}

impl Converter {
    /// Creates a new converter to convert from the first argument to the second argument.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Converter;
    /// let converter = Converter::new("EPSG:32611", "EPSG:4326");
    /// ```
    pub fn new(from: &str, to: &str) -> Result<Converter, Error> {
        Ok(Converter {
            proj: Proj::new_known_crs(from, to, None)
                .ok_or(anyhow!("could not create proj from {} to {}"))?,
        })
    }

    /// Converts a point.
    ///
    /// # Examples
    ///
    /// TODO
    pub fn convert<P: Into<Point>>(&self, point: P) -> Result<Point, Error> {
        let point = point.into();
        let converted = self.proj.convert((point.x, point.y))?;
        Ok(Point::from_2d(converted, point.z))
    }
}

impl Point {
    /// Creates a point from a 2d geo_types point.
    ///
    /// # Examples
    ///
    /// TODO
    pub fn from_2d<P: Into<geo_types::Point<f64>>>(point: P, z: f64) -> Point {
        let point = point.into();
        Point {
            x: point.x(),
            y: point.y(),
            z: z,
        }
    }
}

impl From<las::Point> for Point {
    fn from(las: las::Point) -> Point {
        Point {
            x: las.x,
            y: las.y,
            z: las.z,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use las::{Read, Reader};

    #[test]
    fn las_point() {
        let point = Reader::from_path("data/points.las")
            .unwrap()
            .points()
            .next()
            .unwrap()
            .unwrap();
        assert_eq!(point.x, 320000.34);
        assert_eq!(point.y, 4181319.35);
        assert_eq!(point.z, 2687.59);

        let converter = Converter::new("EPSG:32611", "EPSG:4326").unwrap();
        let point = converter.convert(point).unwrap();
        assert_eq!(point.x, -119.043462374326);
        assert_eq!(point.y, 37.76149775590434);
    }
}
