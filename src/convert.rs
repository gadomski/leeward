use anyhow::{anyhow, Error};
use proj::Proj;

pub const WGS_84: Ellipsoid = Ellipsoid {
    a: 6378137.,
    _f: 1. / 298.257223563,
    b: 6356752.3142,
};

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

/// An ellipsoid definition.
pub struct Ellipsoid {
    a: f64,
    _f: f64,
    b: f64, // this is derived but that's ok
}

impl Converter {
    /// Creates a new converter to convert from the argument to WGS84 ellipsoidal.
    ///
    /// TODO should be able to handle other targets
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Converter;
    /// let converter = Converter::new("EPSG:32611");
    /// ```
    pub fn new(from: &str) -> Result<Converter, Error> {
        Ok(Converter {
            proj: Proj::new_known_crs(from, "EPSG:4326", None)
                .ok_or(anyhow!("could not create proj from {} to {}"))?,
        })
    }

    /// Converts a point.
    ///
    /// X and Y will be in radians.
    ///
    /// # Examples
    ///
    /// TODO
    pub fn convert<P: Into<Point>>(&self, point: P) -> Result<Point, Error> {
        let point = point.into();
        let converted = self.proj.convert((point.x, point.y))?;
        let mut point = Point::from_2d(converted, point.z);
        point.x = point.x.to_radians();
        point.y = point.y.to_radians();
        Ok(point)
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

    /// Converts this point from lat/lon to ECEF.
    ///
    /// Doesn't do any checks to make sure it's a lat lon, that's up to you.
    ///
    /// # Examples
    ///
    /// TODO
    pub fn to_ecef(&self, ellipsoid: Ellipsoid) -> Point {
        Point {
            x: (ellipsoid.n(self.y) + self.z) * self.y.cos() * self.x.cos(),
            y: (ellipsoid.n(self.y) + self.z) * self.y.cos() * self.x.sin(),
            z: (ellipsoid.b2() / ellipsoid.a2() * ellipsoid.n(self.y) + self.z) * self.y.sin(),
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

impl Ellipsoid {
    fn n(&self, latitude: f64) -> f64 {
        self.a2() / (self.a2() * latitude.cos().powi(2) + self.b2() * latitude.sin().powi(2)).sqrt()
    }

    fn a2(&self) -> f64 {
        self.a.powi(2)
    }

    fn b2(&self) -> f64 {
        self.b.powi(2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
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

        let converter = Converter::new("EPSG:32611").unwrap();
        let point = converter.convert(point).unwrap();
        assert_eq!(point.x.to_degrees(), -119.043462374326);
        assert_eq!(point.y.to_degrees(), 37.76149775590434);

        let point = point.to_ecef(WGS_84);
        assert_relative_eq!(point.x, -2452.031e3, max_relative = 1.0);
        assert_relative_eq!(point.y, -4415.678e3, max_relative = 1.0);
        assert_relative_eq!(point.z, 3886.195e3, max_relative = 1.0);
    }
}
