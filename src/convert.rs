use anyhow::{anyhow, Error};
use nalgebra::Vector3;
use proj::Proj;

pub const WGS_84: Ellipsoid = Ellipsoid {
    a: 6378137.,
    _f: 1. / 298.257223563,
    b: 6356752.3142,
};

/// Used to convert coordinates between UTM, Lat/Lon, ECEF, and body frame.
pub struct GeodeticConverter {
    proj: Proj,
}

/// An ellipsoid definition.
pub struct Ellipsoid {
    a: f64,
    _f: f64,
    b: f64, // this is derived but that's ok
}

/// A projected point.
pub struct ProjectedPoint(Vector3<f64>);

/// A geodetic point.
pub struct GeodeticPoint {
    latitude: f64,
    longitude: f64,
    height: f64,
}

/// A geocentric point.
pub struct GeocentricPoint(Vector3<f64>);

impl GeodeticConverter {
    /// Creates a new converter to convert from the argument to WGS84 ellipsoidal.
    ///
    /// TODO should be able to handle other ellipsoids.
    ///
    /// # Examples
    ///
    ///  TODO
    pub fn new(utm_zone: u8) -> Result<GeodeticConverter, Error> {
        let from = format!("EPSG:326{:02}", utm_zone);
        Ok(GeodeticConverter {
            proj: Proj::new_known_crs(&from, "EPSG:4326", None)
                .ok_or(anyhow!("could not create proj from {} to {}"))?,
        })
    }

    /// Converts a point to geodetic.
    ///
    /// X and Y will be in radians.
    ///
    /// # Examples
    ///
    /// TODO
    pub fn convert<P: Into<ProjectedPoint>>(&self, point: P) -> Result<GeodeticPoint, Error> {
        let point = point.into();
        let converted = self.proj.convert((point.0.x, point.0.y))?;
        Ok(GeodeticPoint::new(
            converted.x().to_radians(),
            converted.y().to_radians(),
            point.0.z,
        ))
    }
}

impl GeodeticPoint {
    /// # Examples
    ///
    /// TODO
    pub fn new(longitude: f64, latitude: f64, height: f64) -> GeodeticPoint {
        GeodeticPoint {
            longitude,
            latitude,
            height,
        }
    }

    /// Converts this point from lat/lon to ECEF.
    ///
    /// Doesn't do any checks to make sure it's a lat lon, that's up to you.
    ///
    /// # Examples
    ///
    /// TODO
    pub fn to_ecef(&self, ellipsoid: Ellipsoid) -> GeocentricPoint {
        let n = ellipsoid.n(self.latitude);
        GeocentricPoint(Vector3::new(
            (n + self.height) * self.latitude.cos() * self.longitude.cos(),
            (n + self.height) * self.latitude.cos() * self.longitude.sin(),
            (ellipsoid.b2() / ellipsoid.a2() * n + self.height) * self.latitude.sin(),
        ))
    }
}

impl From<las::Point> for ProjectedPoint {
    fn from(las: las::Point) -> ProjectedPoint {
        ProjectedPoint(Vector3::new(las.x, las.y, las.z))
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

        let converter = GeodeticConverter::new(11).unwrap();
        let geodetic_point = converter.convert(point).unwrap();
        assert_eq!(geodetic_point.longitude.to_degrees(), -119.043462374326);
        assert_eq!(geodetic_point.latitude.to_degrees(), 37.76149775590434);
        assert_eq!(geodetic_point.height, 2687.59);

        let geocentric_point = geodetic_point.to_ecef(WGS_84);
        assert_relative_eq!(geocentric_point.0.x, -2452.031e3, max_relative = 1.0);
        assert_relative_eq!(geocentric_point.0.y, -4415.678e3, max_relative = 1.0);
        assert_relative_eq!(geocentric_point.0.z, 3886.195e3, max_relative = 1.0);
    }
}
