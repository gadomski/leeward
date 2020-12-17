use crate::Point;

pub const WGS_84: Ellipsoid = Ellipsoid {
    a: 6378137.,
    a2: 6378137. * 6378137.,
    f: 1. / 298.257223563,
    b: 6356752.3142,
    b2: 6356752.3142 * 6356752.3142,
};

/// Converts from projected (UTM) coordinates into geodetic coordinates.
///
/// The geodetic coordinates are in radians.
///
/// # Examples
///
/// ```
/// use leeward::convert;
/// let projected = Point::new(320000.34, 4181319.35, 2687.59);
/// let geodetic = convert::projected_to_geodetic(projected, 11); // 11 is the UTM zone
/// ```
pub fn projected_to_geodetic(point: Point, utm_zone: u8) -> Point {
    let ellipsoid = WGS_84;
    let n = ellipsoid.f / (2. - ellipsoid.f);
    let a = ellipsoid.a / (1. + n) * (1. + n.powi(2) / 4. + n.powi(4) / 64.);
    let k0 = 0.9996;
    let xi = point.y / (k0 * a);
    let nu = (point.x - 500e3) / (k0 * a);
    let b1 = 0.5 * n - (2. / 3.) * n.powi(2) - (37. / 96.) * n.powi(3);
    let b2 = (1. / 48.) * n.powi(2) - (1. / 15.) * n.powi(3);
    let b3 = (17. / 480.) * n.powi(3);
    let d1 = 2. * n - (2. / 3.) * n.powi(2) - 2. * n.powi(3);
    let d2 = (7. / 3.) * n.powi(2) - (8. / 5.) * n.powi(3);
    let d3 = (56. / 15.) * n.powi(3);
    let xi_prime = xi
        - (b1 * (2. * xi).sin() * (2. * nu).cosh()
            + b2 * (4. * xi).sin() * (4. * nu).cosh()
            + b3 * (6. * xi).sin() * (6. * nu).cosh());
    let nu_prime = nu
        - (b1 * (2. * xi).cos() * (2. * nu).sinh()
            + b2 * (4. * xi).cos() * (4. * nu).sinh()
            + b3 * (6. * xi).cos() * (6. * nu).sinh());
    let chi = (xi_prime.sin() / nu_prime.cosh()).asin();
    let latitude = chi + d1 * (2. * chi).sin() + d2 * (4. * chi).sin() + d3 * (6. * chi).sin();
    let reference_meridian = f64::from(utm_zone) * 6f64.to_radians() - 183f64.to_radians();
    let longitude = reference_meridian + (nu_prime.sinh() / xi_prime.cos()).atan();
    Point::new(longitude, latitude, point.z)
}

/// Converts a geodetic point to ECEF.
///
/// Uses the WGS84 ellipsoid.
///
/// # Examples
///
/// ```
/// use leeward::convert;
/// let geodetic = Point::new(-119.0434.to_radians(), 37.7614978.to_radians(), 2687.59);
/// let geocentric = convert::geodetic_to_ecef(geodetic);
/// ```
pub fn geodetic_to_ecef(point: Point) -> Point {
    unimplemented!()
}

/// An ellipsoid.
///
/// Some of the fields are derived, but required to minimise computations when using the ellipsoid.
#[derive(Clone, Copy, Debug)]
pub struct Ellipsoid {
    a: f64,
    a2: f64,
    f: f64,
    b: f64,
    b2: f64,
}

impl Ellipsoid {
    fn n(&self, latitude: f64) -> f64 {
        self.a2 / (self.a2 * latitude.cos().powi(2) + self.b2 * latitude.sin().powi(2)).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use crate::Point;
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

        let geodetic = super::projected_to_geodetic(Point::new(point.x, point.y, point.z), 11);
        assert_relative_eq!(
            geodetic.x.to_degrees(),
            -119.043462374326,
            max_relative = 0.00000001
        );
        assert_relative_eq!(
            geodetic.y.to_degrees(),
            37.76149775590434,
            max_relative = 0.00000001
        );
        assert_eq!(geodetic.z, 2687.59);

        let geocentric = super::geodetic_to_ecef(geodetic);
        assert_relative_eq!(geocentric.x, -2452.031e3, max_relative = 1.0);
        assert_relative_eq!(geocentric.y, -4415.678e3, max_relative = 1.0);
        assert_relative_eq!(geocentric.z, 3886.195e3, max_relative = 1.0);
    }
}
