use crate::*;

#[derive(Clone, Copy, Debug)]
pub struct LLA {
    /// radians
    pub latitude: f64,
    /// radians
    pub longitude: f64,
    /// meters above WGS-84 ellipsoid
    pub altitude: f32,
}

impl Default for LLA {
    fn default() -> Self {
        Self {
            latitude: Default::default(),
            longitude: Default::default(),
            altitude: Default::default(),
        }
    }
}

impl LLA {
    pub fn from_rads(latitude: f64, longitude: f64, altitude: f32) -> Self {
        Self { latitude, longitude, altitude }
    }
    pub fn from_degs(latitude: f64, longitude: f64, altitude: f32) -> Self {
        Self {
            latitude: latitude.to_radians(),
            longitude: longitude.to_radians(),
            altitude,
        }
    }

    pub fn to_ecef(&self) -> ECEF {
        //https://stackoverflow.com/questions/19478200/convert-latitude-and-longitude-to-ecef-coordinates-system
        //Direct implementation of https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
        let cos_lat = (self.latitude).cos();
        let sin_lat = (self.latitude).sin();
        let cos_lon = (self.longitude).cos();
        let sin_lon = (self.longitude).sin();

        let n = Earth::EQUATORIAL_RADIUS / (1. - Earth::ECCENTRICITY_SQUARED as f64 * sin_lat * sin_lat).sqrt();

        ECEF::new(
            (n + self.altitude as f64) * cos_lat * cos_lon,
            (n + self.altitude as f64) * cos_lat * sin_lon,
            (n * (1.0 - Earth::ECCENTRICITY_SQUARED) + self.altitude as f64) * sin_lat,
        )
    }

    pub fn to_tangent(&self, reference: &Reference) -> Vector3<f32> {
        reference.lla_to_tangent(*self)
    }

    pub fn from_tangent(reference: &Reference, tangent: Vector3<f32>) -> LLA {
        //1 - Projeter coordonnee local dans le repere de la terre
        let local_coor_in_earth = reference.tangent_to_ecef(tangent);

        //3 - Calculer les coordonnees local dans le repere ECEF Cartesien
        let local_coor_in_cart_ecef = local_coor_in_earth + reference.ecef;

        //4 - Convertir la position local ECEF cartesien dans ECEF LLA
        //Methode Borkowski
        //Ref : Borkowski K.M., (1987): Transformation of Geocentric to Geodetic Coordinates without Approximations,
        //      Astrophys. Space Sci., 139, pp. 1�4.
        //      Borkowski K.M., (1989): Accurate Algorithms to Transform Geocentric to Geodetic Coordinates, Bulletin
        //      Geodesique, Vol. 63, pp. 50�56.
        local_coor_in_cart_ecef.to_lla()
    }

    pub fn as_slice_rads(&self) -> [f64; 3] {
        [self.latitude, self.longitude, self.altitude as f64]
    }
}
