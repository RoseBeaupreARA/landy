use core::ops::{Add, Div, Mul, Neg, Sub};

use crate::prelude::*;

#[derive(Clone, Copy, Debug)]
pub struct ECEF {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl ECEF {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn to_lla(&self) -> LLA {
        //Methode Borkowski
        //Ref : Borkowski K.M., (1987): Transformation of Geocentric to Geodetic Coordinates without Approximations,
        //      Astrophys. Space Sci., 139, pp. 1�4.
        //      Borkowski K.M., (1989): Accurate Algorithms to Transform Geocentric to Geodetic Coordinates, Bulletin
        //      Geodesique, Vol. 63, pp. 50�56.
        let beta = (self.x.powi(2) + self.y.powi(2)).sqrt();

        const SQRT_1_MINUS_ECCENTRICITY_SQUARED: f64 = 0.99664718933525251928015447138147944404426736382199023615; // = sqrt(1.0 - Earth::ECCENTRICITY_SQUARED)

        let sqrt1_minus_eccentricity_squared_times_abs_z = SQRT_1_MINUS_ECCENTRICITY_SQUARED * self.z.abs();
        const ECCENTRICITY_SQUARED_TIMES_EQUATORIAL_RADIUS: f64 = Earth::ECCENTRICITY_SQUARED * Earth::EQUATORIAL_RADIUS;

        let one_over_beta = 1.0 / beta;
        let ei = (sqrt1_minus_eccentricity_squared_times_abs_z - ECCENTRICITY_SQUARED_TIMES_EQUATORIAL_RADIUS) * one_over_beta;
        let fi = (sqrt1_minus_eccentricity_squared_times_abs_z + ECCENTRICITY_SQUARED_TIMES_EQUATORIAL_RADIUS) * one_over_beta;
        let ei_squared = ei * ei;
        let pi = (4.0 / 3.0) * (ei * fi + 1.0);
        let qi = 2.0 * (ei_squared - (fi * fi));
        let di = (pi * pi * pi) + (qi * qi);
        let sqrt_di = (di).sqrt();
        let vi = (sqrt_di - qi).powf(1.0 / 3.0) - (sqrt_di + qi).powf(1.0 / 3.0);
        let gi = 0.5 * ((ei_squared + vi).sqrt() + ei);
        let ti = ((gi * gi) + (fi - vi * gi) / (2.0 * gi - ei)).sqrt() - gi;

        let longitude = self.y.atan2(self.x);
        let latitude = self.z.signum() * ((1. - (ti * ti)) / (ti * SQRT_1_MINUS_ECCENTRICITY_SQUARED * 2.0)).atan();
        let altitude = (beta - Earth::EQUATORIAL_RADIUS * ti) * (latitude).cos()
            + (self.z - self.z.signum() * Earth::EQUATORIAL_RADIUS * SQRT_1_MINUS_ECCENTRICITY_SQUARED) * latitude.sin();

        return LLA {
            latitude,
            longitude,
            altitude: altitude as f32,
        };
    }

    pub fn to_tangent(&self, reference: &Reference) -> Vector3<f32> {
        reference.ecef_to_tangent(*self)
    }

    pub fn from_tangent(reference: &Reference, tangent: Vector3<f32>) -> ECEF {
        reference.tangent_to_ecef(tangent)
    }
}

impl Add for ECEF {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Sub for ECEF {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Mul<f64> for ECEF {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl Div<f64> for ECEF {
    type Output = Self;
    fn div(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

impl Mul<ECEF> for f64 {
    type Output = ECEF;
    fn mul(self, rhs: ECEF) -> Self::Output {
        rhs * self
    }
}

impl Neg for ECEF {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}
