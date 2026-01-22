use crate::*;

#[derive(Clone, Debug)]
pub struct Reference {
    pub lla: LLA,
    pub ecef: ECEF,
    sin_lat: f64,
    sin_lon: f64,
    cos_lat: f64,
    cos_lon: f64,
}

impl Reference {
    pub fn new(reference: LLA) -> Self {
        Reference {
            lla: reference,
            ecef: reference.to_ecef(),
            sin_lat: reference.latitude.sin(),
            sin_lon: reference.longitude.sin(),
            cos_lat: reference.latitude.cos(),
            cos_lon: reference.longitude.cos(),
        }
    }

    pub fn ecef_to_tangent(&self, ecef: ECEF) -> Vector3<f32> {
        let dxyz_local_ecef0 = ecef.x - self.ecef.x;
        let dxyz_local_ecef1 = ecef.y - self.ecef.y;
        let dxyz_local_ecef2 = ecef.z - self.ecef.z;

        Vector3::new(
            ((-self.sin_lat * self.cos_lon) * dxyz_local_ecef0 + (-self.sin_lat * self.sin_lon) * dxyz_local_ecef1 + self.cos_lat * dxyz_local_ecef2) as f32,
            ((-self.sin_lon) * dxyz_local_ecef0 + self.cos_lon * dxyz_local_ecef1 + (0.) * dxyz_local_ecef2) as f32,
            ((-self.cos_lat * self.cos_lon) * dxyz_local_ecef0 + (-self.cos_lat * self.sin_lon) * dxyz_local_ecef1 + (-self.sin_lat) * dxyz_local_ecef2) as f32,
        )
    }

    pub fn lla_to_tangent(&self, lla: LLA) -> Vector3<f32> {
        self.ecef_to_tangent(lla.to_ecef())
    }

    pub fn tangent_to_ecef(&self, tangent: Vector3<f32>) -> ECEF {
        ECEF::new(
            -self.sin_lat * self.cos_lon * tangent.x as f64 - self.sin_lon * tangent.y as f64 - self.cos_lat * self.cos_lon * tangent.z as f64,
            -self.sin_lat * self.sin_lon * tangent.x as f64 + self.cos_lon * tangent.y as f64 - self.cos_lat * self.sin_lon * tangent.z as f64,
            self.cos_lat * tangent.x as f64 - self.sin_lat * tangent.z as f64,
        )
    }

    pub fn tangent_to_lla(&self, tangent: Vector3<f32>) -> LLA {
        (self.tangent_to_ecef(tangent) + self.ecef).to_lla()
    }
}
