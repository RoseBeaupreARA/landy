pub struct Earth {}

impl Earth {
    pub const FLATNESS: f64 = 1.0 / 298.257223563;
    pub const ECCENTRICITY_SQUARED: f64 = Earth::FLATNESS * (2. - Earth::FLATNESS);
    pub const EQUATORIAL_RADIUS: f64 = 6378137.0;
    pub const GEOMAGNETIC_RADIUS: f64 = 6371200.0;
    pub const ANGULAR_SPEED: f64 = 7.2921E-5;
    pub const ANGULAR_SPEED_SQUARED: f64 = Earth::ANGULAR_SPEED * Earth::ANGULAR_SPEED;
    pub const GRAVITY: f32 = 9.80639076;
}
