mod earth;
mod ecef;
mod lla;
mod prelude;
mod reference;
mod skypack;

pub use crate::prelude::*;
use crate::skypack::{DeviceError, ResponsePacket, Skypack};
use anyhow::Result;
use chrono::Utc;
use clap::Parser;
use rand::Rng;
use rand::rngs::ThreadRng;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::{thread::sleep, time::Duration};
use tokio::time::Instant;

#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    /// Horizontal noise, peak-peak (meters)
    #[arg(long = "h-noise")]
    #[arg(long, default_value = "0")]
    h_noise: f32,

    /// Vertical noise amplitude, peak-peak (meters)
    #[arg(long = "v-noise")]
    #[arg(long, default_value = "0")]
    v_noise: f32,

    /// Rate (Hz)
    #[arg(long)]
    #[arg(long, default_value = "1")]
    rate: f32,

    /// Velocity (m/s)
    #[arg(long)]
    #[arg(long, default_value = "0")]
    vel: f32,

    /// Delay (secs)
    #[arg(long)]
    #[arg(long, default_value = "0")]
    delay: f32,

    /// Velocity direction in degrees
    #[arg(long = "vel-degrees")]
    #[arg(long, default_value = "0")]
    vel_degrees: f32,

    /// IP address
    #[arg(long)]
    #[arg(long, default_value = "127.0.0.1")]
    ip: String,

    /// Port
    #[arg(long)]
    #[arg(long, default_value_t = 41263)]
    port: u16,
}

fn get_nav_lla(telemetry: &serde_json::Value) -> Result<LLA> {
    let latitude = telemetry["nav"]["lla"][0]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("Telemetry data has no latitude"))?;
    let longitude = telemetry["nav"]["lla"][1]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("Telemetry data has no latitude"))?;
    let altitude = telemetry["nav"]["lla"][2]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("Telemetry data has no latitude"))?
        as f32;
    Ok(LLA::from_degs(
        latitude.to_degrees(),
        longitude.to_degrees(),
        altitude,
    ))
}

fn get_locked_gnss_time_secs(telemetry: &serde_json::Value) -> Result<f64> {
    let clocks = telemetry["time"]["clocks"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("Telemetry has no clocks"))?;
    let gnss_clock = clocks
        .iter()
        .filter(|x| x.get("name").unwrap().as_str().unwrap() == "gnss")
        .next()
        .ok_or_else(|| anyhow::anyhow!("Telemetry has no GNSS clock"))?;
    let is_utc = gnss_clock.get("scale").map(|x| x.as_i64()).flatten() == Some(1);
    if !is_utc {
        return Err(anyhow::anyhow!("GNSS clock is not UTC"));
    }
    let is_synchronized = gnss_clock.get("state").map(|x| x.as_i64()).flatten() == Some(2);
    if !is_synchronized {
        return Err(anyhow::anyhow!("GNSS clock is not synchronized"));
    }
    let time = gnss_clock
        .get("time")
        .map(|x| x.as_f64())
        .flatten()
        .ok_or_else(|| anyhow::anyhow!("GNSS clock has no time"))?;

    Ok(time)
}

fn get_reference(telemetry: &serde_json::Value) -> Option<LLA> {
    let reference = telemetry.get("ref")?.as_array()?;
    Some(LLA::from_degs(
        reference.get(0)?.as_f64()?,
        reference.get(1)?.as_f64()?,
        reference.get(2)?.as_f64()? as f32,
    ))
}

struct App {
    skypack: Arc<Skypack>,
    rng: ThreadRng,
    velocity_ned: nalgebra::Vector3<f32>,
    delay: f32,
    interval: Duration,
    reference: Reference,
    init_utc: f64,
    h_noise: f32,
    v_noise: f32,
}

impl App {
    async fn run(&mut self) {
        loop {
            let now = Instant::now();

            match self.iteration().await {
                Ok(_) => (),
                Err(e) => {
                    eprintln!("Iteration failed: {}", e)
                }
            }

            let work_time = now.elapsed();
            if work_time < self.interval {
                sleep(self.interval - work_time);
            }
        }
    }

    async fn iteration(&mut self) -> Result<()> {
        //Fetch telemetry
        let telemetry = {
            let response = self.skypack.get_telemetry_sync()?;
            if response.res != 0 {
                None
            } else {
                response.data
            }
        }
        .ok_or_else(|| anyhow::anyhow!("Failed to fetch telemetry"))?;

        let skymate_utc = get_locked_gnss_time_secs(&telemetry)?;
        let elapsed_secs = skymate_utc - self.init_utc - self.delay as f64;
        let offset_tangent = self.velocity_ned * elapsed_secs as f32;
        let lla = self.reference.tangent_to_lla(
            offset_tangent
                + Vector3::new(
                    self.rng.random_range(-1_f32..1_f32) * self.h_noise / 2.,
                    self.rng.random_range(-1_f32..1_f32) * self.h_noise / 2.,
                    self.rng.random_range(-1_f32..1_f32) * self.v_noise / 2.,
                ),
        );

        self.skypack
            .set_precision_landing_zone(lla, self.velocity_ned, skymate_utc)
            .await?;
        println!(
            "Sent {}, {}, {}",
            lla.latitude.to_degrees(),
            lla.longitude.to_degrees(),
            lla.altitude
        );

        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let addr = format!("{}:{}", args.ip, args.port);
    let rng = rand::rng();
    let skypack = Skypack::new("0.0.0.0:0", &addr).await?;
    let angle = args.vel_degrees.to_radians();
    let velocity_ned = Vector3::new(angle.cos() * args.vel, angle.sin() * args.vel, 0.0);

    //Use SKYMATE initial position
    println!("Acquiring SKYMATE Reference...");
    let reference_lla = loop {
        match skypack.get_telemetry().await {
            Ok(telemetry) => match get_reference(&telemetry.data.unwrap()) {
                None => continue,
                Some(lla) => break lla,
                _ => continue,
            },
            _ => continue,
        }
    };
    let reference = Reference::new(reference_lla);
    println!(
        "SKYMATE Reference: {}, {}, {}",
        reference_lla.latitude.to_degrees(),
        reference_lla.longitude.to_degrees(),
        reference_lla.altitude
    );

    println!("Acquiring SKYMATE UTC Time...");
    let init_utc = loop {
        match skypack.get_telemetry().await {
            Ok(telemetry) => match get_locked_gnss_time_secs(&telemetry.data.unwrap()) {
                Ok(t) => break t,
                Err(_) => continue,
            },
            Err(_) => continue,
        }
    };
    let now_utc = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();
    if init_utc > now_utc {
        println!(
            "SKYMATE UTC Time: {} ({}  secs in the future)",
            init_utc,
            init_utc - now_utc
        );
    } else {
        println!(
            "SKYMATE UTC Time: {} ({} secs in the past)",
            init_utc,
            now_utc - init_utc
        );
    }

    let mut app = App {
        skypack,
        rng,
        velocity_ned,
        delay: args.delay,
        interval: Duration::from_secs_f32(1.0 / args.rate),
        reference,
        init_utc,
        h_noise: args.h_noise,
        v_noise: args.v_noise,
    };
    app.run().await;

    Ok(())
}
