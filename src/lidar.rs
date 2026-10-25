//! Direct LD19 serial reader, for checking the sensor with the ROS stack
//! stopped (`--lidar /dev/lidar`). Two readers on one tty split the bytes
//! between them, so don't run this while the container holds the port.
//!
//! Packet (47 bytes): 54 2C | speed u16 | start u16 | 12 x (mm u16, intensity u8)
//! | end u16 | timestamp u16 | crc8. Angles are 0.01 degree, clockwise.

use std::io::Read;
use std::thread;
use std::time::Duration;

use crate::state::{ScanView, Shared, ERROR, INFO, WARN};

pub const SERIAL_TOPIC: &str = "serial:ld19";
pub const PACKET_LEN: usize = 47;
const POINTS: usize = 12;
const HEADER: u8 = 0x54;
const VER_LEN: u8 = 0x2C;

/// LDROBOT's CRC-8 (poly 0x4D, MSB first, init 0).
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x4D } else { crc << 1 };
        }
    }
    crc
}

#[derive(Debug, Clone, PartialEq)]
pub struct Packet {
    pub speed_dps: u16,
    pub start_deg: f64,
    pub end_deg: f64,
    pub points: [(f64, u16, u8); POINTS], // (angle deg, distance mm, intensity)
}

pub fn parse_packet(p: &[u8]) -> Option<Packet> {
    if p.len() < PACKET_LEN || p[0] != HEADER || p[1] != VER_LEN || crc8(&p[..PACKET_LEN - 1]) != p[PACKET_LEN - 1] {
        return None;
    }
    let u16_at = |i: usize| u16::from_le_bytes([p[i], p[i + 1]]);
    let start = u16_at(4) as f64 / 100.0;
    let end = u16_at(42) as f64 / 100.0;
    let span = (end - start).rem_euclid(360.0);
    let step = span / (POINTS - 1) as f64;
    let mut points = [(0.0, 0u16, 0u8); POINTS];
    for (i, point) in points.iter_mut().enumerate() {
        let base = 6 + i * 3;
        *point = ((start + step * i as f64).rem_euclid(360.0), u16_at(base), p[base + 2]);
    }
    Some(Packet { speed_dps: u16_at(2), start_deg: start, end_deg: end, points })
}

/// Collects packets into revolutions and turns each into a `ScanView`.
pub struct Assembler {
    buf: Vec<u8>,
    rev: Vec<(f64, u16, u8)>,
    last_start: f64,
    pub bad_crc: u64,
    pub speed_dps: u16,
}

impl Default for Assembler {
    fn default() -> Self {
        Assembler { buf: Vec::new(), rev: Vec::new(), last_start: -1.0, bad_crc: 0, speed_dps: 0 }
    }
}

impl Assembler {
    /// Feed raw bytes; returns completed revolutions as (angle deg, mm, intensity).
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<Vec<(f64, u16, u8)>> {
        self.buf.extend_from_slice(bytes);
        let mut done = Vec::new();
        let mut i = 0;
        while i + PACKET_LEN <= self.buf.len() {
            if self.buf[i] != HEADER || self.buf[i + 1] != VER_LEN {
                i += 1;
                continue;
            }
            match parse_packet(&self.buf[i..i + PACKET_LEN]) {
                Some(packet) => {
                    self.speed_dps = packet.speed_dps;
                    if packet.start_deg < self.last_start && !self.rev.is_empty() {
                        done.push(std::mem::take(&mut self.rev));
                    }
                    self.last_start = packet.start_deg;
                    self.rev.extend_from_slice(&packet.points);
                    i += PACKET_LEN;
                }
                None => {
                    self.bad_crc += 1;
                    i += 1;
                }
            }
        }
        self.buf.drain(..i);
        if self.buf.len() > 4096 {
            self.buf.clear();
        }
        done
    }
}

pub fn spawn(shared: Shared, path: String) {
    thread::Builder::new()
        .name("lidar".into())
        .spawn(move || loop {
            let opened = serialport::new(&path, 230_400).timeout(Duration::from_millis(200)).open();
            let mut port = match opened {
                Ok(p) => {
                    shared.lock().unwrap().event(INFO, "lidar", format!("reading {path} at 230400 baud"));
                    p
                }
                Err(e) => {
                    shared.lock().unwrap().event(ERROR, "lidar", format!("open {path}: {e}"));
                    thread::sleep(Duration::from_secs(3));
                    continue;
                }
            };
            let mut assembler = Assembler::default();
            let mut buf = [0u8; 1024];
            loop {
                match port.read(&mut buf) {
                    Ok(0) => continue,
                    Ok(n) => {
                        let revs = assembler.feed(&buf[..n]);
                        if revs.is_empty() {
                            continue;
                        }
                        let yaw = shared.lock().unwrap().lidar_yaw;
                        for rev in revs {
                            // LD19 angles run clockwise; ROS (and the plot) want CCW.
                            let beams = rev.iter().map(|(deg, mm, _)| (-deg.to_radians(), *mm as f64 / 1000.0));
                            let view = ScanView::build("ld19 (serial)".into(), beams, yaw, 0.02, 12.0);
                            let mut st = shared.lock().unwrap();
                            st.hit(SERIAL_TOPIC);
                            st.scans.insert("serial".into(), view);
                            st.serial_lidar = Some((assembler.speed_dps, assembler.bad_crc));
                        }
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(e) => {
                        shared.lock().unwrap().event(WARN, "lidar", format!("read {path}: {e}; reopening"));
                        break;
                    }
                }
            }
            thread::sleep(Duration::from_secs(1));
        })
        .expect("spawn lidar thread");
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(start_cdeg: u16, end_cdeg: u16, mm: u16) -> Vec<u8> {
        let mut p = vec![HEADER, VER_LEN];
        p.extend_from_slice(&3600u16.to_le_bytes());
        p.extend_from_slice(&start_cdeg.to_le_bytes());
        for _ in 0..POINTS {
            p.extend_from_slice(&mm.to_le_bytes());
            p.push(200);
        }
        p.extend_from_slice(&end_cdeg.to_le_bytes());
        p.extend_from_slice(&1234u16.to_le_bytes());
        let crc = crc8(&p);
        p.push(crc);
        p
    }

    #[test]
    fn crc_table_matches_ldrobot() {
        // First entries of LDROBOT's CrcTable[] are CRC of single bytes.
        let table: Vec<u8> = (0..8u8).map(|b| crc8(&[b])).collect();
        assert_eq!(table, vec![0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae]);
    }

    #[test]
    fn parses_packet_and_interpolates() {
        let p = packet(1000, 2100, 1500);
        assert_eq!(p.len(), PACKET_LEN);
        let parsed = parse_packet(&p).unwrap();
        assert_eq!(parsed.points[0].0, 10.0);
        assert!((parsed.points[11].0 - 21.0).abs() < 1e-9);
        assert_eq!(parsed.points[5].1, 1500);
        let mut bad = p.clone();
        bad[10] ^= 1;
        assert!(parse_packet(&bad).is_none());
    }

    #[test]
    fn assembles_revolutions_across_wrap() {
        let mut a = Assembler::default();
        let mut bytes = vec![0x00, 0x13]; // junk before sync
        bytes.extend(packet(35000, 35900, 1000));
        bytes.extend(packet(100, 1000, 1000)); // wrapped: new revolution
        let revs = a.feed(&bytes[..30]);
        assert!(revs.is_empty());
        let revs = a.feed(&bytes[30..]);
        assert_eq!(revs.len(), 1);
        assert_eq!(revs[0].len(), POINTS);
    }
}
