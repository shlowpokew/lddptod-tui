#![allow(dead_code)]
//! ROS 2 message layouts, field for field, so serde can decode the CDR bytes.
//! Only the messages the rover stack publishes are here.

use serde::Deserialize;

#[derive(Deserialize, Clone, Debug, Default)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Deserialize, Clone, Debug, Default)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

#[derive(Deserialize, Clone, Debug, Default)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Deserialize, Clone, Debug, Default)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Quaternion {
    pub fn yaw(&self) -> f64 {
        (2.0 * (self.w * self.z + self.x * self.y))
            .atan2(1.0 - 2.0 * (self.y * self.y + self.z * self.z))
    }
}

#[derive(Deserialize, Clone, Debug, Default)]
pub struct Pose {
    pub position: Vector3,
    pub orientation: Quaternion,
}

#[derive(Deserialize, Clone, Debug, Default)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

/// sensor_msgs/LaserScan
#[derive(Deserialize, Clone, Debug, Default)]
pub struct LaserScan {
    pub header: Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: Vec<f32>,
    pub intensities: Vec<f32>,
}

// float64[36] covariance. serde only derives arrays up to 32 long, and CDR
// writes fixed arrays without a length, so 6 x 6 decodes the same bytes.
#[derive(Deserialize, Clone, Debug)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    pub covariance: [[f64; 6]; 6],
}

#[derive(Deserialize, Clone, Debug)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    pub covariance: [[f64; 6]; 6],
}

/// nav_msgs/Odometry
#[derive(Deserialize, Clone, Debug)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: String,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}

/// geometry_msgs/PoseStamped
#[derive(Deserialize, Clone, Debug)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

/// geometry_msgs/PoseArray
#[derive(Deserialize, Clone, Debug)]
pub struct PoseArray {
    pub header: Header,
    pub poses: Vec<Pose>,
}

/// std_msgs/String
#[derive(Deserialize, Clone, Debug)]
pub struct StringMsg {
    pub data: String,
}

/// std_msgs/Bool
#[derive(Deserialize, Clone, Debug)]
pub struct BoolMsg {
    pub data: bool,
}

#[derive(Deserialize, Clone, Debug)]
pub struct MultiArrayDimension {
    pub label: String,
    pub size: u32,
    pub stride: u32,
}

#[derive(Deserialize, Clone, Debug)]
pub struct MultiArrayLayout {
    pub dim: Vec<MultiArrayDimension>,
    pub data_offset: u32,
}

/// std_msgs/Float32MultiArray
#[derive(Deserialize, Clone, Debug)]
pub struct Float32MultiArray {
    pub layout: MultiArrayLayout,
    pub data: Vec<f32>,
}

#[derive(Deserialize, Clone, Debug)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}

#[derive(Deserialize, Clone, Debug)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub transform: Transform,
}

/// tf2_msgs/TFMessage
#[derive(Deserialize, Clone, Debug)]
pub struct TfMessage {
    pub transforms: Vec<TransformStamped>,
}

#[derive(Deserialize, Clone, Debug)]
pub struct MapMetaData {
    pub map_load_time: Time,
    pub resolution: f32,
    pub width: u32,
    pub height: u32,
    pub origin: Pose,
}

/// nav_msgs/OccupancyGrid
#[derive(Deserialize, Clone, Debug)]
pub struct OccupancyGrid {
    pub header: Header,
    pub info: MapMetaData,
    pub data: Vec<i8>,
}
