//! DDS side: one thread owns the ROS 2 node, drains every subscription and
//! writes into the shared state. Nothing here needs ROS installed.

use std::thread;
use std::time::{Duration, Instant};

use ros2_client::qos::{Durability, History, WhenFull};
use ros2_client::{
    Context, ContextOptions, MessageTypeName, Name, Node, NodeName, NodeOptions, QosProfile,
    Subscription,
};
use serde::de::DeserializeOwned;

use crate::msgs::*;
use crate::state::{LogLine, MapView, OdomView, Pose2, ScanView, Shared, State, GraphTopic, clock};

type Handler = Box<dyn FnMut(&mut State)>;

pub fn spawn(shared: Shared, domain: u16) {
    thread::Builder::new()
        .name("ros".into())
        .spawn(move || {
            if let Err(error) = run(&shared, domain) {
                let mut st = shared.lock().unwrap();
                st.ros_status = format!("DDS failed: {error}");
                st.event(crate::state::ERROR, "tui", format!("DDS failed: {error}"));
            }
        })
        .expect("spawn ros thread");
}

fn err(e: impl std::fmt::Debug) -> String {
    format!("{e:?}")
}

fn subscribe<M: DeserializeOwned + 'static>(
    node: &mut Node,
    topic: &str,
    package: &str,
    type_name: &str,
    qos: &QosProfile,
) -> Result<Subscription<M>, String> {
    let name = Name::parse(topic).map_err(err)?;
    let topic = node
        .create_topic(&name, MessageTypeName::new(package, type_name), qos)
        .map_err(err)?;
    node.create_subscription::<M>(&topic, None).map_err(err)
}

/// Wraps a subscription in a closure that drains it into the state.
fn handler<M, F>(subscription: Subscription<M>, topic: &'static str, mut apply: F) -> Handler
where
    M: DeserializeOwned + 'static,
    F: FnMut(&mut State, M) + 'static,
{
    Box::new(move |st: &mut State| {
        // Bounded so a flood on one topic can't starve the others.
        for _ in 0..200 {
            match subscription.take() {
                Ok(Some((message, _info))) => {
                    st.hit(topic);
                    apply(st, message);
                }
                Ok(None) => break,
                Err(e) => {
                    st.topic_error(topic, err(e));
                    break;
                }
            }
        }
    })
}

fn run(shared: &Shared, domain: u16) -> Result<(), String> {
    let context = Context::with_options(ContextOptions::new().domain_id(domain)).map_err(err)?;
    let mut node = context
        .new_node(
            NodeName::new("/debug", "rover_tui").map_err(err)?,
            NodeOptions::new().enable_rosout(false).start_parameter_services(false),
        )
        .map_err(err)?;

    // Best-effort + volatile matches any publisher, reliable or not.
    let sensor = QosProfile::subscription_default()
        .reliability_best_effort()
        .durability(Durability::Volatile)
        .history(History::KeepLast { depth: 10 });
    let reliable = QosProfile::subscription_default()
        .reliability_reliable(WhenFull::DEFAULT)
        .durability(Durability::Volatile)
        .history(History::KeepLast { depth: 200 });
    // /tf_static is published once, transient-local; we need the replay.
    let latched = QosProfile::subscription_default()
        .reliability_reliable(WhenFull::DEFAULT)
        .durability(Durability::TransientLocal)
        .history(History::KeepLast { depth: 100 });

    let mut handlers: Vec<Handler> = Vec::new();
    let mut failures: Vec<String> = Vec::new();
    macro_rules! sub {
        ($topic:literal, $pkg:literal, $ty:literal, $msg:ty, $qos:expr, $apply:expr) => {
            match subscribe::<$msg>(&mut node, $topic, $pkg, $ty, &$qos) {
                Ok(s) => handlers.push(handler::<$msg, _>(s, $topic, $apply)),
                Err(e) => failures.push(format!("{}: {}", $topic, e)),
            }
        };
    }

    for topic in ["/scan", "/scan/raw"] {
        // Topic names must be literals for the macro, so branch here.
        let result = subscribe::<LaserScan>(&mut node, topic, "sensor_msgs", "LaserScan", &sensor);
        match result {
            Ok(s) => {
                let key: &'static str = if topic == "/scan" { "/scan" } else { "/scan/raw" };
                handlers.push(handler::<LaserScan, _>(s, key, move |st: &mut State, m: LaserScan| {
                    let view = ScanView::from_msg(&m, st.lidar_yaw);
                    st.scans.insert(key.to_string(), view);
                }));
            }
            Err(e) => failures.push(format!("{topic}: {e}")),
        }
    }
    sub!("/odom", "nav_msgs", "Odometry", Odometry, sensor, |st: &mut State, m: Odometry| {
        st.odom = Some(OdomView {
            frame: m.header.frame_id,
            child: m.child_frame_id,
            pose: Pose2 {
                x: m.pose.pose.position.x,
                y: m.pose.pose.position.y,
                yaw: m.pose.pose.orientation.yaw(),
            },
            linear: m.twist.twist.linear.x,
            angular: m.twist.twist.angular.z,
        });
    });
    sub!("/wheel/velocity", "std_msgs", "Float32MultiArray", Float32MultiArray, sensor,
        |st: &mut State, m: Float32MultiArray| st.wheel_velocity = Some(m.data));
    sub!("/cmd_vel", "geometry_msgs", "Twist", Twist, sensor,
        |st: &mut State, m: Twist| st.cmd_vel = Some((m.linear.x, m.angular.z)));
    sub!("/mission/state", "std_msgs", "String", StringMsg, reliable,
        |st: &mut State, m: StringMsg| st.set_mission(m.data, false));
    sub!("/safety/warning", "std_msgs", "String", StringMsg, reliable,
        |st: &mut State, m: StringMsg| st.set_safety(m.data));
    sub!("/arduino/telemetry", "std_msgs", "String", StringMsg, reliable,
        |st: &mut State, m: StringMsg| st.set_telemetry(m.data));
    sub!("/arduino/command", "std_msgs", "String", StringMsg, reliable,
        |st: &mut State, m: StringMsg| st.event(crate::state::INFO, "/arduino/command", m.data));
    sub!("/human_target", "geometry_msgs", "PoseStamped", PoseStamped, sensor,
        |st: &mut State, m: PoseStamped| st.human_target = Some((m.pose.position.x, m.pose.position.y)));
    sub!("/sos/human_targets", "geometry_msgs", "PoseArray", PoseArray, sensor,
        |st: &mut State, m: PoseArray| {
            st.sos_targets = m.poses.iter().map(|p| (p.position.x, p.position.y)).collect();
        });

    macro_rules! flag {
        ($topic:literal) => {
            sub!($topic, "std_msgs", "Bool", BoolMsg, reliable,
                |st: &mut State, m: BoolMsg| st.set_flag($topic, m.data));
        };
    }
    flag!("/follow/enable");
    flag!("/breadcrumb/recording");
    flag!("/backtrack/enable");
    flag!("/backtrack/complete");
    flag!("/sos/seek_enable");
    flag!("/sos/human_nearby");
    flag!("/hardware/button_a");
    flag!("/hardware/button_b");
    flag!("/hardware/sos");

    sub!("/tf", "tf2_msgs", "TFMessage", TfMessage, sensor,
        |st: &mut State, m: TfMessage| st.set_tf(&m.transforms, false));
    sub!("/tf_static", "tf2_msgs", "TFMessage", TfMessage, latched,
        |st: &mut State, m: TfMessage| st.set_tf(&m.transforms, true));
    // slam_toolbox republishes the map every few seconds, so volatile works
    // whether its publisher is latched or not.
    sub!("/map", "nav_msgs", "OccupancyGrid", OccupancyGrid, reliable,
        |st: &mut State, m: OccupancyGrid| st.map = Some(MapView::from_msg(&m)));

    // /rosout already exists inside the Context; reuse its Topic.
    let rosout_topic = context.get_rosout_topic();
    match node.create_subscription::<ros2_client::Log>(&rosout_topic, Some(reliable.clone())) {
        Ok(s) => handlers.push(handler::<ros2_client::Log, _>(s, "/rosout", |st: &mut State, m: ros2_client::Log| {
            st.rosout.push(LogLine { time: clock(), level: m.level, source: m.name, text: m.msg });
        })),
        Err(e) => failures.push(format!("/rosout: {}", err(e))),
    }

    {
        let mut st = shared.lock().unwrap();
        st.ros_status = format!("DDS up, domain {domain}, {} subscriptions", handlers.len());
        st.event(crate::state::INFO, "tui", format!("joined ROS 2 domain {domain} as /debug/rover_tui"));
        for f in &failures {
            st.event(crate::state::ERROR, "tui", format!("subscribe failed: {f}"));
        }
    }

    let mut last_graph: Option<Instant> = None;
    loop {
        {
            let mut st = shared.lock().unwrap();
            for h in handlers.iter_mut() {
                h(&mut st);
            }
        }
        if last_graph.is_none_or(|t| t.elapsed() >= Duration::from_secs(1)) {
            last_graph = Some(Instant::now());
            // Graph queries walk discovery data; do them without the lock.
            let mut graph: Vec<GraphTopic> = context
                .discovered_topics()
                .into_iter()
                .filter_map(|t| {
                    let ros_name = format!("/{}", t.name.strip_prefix("rt/")?);
                    Some(GraphTopic {
                        publishers: node.publisher_count(&ros_name),
                        subscribers: node.subscription_count(&ros_name),
                        name: ros_name,
                        type_name: pretty_type(&t.type_name),
                    })
                })
                .collect();
            graph.sort_by(|a, b| a.name.cmp(&b.name));
            graph.dedup_by(|a, b| a.name == b.name);
            shared.lock().unwrap().graph = graph;
        }
        thread::sleep(Duration::from_millis(5));
    }
}

/// `sensor_msgs::msg::dds_::LaserScan_` -> `sensor_msgs/msg/LaserScan`
pub fn pretty_type(dds: &str) -> String {
    dds.replace("::dds_::", "::").replace("::", "/").trim_end_matches('_').to_string()
}

#[cfg(test)]
mod tests {
    #[test]
    fn pretty_type_names() {
        assert_eq!(super::pretty_type("sensor_msgs::msg::dds_::LaserScan_"), "sensor_msgs/msg/LaserScan");
    }
}
