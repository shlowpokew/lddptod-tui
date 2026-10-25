//! rover-tui: one-screen debugger for the breadcrumbs rover.
//! Run it on the Pi next to the ROS container (host networking, same domain).

mod demo;
mod lidar;
mod msgs;
mod ros;
mod state;
mod sys;
mod ui;

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};

const HELP: &str = "\
rover-tui: terminal debugger for the breadcrumbs rover

USAGE: rover-tui [options]

  --ros2 DIR        breadcrumbs checkout on the Pi (default: ~/etc/ros2)
                    reads DIR/.env (ROS_DOMAIN_ID, LIDAR_YAW_RAD) and
                    DIR/maps/breadcrumbs.json
  --domain N        ROS_DOMAIN_ID (default: env, then DIR/.env, then 42)
  --lidar [DEV]     also read the LD19 straight off the serial port
                    (default /dev/lidar). Only with the container stopped:
                    two readers on one port corrupt each other's data.
  --no-ros          don't join DDS (host checks, docker logs, --lidar only)
  --container NAME  docker container to watch (default: ros2-mapping-1)
  --no-docker-logs  don't follow docker logs
  --demo            fake data, no robot needed
  -h, --help        this text
";

struct Args {
    ros2_dir: PathBuf,
    domain: Option<u16>,
    lidar: Option<String>,
    ros: bool,
    container: String,
    docker_logs: bool,
    demo: bool,
}

fn parse_args() -> Result<Args, String> {
    let home = std::env::var("HOME").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from("."));
    let mut args = Args {
        ros2_dir: home.join("etc").join("ros2"),
        domain: None,
        lidar: None,
        ros: true,
        container: "ros2-mapping-1".into(),
        docker_logs: true,
        demo: false,
    };
    let mut it = std::env::args().skip(1).peekable();
    while let Some(a) = it.next() {
        match a.as_str() {
            "-h" | "--help" => {
                print!("{HELP}");
                std::process::exit(0);
            }
            "--ros2" => args.ros2_dir = PathBuf::from(it.next().ok_or("--ros2 needs a directory")?),
            "--domain" => {
                let v = it.next().ok_or("--domain needs a number")?;
                args.domain = Some(v.parse().map_err(|_| format!("bad domain {v:?}"))?);
            }
            "--lidar" => {
                let dev = if it.peek().is_some_and(|next| !next.starts_with("--")) {
                    it.next().unwrap()
                } else {
                    "/dev/lidar".to_string()
                };
                args.lidar = Some(dev);
            }
            "--no-ros" => args.ros = false,
            "--container" => args.container = it.next().ok_or("--container needs a name")?,
            "--no-docker-logs" => args.docker_logs = false,
            "--demo" => args.demo = true,
            other => return Err(format!("unknown option {other:?} (try --help)")),
        }
    }
    Ok(args)
}

/// KEY=VALUE lines, as docker compose reads them.
fn read_env_file(path: &Path) -> HashMap<String, String> {
    let Ok(text) = std::fs::read_to_string(path) else { return HashMap::new() };
    text.lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .filter_map(|l| l.split_once('='))
        .map(|(k, v)| (k.trim().to_string(), v.trim().trim_matches('"').trim_matches('\'').to_string()))
        .collect()
}

fn setting(name: &str, env_file: &HashMap<String, String>) -> Option<String> {
    std::env::var(name).ok().filter(|v| !v.is_empty()).or_else(|| env_file.get(name).filter(|v| !v.is_empty()).cloned())
}

fn main() {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("rover-tui: {e}");
            std::process::exit(2);
        }
    };
    let env_file = read_env_file(&args.ros2_dir.join(".env"));
    let domain = args
        .domain
        .or_else(|| setting("ROS_DOMAIN_ID", &env_file).and_then(|v| v.parse().ok()))
        .unwrap_or(42);
    let yaw = setting("LIDAR_YAW_RAD", &env_file).and_then(|v| v.parse().ok()).unwrap_or(0.0);

    let shared = Arc::new(Mutex::new(state::State::new(domain, yaw)));
    {
        let mut st = shared.lock().unwrap();
        let env_note = if env_file.is_empty() { "not found" } else { "loaded" };
        st.event(state::INFO, "tui", format!("{}/.env {env_note}; domain {domain}, lidar yaw {yaw}", args.ros2_dir.display()));
    }

    if args.demo {
        demo::spawn(shared.clone());
    } else {
        if args.ros {
            ros::spawn(shared.clone(), domain);
        } else {
            shared.lock().unwrap().ros_status = "DDS off (--no-ros)".into();
        }
        if let Some(dev) = &args.lidar {
            lidar::spawn(shared.clone(), dev.clone());
        }
        sys::spawn(shared.clone(), args.container.clone(), args.ros2_dir.clone(), args.docker_logs);
    }

    let mut terminal = ratatui::init();
    let result = run(&mut terminal, &shared);
    ratatui::restore();
    if let Err(e) = result {
        eprintln!("rover-tui: {e}");
        std::process::exit(1);
    }
    // DDS and docker threads don't shut down cleanly; just leave.
    std::process::exit(0);
}

fn run(terminal: &mut ratatui::DefaultTerminal, shared: &state::Shared) -> std::io::Result<()> {
    let mut view = ui::View::default();
    loop {
        {
            let st = shared.lock().unwrap();
            terminal.draw(|f| ui::draw(f, &st, &mut view))?;
        }
        // ~10 fps: plenty for a 10 Hz LiDAR and easy on the Pi.
        if !event::poll(Duration::from_millis(100))? {
            continue;
        }
        let Event::Key(key) = event::read()? else { continue };
        if key.kind != KeyEventKind::Press {
            continue;
        }
        if key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('c') {
            return Ok(());
        }
        let k = match key.code {
            KeyCode::Char(c) => ui::Key::Char(c),
            KeyCode::Up => ui::Key::Up,
            KeyCode::Down => ui::Key::Down,
            KeyCode::Left => ui::Key::Left,
            KeyCode::Right => ui::Key::Right,
            KeyCode::PageUp => ui::Key::PageUp,
            KeyCode::PageDown => ui::Key::PageDown,
            KeyCode::Home => ui::Key::Home,
            KeyCode::End => ui::Key::End,
            KeyCode::Tab => ui::Key::Tab,
            KeyCode::BackTab => ui::Key::BackTab,
            KeyCode::Esc => return Ok(()),
            _ => continue,
        };
        let st = shared.lock().unwrap();
        if !view.on_key(k, &st) {
            return Ok(());
        }
    }
}
