#!/usr/bin/env python3
"""Capture one native powerplant SITL flight, calibrate, or verify frozen RGB-D SLAM."""
import argparse
from contextlib import ExitStack, contextmanager
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from validation.slam.experiment import run_evidence
from validation.slam import score

HERE = Path(__file__).resolve().parent
RAW = ["/clock", "/camera", "/camera/camera_info", "/depth_camera",
       "/depth_camera/camera_info", "/slam/input/imu", "/slam/input/occluded"]


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def software():
    from ament_index_python.packages import get_package_share_directory
    import xml.etree.ElementTree as ET
    # The RTAB-Map core Debian package does not register itself in the ament index.
    version_lines = subprocess.check_output(["rtabmap", "--version"], text=True).splitlines()
    versions = {"rtabmap": next(line.split(":", 1)[1].strip() for line in version_lines
                                if line.startswith("RTAB-Map:"))}
    for name in ("rtabmap_slam", "rtabmap_odom", "rtabmap_msgs", "imu_filter_madgwick",
                 "px4_powerplant_slam"):
        share = Path(get_package_share_directory(name))
        versions[name] = ET.parse(share/"package.xml").getroot().findtext("version")
    # Include effective installed code (symlink installs are supported).
    from px4_powerplant_slam import inputs, geometry
    launch = Path(get_package_share_directory("px4_powerplant_slam"))/"launch/simulation.launch.py"
    for path in [launch, Path(inputs.__file__), Path(geometry.__file__), HERE/"route.py",
                 HERE/"score.py", HERE/"observe.py", HERE/"experiment.py", HERE/"capture_route.py",
                 HERE/"route_evidence.py", HERE/"run.py"]:
        versions[path.name] = digest(path)
    return versions


@contextmanager
def process(args, env, log):
    with log.open("x") as output:
        child = subprocess.Popen(args, env=env, stdout=output, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        try:
            yield child
        finally:
            for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    child.wait(wait)
                    break
                except subprocess.TimeoutExpired:
                    continue


@contextmanager
def estimator(run, loops, env):
    run.mkdir(parents=True, exist_ok=False)
    with ExitStack() as stack:
        monitor = stack.enter_context(process([sys.executable, str(HERE/"observe.py"), str(run)],
                                              env, run/"observer.log"))
        launch = stack.enter_context(process([
            "ros2", "launch", "px4_powerplant_slam", "simulation.launch.py",
            "wait_for_capture_start:=true",
            "database_path:=" + str(run/"rtabmap.db"), "loop_closure:=" + str(loops).lower()
        ], env, run/"launch.log"))
        def healthy():
            for name, proc in (("observer", monitor), ("launch", launch)):
                if proc.poll() is not None:
                    raise RuntimeError(f"{name} exited ({proc.returncode}); see {run}")
        deadline = time.monotonic() + 65
        while not (run/"ready").is_file():
            healthy()
            if time.monotonic() > deadline:
                raise TimeoutError(f"SLAM publishers and input subscriptions missing; see {run}")
            time.sleep(.2)
        yield healthy


def replay(capture, run, loops, args):
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.replay_domain))
    if env["ROS_DOMAIN_ID"] == os.environ.get("ROS_DOMAIN_ID", "0"):
        raise ValueError("the replay domain must differ from the live SITL domain")
    meta = yaml.safe_load((capture/"raw/metadata.yaml").read_text())["rosbag2_bagfile_information"]
    duration = meta["duration"]["nanoseconds"] * 1e-9
    if duration <= 0:
        raise ValueError("empty bag timeline")
    with estimator(run, loops, env) as healthy:
        # The original /clock is in the MCAP. Rosbag must not synthesize a second clock.
        with process(["ros2", "bag", "play", str(capture/"raw"), "--rate", str(args.rate),
                      "--read-ahead-queue-size", "10", "--delay", "2", "--topics", *RAW],
                     env, run/"play.log") as player:
            deadline = time.monotonic() + 2 * duration / args.rate + 90
            while player.poll() is None:
                healthy()
                if time.monotonic() > deadline:
                    raise TimeoutError(f"playback did not finish within its bag duration budget: {run}")
                time.sleep(.3)
            if player.returncode != 0:
                raise RuntimeError(f"rosbag playback failed; see {run/'play.log'}")
        # Allow late asynchronous graph optimization to finish before taking its final revision.
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            healthy()
            quiet = time.time() - max((run/name).stat().st_mtime for name in
                                      ("odom.jsonl", "graphs.jsonl", "info.jsonl", "depth.jsonl"))
            if quiet >= 6:
                break
            time.sleep(.3)
        else:
            raise TimeoutError("estimator did not drain after the end of the bag")
    return run_evidence(run, capture)


def dump(path, value):
    with path.open("x") as out:
        json.dump(value, out, indent=2, allow_nan=False)
        out.write("\n")


def frozen_identity(path):
    companion = path.with_name(path.name + ".identity.json")
    data = json.loads(companion.read_text())
    if data["thresholds_sha256"] != digest(path) or data["software"] != software():
        raise ValueError("frozen software, route or thresholds differ from this run")
    return data


def capture(args):
    dest = args.output_dir
    route = [sys.executable, str(HERE/"capture_route.py"), "--output-dir", str(dest),
             "--px4-root", str(args.px4_root), "--split", args.split]
    if args.prepare_only:
        subprocess.run([*route, "--prepare-only"], check=True)
        return
    dest.mkdir(parents=True, exist_ok=False)
    identity = {"software": software(), "profile": "native", "split": args.split}
    if args.split == "blind":
        if args.thresholds is None:
            raise ValueError("blind capture requires frozen --thresholds")
        frozen_identity(args.thresholds)
        identity["thresholds_sha256"] = digest(args.thresholds)
    dump(dest/"setup.json", identity)
    with estimator(dest/"online", True, os.environ.copy()) as healthy:
        # The route driver owns flight termination and confirms landing on errors.
        with subprocess.Popen(route) as driver:
            try:
                while driver.poll() is None:
                    healthy()
                    time.sleep(.3)
                if driver.returncode:
                    raise RuntimeError(f"route capture exited {driver.returncode}; see {dest}")
            finally:
                if driver.poll() is None:
                    driver.send_signal(signal.SIGINT)
                    driver.wait()
        time.sleep(3)
        healthy()


def grade(args):
    cap = args.capture
    manifest = json.loads((cap/"manifest.json").read_text())
    setup = json.loads((cap/"setup.json").read_text())
    split = "calibration" if args.command == "calibrate" else "blind"
    if manifest["split"] != split or setup["split"] != split or setup["software"] != software():
        raise ValueError("capture split or software differs from the requested experiment")
    if not manifest["passed"] or setup["profile"] != "native":
        raise ValueError("grading requires a completed native flight and validated raw evidence")
    raw_bag = cap/"raw"/manifest["bag_file"]
    if raw_bag.parent != cap/"raw" or digest(raw_bag) != manifest["bag_sha256"]:
        raise ValueError("raw MCAP does not match the captured bag SHA-256")
    assets = {key: manifest[key] for key in
              ("world_sha256", "assets_sha256", "model_sha256", "camera_sha256", "loaded_profile")}
    if split == "blind":
        identity = frozen_identity(args.thresholds)
        if assets != identity["assets"]:
            raise ValueError("blind world, model or live camera profile differs from calibration")
        if setup["thresholds_sha256"] != digest(args.thresholds):
            raise ValueError("blind capture predates the selected frozen thresholds")
    if (args.output_dir.exists() and any(args.output_dir.iterdir())):
        raise ValueError("select a new output directory to preserve existing trials")
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    evidence = {"schema_version": 1, "capture_id": manifest["capture_id"], "split": split,
                "bag_sha256": manifest["bag_sha256"], "truth": str(cap/"truth.csv"),
                "windows": str(cap/"events.json"), "reference_b": str(cap/"b_reference.npy"),
                "voxel_m": .25, "capture_manifest": str(cap/"manifest.json"), "rounds": []}
    for index in range(1, 4):
        pair = {}
        for mode in ("on", "off"):
            pair[mode] = replay(cap, out/f"{mode}_{index}", mode == "on", args)
        evidence["rounds"].append(pair)
    path = out/"replay_evidence.json"
    dump(path, evidence)
    if split == "calibration":
        summary = score.score_capture(path)
        dump(out/"report.json", summary)
        score.freeze([path], args.thresholds, plot=out/"comparison.png")
        companion = args.thresholds.with_name(args.thresholds.name + ".identity.json")
        identity = {"software": setup["software"], "assets": assets,
                    "thresholds_sha256": digest(args.thresholds)}
        if companion.exists():
            if json.loads(companion.read_text()) != identity:
                raise ValueError("pre-existing frozen calibration identity differs")
        else:
            dump(companion, identity)
        return
    online = dict(evidence, split="online", rounds=[{"on": run_evidence(cap/"online", cap),
                                                     "off": evidence["rounds"][0]["off"]}])
    live_path = out/"online_evidence.json"
    dump(live_path, online)
    report = score.evaluate([path, live_path], args.thresholds, out/"report.json", out/"comparison.png")
    if not report["passed"]:
        raise ValueError(f"blind experiment did not pass; see {out/'report.json'}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="command", required=True)
    record = modes.add_parser("capture", help="attach to running native powerplant SITL")
    record.add_argument("--split", required=True, choices=("calibration", "blind"))
    record.add_argument("--px4-root", type=Path, default=os.environ.get("PX4_ROOT"),
                        required="PX4_ROOT" not in os.environ)
    record.add_argument("--prepare-only", action="store_true")
    record.add_argument("--thresholds", type=Path)
    record.add_argument("--output-dir", type=Path, required=True)
    for mode in ("calibrate", "verify"):
        sub = modes.add_parser(mode)
        sub.add_argument("--capture", type=Path, required=True)
        sub.add_argument("--thresholds", type=Path, required=True)
        sub.add_argument("--output-dir", type=Path, required=True)
        sub.add_argument("--replay-domain", type=int, choices=range(0, 233), metavar="ID", default=112)
        sub.add_argument("--rate", type=float, default=1.)
    args = parser.parse_args(argv)
    for key, value in vars(args).items():
        if isinstance(value, Path):
            setattr(args, key, value.resolve())
    new_output = not args.output_dir.exists() or (
        args.output_dir.is_dir() and not any(args.output_dir.iterdir()))
    try:
        if args.command == "capture":
            capture(args)
        elif args.rate != 1.0:
            raise ValueError("formal calibration and verification require original speed (--rate 1)")
        else:
            grade(args)
    except (ValueError, RuntimeError, OSError, KeyError, subprocess.CalledProcessError) as exc:
        print(str(exc), file=sys.stderr)
        if new_output and args.output_dir.is_dir():
            failure = args.output_dir/"failure.json"
            if not failure.exists():
                dump(failure, {"passed": False, "stage": args.command, "reason": str(exc)})
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
