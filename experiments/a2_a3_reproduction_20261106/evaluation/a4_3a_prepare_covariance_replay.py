#!/usr/bin/env python3
"""Create an independent A4-3a ROS2 replay script from the user's validated A3-B2 script.

Source remains untouched. Makes only tightly validated string substitutions.
The generated script saves covariance but does not treat it as calibrated.
"""
from pathlib import Path
import argparse,sys

DEFAULT=Path.home()/"research/fastlio2_b2_ws/a3_b2_replay_three_uav.py"

def replace_once(text, old, new, description):
    n=text.count(old)
    if n!=1:
        raise RuntimeError(f"{description}: expected one exact anchor, found {n}. Original script not modified.")
    return text.replace(old,new,1)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--source",type=Path,default=DEFAULT)
    ap.add_argument("--dest",type=Path,default=None)
    args=ap.parse_args()
    original=args.source.expanduser().resolve()
    dest=(args.dest.expanduser() if args.dest else original.with_name("a4_3a_replay_three_uav_cov.py"))
    if not original.is_file():
        raise FileNotFoundError(original)
    if dest.exists():
        raise FileExistsError(f"Refusing to overwrite: {dest}")
    s=original.read_text(encoding="utf-8")
    s=replace_once(s,
       '    output_dir = root / "A3_1/A3_B2_gyro_zero"\n    output_dir.mkdir(exist_ok=True)',
       '    output_dir = root / "A4_3A/fastlio_covariance_replay"\n    output_dir.mkdir(parents=True, exist_ok=True)',
       'output directory')
    s=replace_once(s,
       '    received = []\n\n    def on_odom(msg):',
       '    received = []\n    received_cov = []\n\n    def on_odom(msg):',
       'callback storage')
    s=replace_once(s,
       '        received.append(\n            (t, p.x, p.y, p.z, q.x, q.y, q.z, q.w)\n        )',
       '        received.append(\n            (t, p.x, p.y, p.z, q.x, q.y, q.z, q.w)\n        )\n        received_cov.append(\n            np.asarray(msg.pose.covariance, dtype=np.float64).reshape(6, 6).copy()\n        )',
       'append covariance')
    s=replace_once(s,
       '            f"a3_b2_drone_{args.drone}_odom.npz"',
       '            f"drone_{args.drone}_odom_cov.npz"',
       'new output filename')
    s=replace_once(s,
       '        np.savez_compressed(\n            output,\n            odom_time_pose_xyzw=odom,',
       '        cov = np.asarray(received_cov, dtype=np.float64).reshape(-1, 6, 6)\n'
       '        assert len(odom) == len(cov), "Pose/covariance sample count mismatch"\n'
       '        if len(cov):\n'
       '            assert np.isfinite(cov).all(), "Nonfinite covariance"\n'
       '            symmetry_error = float(np.max(np.abs(cov - cov.transpose(0, 2, 1))))\n'
       '            eig_min = float(np.min(np.linalg.eigvalsh(0.5 * (cov + cov.transpose(0, 2, 1)))))\n'
       '            nonzero = bool(np.any(cov != 0))\n'
       '            print(f"Covariance audit: samples={len(cov)}, max_asymmetry={symmetry_error:.3e}, min_eig={eig_min:.3e}, nonzero={nonzero}", flush=True)\n'
       '        else:\n'
       '            print("WARNING: no odometry/covariance received", flush=True)\n'
       '        np.savez_compressed(\n'
       '            output,\n'
       '            odom_time_pose_xyzw=odom,\n'
       '            odom_pose_covariance_6x6=cov,',
       'persist covariance')
    s=replace_once(s,
       '                "offline replay only"',
       '                "offline replay only; A4-3a IKFoM tangent covariance diagnostic; uncalibrated"',
       'provenance')
    dest.write_text(s,encoding="utf-8")
    dest.chmod(original.stat().st_mode)
    print("CREATED:",dest)
    print("INPUT UNCHANGED:",original)
    print("OUTPUT ROOT: ~/research/isaac_a1_diagnostics/A4_3A/fastlio_covariance_replay")
    print("Requires a running fast_lio node with the matching drone_id remappings.")

if __name__=="__main__":
    try: main()
    except Exception as e:
        print(f"FAIL_CLOSED: {e}",file=sys.stderr)
        sys.exit(1)
