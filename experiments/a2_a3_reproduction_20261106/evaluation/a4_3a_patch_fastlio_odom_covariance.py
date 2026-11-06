#!/usr/bin/env python3
"""Safely patch local FAST_LIO_ROS2 laserMapping.cpp Odometry covariance publication.

This intentionally changes only two things:
1. publish Odometry AFTER the current-frame covariance is written.
2. map the 23D IKFoM pos(0:3), rot(3:6) error covariance into the
   [position(3), rotation(3)] message field in the same native tangent
   convention.

DO NOT treat the exported orientation covariance as fully validated ROS
world-frame rotation uncertainty until frame/tangent verification. Do not
treat internal IKFoM covariance as calibrated localization uncertainty.
"""
from __future__ import annotations
import argparse
import datetime
import difflib
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile

DEFAULT = Path.home() / "research/fastlio2_b2_ws/src/FAST_LIO_ROS2/src/laserMapping.cpp"

OLD = """    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }"""

NEW = """    // A4-3a: populate CURRENT-frame pose covariance BEFORE publishing.
    // IKFoM state_ikfom starts with pos[0:3], rot[3:6].
    // This exports the native IKFoM small-angle tangent covariance.
    // Verify rotation tangent-frame convention before using as a calibrated
    // ROS Odometry orientation covariance or a payload-ESKF input.
    const auto& P = kf.get_P();
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            odomAftMapped.pose.covariance[i * 6 + j] = P(i, j);
        }
    }
    pubOdomAftMapped->publish(odomAftMapped);"""

def patch(path: Path, dry_run: bool):
    if not path.is_file():
        raise FileNotFoundError(f"Missing C++ source: {path}")
    src = path.read_text(encoding="utf-8")
    # The line directly before the original block is set_posestamp(odomAftMapped.pose);
    # and the code directly after must be the original TF output section.
    boundary_start = "void publish_odometry("
    boundary_end = "void publish_path("
    if src.count(boundary_start) != 1 or src.count(boundary_end) != 1:
        raise RuntimeError("Unexpected publish_odometry/publish_path layout; refusing patch")
    start = src.index(boundary_start)
    end = src.index(boundary_end, start)
    func = src[start:end]
    if NEW in func:
        print("ALREADY_PATCHED: publish_odometry() already contains A4-3a correction")
        return
    if func.count(OLD) != 1:
        raise RuntimeError(
            "Original publication/covariance block differs from audited lines 635-646; "
            "refusing blind patch. Inspect sed -n '625,665p' laserMapping.cpp")
    if "set_posestamp(odomAftMapped.pose);" not in func or "tf_br->sendTransform(trans);" not in func:
        raise RuntimeError("Odometry frame or TF publishing is different; refusing patch")
    new_func = func.replace(OLD, NEW, 1)
    assert new_func.count('pubOdomAftMapped->publish(odomAftMapped);') == 1
    assert new_func.index('odomAftMapped.pose.covariance[i * 6 + j]') < new_func.index('pubOdomAftMapped->publish(odomAftMapped);')
    revised = src[:start] + new_func + src[end:]
    assert revised.replace(new_func,func,1) == src
    diff = ''.join(difflib.unified_diff(
        src.splitlines(keepends=True), revised.splitlines(keepends=True),
        fromfile=str(path)+" (BEFORE)", tofile=str(path)+" (AFTER)"))
    print(diff)
    if dry_run:
        print("DRY_RUN: no files written.")
        return

    backup = path.with_name(path.name + ".bak_a4_3a")
    if backup.exists():
        # Never overwrite an existing backup, and require it represent the
        # source presently patched so that rollback remains unambiguous.
        if backup.read_bytes() != path.read_bytes():
            raise FileExistsError(
                f"Backup exists but differs from current file: {backup}; "
                "refusing change. Inspect both files manually.")
    else:
        shutil.copy2(path, backup)
    fd, tempname = tempfile.mkstemp(prefix=".laserMapping.a4_3a.",suffix=".cpp",dir=path.parent)
    try:
        with os.fdopen(fd,"w",encoding="utf-8",newline="") as f:
            f.write(revised)
            f.flush()
            os.fsync(f.fileno())
        shutil.copymode(path,tempname)
        os.replace(tempname,path)
    finally:
        if os.path.exists(tempname):
            os.unlink(tempname)
    print(f"PATCHED: {path}")
    print(f"BACKUP:  {backup}")
    print("NEXT: review C++ diff, build ROS2, and validate output timestamps and covariance.")

if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--file",type=Path,default=DEFAULT)
    ap.add_argument("--dry-run",action="store_true")
    args=ap.parse_args()
    try:
        patch(args.file.expanduser(),args.dry_run)
    except Exception as exc:
        print(f"FAIL_CLOSED: {exc}",file=sys.stderr)
        sys.exit(1)
