#!/usr/bin/env python3
"""A3-B1 vs A3-B2 Falcon2 offline first-pose-aligned evaluation.

Source data: original 5s PhysX GT and original B1/B2 odometry; never overwrite.
No simulated truth is used during FAST-LIO2 replay, only after for evaluation.
"""
from pathlib import Path
import argparse
import json
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

def load(root, drone, mode):
    if mode == "B1":
        path = root / "A3_1/A3_B1_three_plane" / f"a3_b1_drone_{drone}_odom.npz"
    else:
        path = root / "A3_1/A3_B2_gyro_zero" / f"a3_b2_drone_{drone}_odom.npz"
    with np.load(path, allow_pickle=False) as d:
        odom = np.asarray(d["odom_time_pose_xyzw"], dtype=float)
    if odom.ndim != 2 or odom.shape[1] != 8 or len(odom) < 2:
        raise ValueError(f"Empty/invalid odometry: {path}")
    if not np.isfinite(odom).all() or not np.all(np.diff(odom[:,0]) > 0):
        raise ValueError(f"Nonfinite/unsorted odometry: {path}")
    return odom, path

def errors(t, poses, gt_t, gt_pose):
    p = poses[:,1:4]
    R = Rotation.from_quat(poses[:,4:8])
    pg = np.column_stack([np.interp(t,gt_t,gt_pose[:,j]) for j in range(3)])
    Rg = Slerp(gt_t,Rotation.from_quat(gt_pose[:,3:7]))(t)
    Ralign = Rg[0] * R[0].inv()
    pa = pg[0]+Ralign.apply(p-p[0])
    Ra = Ralign*R
    ep = pa-pg
    er = np.rad2deg((Ra.inv()*Rg).magnitude())
    return ep, er

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--root",type=Path,default=Path.home()/"research/isaac_a1_diagnostics")
    parser.add_argument("--drone",type=int,default=2,choices=(1,2,3))
    parser.add_argument("--out",type=Path,default=None)
    args=parser.parse_args()
    root=args.root.expanduser()
    with np.load(root/"a1_47_mappo_eskf_500steps_300hz.npz",allow_pickle=False) as d:
        gt_t=np.asarray(d["time_s"],dtype=float)
        names=[str(x) for x in d["body_names"]]
        gt_pose=np.asarray(d["body_link_pose_w_xyzw_300hz"][:,names.index(f"Falcon{args.drone}_base_link_inertia")],dtype=float)
    modes={}
    for mode in ("B1","B2"):
        odom,path=load(root,args.drone,mode)
        t=odom[:,0]
        if t[0] < gt_t[0] or t[-1] > gt_t[-1]:
            raise ValueError(f"{mode} odometry outside truth time domain")
        ep,er=errors(t,odom,gt_t,gt_pose)
        modes[mode]={"t":t,"odom":odom,"path":str(path),"e":ep,"rot_e":er}
        print(f"{mode} {len(t)} samples [{t[0]:.6f},{t[-1]:.6f}] s; "
              f"pos RMSE={np.sqrt(np.mean(np.sum(ep**2,axis=1))):.6f} m; "
              f"rot RMSE={np.sqrt(np.mean(er**2)):.6f} deg; "
              f"final={np.linalg.norm(ep[-1]):.6f} m")
    # Five-point metric identical to A3 audit: first-pose alignment on each mode's
    # native initial odometry, then linearly interpolate aligned error vector on common times.
    grid=np.linspace(0.4,0.8,5)
    for mode,v in modes.items():
        t=v["t"]
        if grid[0] < t[0] or grid[-1] > t[-1]:
            raise ValueError(f"Five-point grid outside {mode} times; use actual overlap")
        # Interpolate aligned error vectors; acceptable as diagnostic and explicitly stated.
        ep=np.column_stack([np.interp(grid,t,v["e"][:,j]) for j in range(3)])
        er=np.interp(grid,t,v["rot_e"])
        v["five_pos"]=float(np.sqrt(np.mean(np.sum(ep**2,axis=1))))
        v["five_rot"]=float(np.sqrt(np.mean(er**2)))
        print(f"{mode} early 5-point pos={v['five_pos']:.6f} m rot={v['five_rot']:.6f} deg")
        print(f"  at 0.4..0.8s errors (m):", np.round(np.linalg.norm(ep,axis=1),5))
    for typ in ("pos","rot"):
        if typ=="pos":
            b1=float(np.sqrt(np.mean(np.sum(modes["B1"]["e"]**2,axis=1))))
            b2=float(np.sqrt(np.mean(np.sum(modes["B2"]["e"]**2,axis=1))))
        else:
            b1=float(np.sqrt(np.mean(modes["B1"]["rot_e"]**2)))
            b2=float(np.sqrt(np.mean(modes["B2"]["rot_e"]**2)))
        print(f"{typ} full improvement: {(1-b2/b1)*100:.2f}%")
    report={"method":"separate first-sample pose alignment for each run; GT only for offline evaluation; five-point linearly interpolates aligned errors",
            "drone":args.drone, "common_five_times_s":grid.tolist(),"results":{}}
    for mode,v in modes.items():
        e=v["e"]; rot=v["rot_e"]
        report["results"][mode]={"odom_path":v["path"],"samples":len(v["t"]),
          "time_start_s":float(v["t"][0]),"time_end_s":float(v["t"][-1]),
          "full_pos_rmse_m":float(np.sqrt(np.mean(np.sum(e**2,axis=1)))),
          "full_rot_rmse_deg":float(np.sqrt(np.mean(rot**2))),
          "final_pos_error_m":float(np.linalg.norm(e[-1])),
          "five_pos_rmse_m":v["five_pos"],"five_rot_rmse_deg":v["five_rot"]}
    out=args.out or root/"A3_1/A3_B2_gyro_zero"/f"a3_b1_vs_b2_falcon{args.drone}_comparison.json"
    if out.exists():
        raise FileExistsError(f"Refusing to overwrite {out}")
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2)+"\n")
    print("Saved comparison:",out)

if __name__=="__main__":
    main()
