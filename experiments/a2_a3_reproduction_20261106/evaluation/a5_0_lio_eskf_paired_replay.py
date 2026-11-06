#!/usr/bin/env python3
"""A5-0: causal 10 Hz LIO -> SAME StreamingZengESKF contract, OFFLINE gate.

This is a diagnostic of the missing UAV-pose input leg, NOT an Isaac/MAPPO
closed loop. A4-3a LIO is from a fixed pre-recorded PhysX trajectory.
Each LIO's world frame is aligned using the FIRST paired PhysX GT sample
(oracle alignment, not autonomous multi-agent localization).

ESKF uses original A4-2B pose-derived virtual body IMU and collective thrust
from that same fixed trajectory. It never uses load GT. Only the LIO UAV pose
changes. Angular/linear UAV state placeholders are zeros because the currently
audited ESKF implementation consumes only [position, orientation] from
uav_com_state_w_xyzw; hard-fail if the pinned source implementation changes.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

BASE=Path.home()/"research"
def rmsnorm(v):
    v=np.asarray(v,float)
    return float(np.sqrt(np.mean(np.sum(v*v,axis=-1))))

def load_physx(root):
    with np.load(root/"a1_47_mappo_eskf_500steps_300hz.npz",allow_pickle=False) as z:
        t=np.asarray(z["time_s"],float)
        names=[str(n) for n in z["body_names"]]
        state=np.asarray(z["body_link_pose_w_xyzw_300hz"],float)
    ids=[names.index(f"Falcon{i}_base_link_inertia") for i in (1,2,3)]
    return t,state[:,ids,:]

def sample_physx(tq,t,physx_one):
    p=np.column_stack([np.interp(tq,t,physx_one[:,j]) for j in range(3)])
    R=Slerp(t,Rotation.from_quat(physx_one[:,3:7]))(tq)
    return p,R

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--root",type=Path,default=BASE/"isaac_a1_diagnostics")
    parser.add_argument("--sun-root",type=Path,default=BASE/"payload_ekf_sun_E1")
    parser.add_argument("--out-dir",type=Path,default=None)
    parser.add_argument("--max-age-s",type=float,default=0.105)
    parser.add_argument("--pose-stride",type=int,default=1,
                        help="Kabsch update every input sample; this is 10 Hz, not E5's 100 Hz.")
    args=parser.parse_args()
    root=args.root.expanduser()
    sun=args.sun_root.expanduser()
    out=args.out_dir.expanduser() if args.out_dir else root/"A5_0/lio_eskf_paired_replay"
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"Output not empty, refuse overwrite: {out}")
    if args.max_age_s<0.001 or args.max_age_s>0.2:
        raise ValueError("Unexpected maximum age")
    if args.pose_stride<1: raise ValueError("Invalid stride")

    # Fail closed if the audited ESKF contract changes, since this adapter
    # supplies zeros in velocity/rate slots (these must not be consumed).
    source=sun/"src/payload_ekf_sun/streaming_zeng_eskf.py"
    s=source.read_text()
    expected="contacts = u[:, :3] + np.einsum("
    expected_q="R_uav = quat_xyzw_to_rot(u[:, 3:7])"
    if s.count(expected)!=1 or s.count(expected_q)!=1:
        raise RuntimeError("ESKF UAV position/orientation contract changed; re-audit source before proceeding")
    sys.path.insert(0,str(sun/"src"))
    from payload_ekf_sun.streaming_zeng_eskf import StreamingZengESKF

    with np.load(root/"A4_2B/paired_collective_thrust_A3_B2/a4_2b_paired_10hz_inputs.npz",allow_pickle=False) as z:
        grid=np.asarray(z["time_s"],float)
        uav_gt_p=np.asarray(z["uav_position_gt_W_m"],float)
        sf=np.asarray(z["imu_specific_force_B_mps2"],float)
        total=np.asarray(z["rotor_collective_thrust_N"],float)
        q_gt=np.asarray(z["uav_quat_gt_WB_xyzw"],float)
    if (grid.shape!=(46,) or sf.shape!=(46,3,3) or total.shape!=(46,3)
        or uav_gt_p.shape!=(46,3,3) or q_gt.shape!=(46,3,4)):
        raise ValueError("Unexpected A4-2B input shapes")
    if not np.allclose(grid,np.arange(4,50)*0.1,atol=1e-7):
        raise ValueError("Unexpected A4-2B timestamps")
    t_physx,poses=load_physx(root)
    # Verify same PhysX reference and identical GT layout as input archive.
    gt_grid=np.stack([sample_physx(grid,t_physx,poses[:,i,:])[0] for i in range(3)],axis=1)
    if np.max(np.abs(gt_grid-uav_gt_p))>1e-5:
        raise RuntimeError("A4-2B GT does not match PhysX reference: wrong scenario")

    p_aligned=np.zeros((len(grid),3,3),float)
    q_aligned=np.zeros((len(grid),3,4),float)
    cov_aligned=np.zeros((len(grid),3,3,3),float)
    ages=np.zeros((len(grid),3),float)
    anchor_details=[]
    for i in range(3):
        path=root/f"A4_3A/fastlio_covariance_replay/drone_{i+1}_odom_cov.npz"
        with np.load(path,allow_pickle=False) as z:
            od=np.asarray(z["odom_time_pose_xyzw"],float)
            P=np.asarray(z["odom_pose_covariance_6x6"],float)
        ts=od[:,0]
        if od.shape!=(47,8) or P.shape!=(47,6,6) or not np.isfinite(od).all() or not np.isfinite(P).all():
            raise ValueError(f"Invalid drone{i+1} pose/covariance samples")
        if not np.all(np.diff(ts)>0):raise ValueError("LIO samples not strictly chronological")
        # Match first available LIO sample with PhysX at its exact timestamp.
        p0,R0=sample_physx(np.array([ts[0]]),t_physx,poses[:,i,:])
        R_lio0=Rotation.from_quat(od[0,4:8])
        Ralign=R0[0]*R_lio0.inv()
        A=Ralign.as_matrix()
        shift=p0[0]-A@od[0,1:4]
        # Causal ZOH: LIO observation must have occurred at or before grid[k].
        index=np.searchsorted(ts,grid,side="right")-1
        if np.min(index)<0:
            raise RuntimeError(f"Drone{i+1}: no available LIO for some early ticks")
        age=grid-ts[index]
        if np.min(age)<-1e-9 or np.max(age)>args.max_age_s:
            raise RuntimeError(f"Drone{i+1}: stale/future LIO: age range [{age.min()},{age.max()}] s")
        ages[:,i]=age
        p_aligned[:,i,:]=(A@od[index,1:4].T).T+shift
        q_aligned[:,i,:]=(Ralign*Rotation.from_quat(od[index,4:8])).as_quat()
        Cp=P[index,:3,:3]
        cov_aligned[:,i,:,:]=np.einsum("ab,nbc,dc->nad",A,Cp,A)
        anchor_details.append({
            "drone":i+1,"anchor_time_s":float(ts[0]),
            "anchor_uses_PhysX_oracle":True,
            "max_sample_age_s":float(np.max(age)),
            "translation_W_m":shift.tolist(),
            "rotation_matrix":A.tolist()})
    if np.min(np.linalg.eigvalsh(cov_aligned)) <= 0:
        raise RuntimeError("Nonpositive input position covariance")

    # Existing 13-vector layout: p(3), quaternion xyzw(4), v(3), omega(3).
    # ESKF implementation above uses only position and quaternion; placeholders
    # never represent a real sensed velocity or angular rate.
    u=np.zeros((len(grid),3,13),float)
    u[:,:,:3]=p_aligned
    u[:,:,3:7]=q_aligned
    unused_acc=np.zeros((3,6),float)
    eskf=StreamingZengESKF(pose_stride=args.pose_stride)
    estimate_p=[];estimate_R=[];estimate_v=[];estimate_w=[];estimate_fit=[]
    for k,t in enumerate(grid):
        # Only aggregate thrust is available in the paired A4-2B archive.
        # Even 4-way split is an adapter for sum(axis=1), not per-rotor ground truth.
        rotor=np.repeat((total[k]/4)[:,None],4,axis=1)
        result=eskf.step(float(t),u[k],unused_acc,rotor,specific_force_B=sf[k])
        estimate_p.append(result["p_W"])
        estimate_R.append(result["R_L_W"])
        estimate_v.append(result["v_W"])
        estimate_w.append(result["omega_L"])
        estimate_fit.append(result["kabsch_fit_m"])
    estimate_p=np.asarray(estimate_p);estimate_R=np.asarray(estimate_R)
    estimate_v=np.asarray(estimate_v);estimate_w=np.asarray(estimate_w)
    if not all(np.isfinite(x).all() for x in [estimate_p,estimate_R,estimate_v,estimate_w]):
        raise RuntimeError("Nonfinite ESKF estimates")
    print("A5-0 LIO -> EXISTING StreamingZengESKF OFFLINE REPLAY")
    print("frames=46, 10Hz; THREE live-replay LIO traces, FIRST-POSE PhysX-oracle aligned")
    print("Per-UAV LIO sample age max (s):",np.max(ages,axis=0))
    print("UAV pos vs PhysX RMS by drone (m):",
          [rmsnorm(p_aligned[:,i,:]-uav_gt_p[:,i,:]) for i in range(3)])
    print("Kabsch fit RMS (m):",float(np.sqrt(np.mean(np.asarray(estimate_fit)**2))))
    print("ESKF final position (world m):",estimate_p[-1])
    print("NOTE: did NOT execute MAPPO/Isaac closed loop. A3/A4 fixed replay only.")
    print("NOTE: no payload GT was passed to ESKF; IMU and rotor total use paired ideal A3/E7 records.")
    print("NOTE: LIO coordinates use PhysX ORACLE alignment: NOT MoCap-free.")
    out.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(out/"a5_0_lio_eskf_inputs_and_outputs.npz",
        time_s=grid,uav_lio_aligned_w_xyzw_13=u,
        uav_cov_position_aligned_W_m2=cov_aligned,
        uav_lio_sample_age_s=ages,
        source_imu_specific_force_B_mps2=sf,
        source_collective_thrust_N=total,
        estimate_load_position_W_m=estimate_p,
        estimate_load_rotation_W=estimate_R,
        estimate_load_velocity_W_mps=estimate_v,
        estimate_load_omega_body_radps=estimate_w,
        estimate_kabsch_fit_m=np.asarray(estimate_fit))
    (out/"a5_0_summary.json").write_text(json.dumps({
        "verdict":"PASS: paired LIO/ideal-IMU/thrust fed to existing 10Hz ESKF" ,
        "claim_limit":"OFFLINE input chain; NOT true LIO+MAPPO closed loop; NO autonomy",
        "covariance_used_in_eskf":False,
        "uav_velocity_fields_are_zero_unused_placeholders":True,
        "uav_acceleration_fields_are_zero_because_specific_force_B_provided":True,
        "PhysX_oracle_alignment":True,
        "uav_lio_age_max_s":np.max(ages,axis=0).tolist(),
        "LIO_position_RMSE_by_uav_m":[rmsnorm(p_aligned[:,i,:]-uav_gt_p[:,i,:]) for i in range(3)],
        "ESKF_input_source_sha256":hashlib.sha256(source.read_bytes()).hexdigest(),
        "anchor_details":anchor_details,
    },ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("Saved:",out)

if __name__=="__main__":
    main()
