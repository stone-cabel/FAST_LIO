"""A1-3h-2: Zeng seven-body dynamic geometry and nearest-hit validation."""

from isaaclab.app import AppLauncher

simulation_app = AppLauncher(headless=True).app

try:
    from pathlib import Path

    import gymnasium as gym
    import numpy as np
    import torch
    import trimesh
    import warp as wp
    import omni.usd

    from pxr import Gf, UsdGeom
    from scipy.spatial.transform import Rotation

    import MARL_mav_carry_ext.tasks

    from isaaclab_tasks.utils import parse_env_cfg
    from isaaclab.utils.warp.ops import (
        convert_to_warp_mesh,
        raycast_dynamic_meshes,
    )

    ROOT = Path.home() / "research/isaac_a1_diagnostics"
    DEVICE = "cuda:0"
    TASK = "Isaac-flycrane-payload-decentralized-hovering-v0"
    BASE = "/World/envs/env_0/flycrane"

    # Use collision geometry only.
    # Do not mix visual and collision surfaces.
    TARGETS = [
        (f"Falcon{i}_base_link_inertia", "collisions/mesh_0")
        for i in range(1, 4)
    ] + [
        ("load_link", "collisions/collision"),
    ] + [
        (f"rope_{i}_link", "collisions/mesh_0")
        for i in range(1, 4)
    ]

    # --------------------------------------------------
    # 1. Extract geometry in its own local coordinates
    # --------------------------------------------------

    def primitive_mesh(prim):

        if prim.IsA(UsdGeom.Mesh):

            geom = UsdGeom.Mesh(prim)

            vertices = np.array(
                geom.GetPointsAttr().Get(),
                dtype=np.float64,
            )

            counts = list(
                map(int, geom.GetFaceVertexCountsAttr().Get())
            )

            indices = list(
                map(int, geom.GetFaceVertexIndicesAttr().Get())
            )

            faces = []
            offset = 0

            for count in counts:

                face = indices[offset:offset + count]
                offset += count

                for j in range(1, count - 1):
                    faces.append(
                        (face[0], face[j], face[j + 1])
                    )

            if offset != len(indices) or not faces:
                raise ValueError(
                    f"Unexpected mesh topology: {prim.GetPath()}"
                )

            return vertices, np.array(
                faces, dtype=np.int32
            )

        if prim.IsA(UsdGeom.Cube):

            edge = float(
                UsdGeom.Cube(prim).GetSizeAttr().Get()
            )

            mesh = trimesh.creation.box(
                extents=(edge, edge, edge)
            )

            return (
                np.array(mesh.vertices, dtype=np.float64),
                np.array(mesh.faces, dtype=np.int32),
            )

        if prim.IsA(UsdGeom.Cylinder):

            g = UsdGeom.Cylinder(prim)

            mesh = trimesh.creation.cylinder(
                radius=float(g.GetRadiusAttr().Get()),
                height=float(g.GetHeightAttr().Get()),
                sections=32,
            )

            vertices = np.array(
                mesh.vertices, dtype=np.float64
            )

            axis = str(
                g.GetAxisAttr().Get() or "Z"
            ).upper()

            if axis == "X":
                vertices = Rotation.from_euler(
                    "y", 90, degrees=True
                ).apply(vertices)

            elif axis == "Y":
                vertices = Rotation.from_euler(
                    "x", -90, degrees=True
                ).apply(vertices)

            elif axis != "Z":
                raise ValueError(
                    f"Unsupported cylinder axis: {axis}"
                )

            return (
                vertices,
                np.array(mesh.faces, dtype=np.int32),
            )

        raise TypeError(
            f"Unsupported geometry: "
            f"{prim.GetPath()} {prim.GetTypeName()}"
        )

    # --------------------------------------------------
    # 2. Start Zeng environment
    # --------------------------------------------------

    cfg = parse_env_cfg(
        TASK,
        device=DEVICE,
        num_envs=1,
    )

    cfg.seed = 0

    env = gym.make(TASK, cfg=cfg)

    try:
        env.reset()

        raw = env.unwrapped
        robot = raw.robot

        stage = omni.usd.get_context().get_stage()

        indices, names = robot.find_bodies(".*")

        body_lookup = dict(
            zip(names, map(int, indices))
        )

        cache = UsdGeom.XformCache()

        meshes = []
        bounds = []
        body_ids = []

        print(
            "\n===== A1-3h-2: Zeng collision geometry =====",
            flush=True,
        )

        # --------------------------------------------------
        # 3. Build seven body-local Warp meshes
        # --------------------------------------------------

        for body_name, geometry_suffix in TARGETS:

            if body_name not in body_lookup:
                raise KeyError(
                    f"PhysX body missing: {body_name}; "
                    f"available={list(body_lookup)}"
                )

            body_path = f"{BASE}/{body_name}"

            body = stage.GetPrimAtPath(body_path)

            geom = stage.GetPrimAtPath(
                f"{body_path}/{geometry_suffix}"
            )

            if not body.IsValid() or not geom.IsValid():
                raise RuntimeError(
                    f"USD geometry missing: "
                    f"{body_path}/{geometry_suffix}"
                )

            local_v, triangles = primitive_mesh(geom)

            to_body = (
                cache.GetLocalToWorldTransform(body)
                .GetInverse()
            )

            to_world = (
                cache.GetLocalToWorldTransform(geom)
            )

            verts_link = np.array(
                [
                    list(
                        to_body.Transform(
                            to_world.Transform(
                                Gf.Vec3d(*map(float, v))
                            )
                        )
                    )
                    for v in local_v
                ],
                dtype=np.float32,
            )

            assert np.isfinite(verts_link).all()

            mesh = convert_to_warp_mesh(
                verts_link,
                triangles,
                device=DEVICE,
            )

            meshes.append(mesh)

            bounds.append(
                (
                    verts_link.min(axis=0),
                    verts_link.max(axis=0),
                )
            )

            body_ids.append(
                body_lookup[body_name]
            )

            print(
                f"{body_name:27s} "
                f"{geom.GetTypeName():8s} "
                f"vertices={len(local_v):5d} "
                f"triangles={len(triangles):5d} "
                f"physx_idx={body_lookup[body_name]}",
                flush=True,
            )

        mesh_ids = wp.array2d(
            [[m.id for m in meshes]],
            dtype=wp.uint64,
            device=DEVICE,
        )

        # --------------------------------------------------
        # 4. Read seven PhysX body poses
        # --------------------------------------------------

        def read_poses():

            return (
                robot.data.body_link_pose_w.torch[
                    0, body_ids, :7
                ]
                .detach()
                .cpu()
                .numpy()
                .copy()
                .astype(np.float64)
            )


        # --------------------------------------------------
        # A1-3h-4a: Visual geometry and PhysX body mapping
        # --------------------------------------------------

        import json
        from collections import Counter
        from pxr import Usd

        visual_records = []
        geometry_types = Counter()

        for prim in Usd.PrimRange(
            stage.GetPseudoRoot(),
            Usd.TraverseInstanceProxies(),
        ):

            if not (
                prim.IsA(UsdGeom.Mesh)
                or prim.IsA(UsdGeom.Cube)
                or prim.IsA(UsdGeom.Cylinder)
            ):
                continue

            path = str(prim.GetPath())

            if not path.startswith(BASE + "/"):
                continue

            if "/visuals/" not in path:
                continue

            geometry_types[prim.GetTypeName()] += 1

            candidates = [
                name
                for name in body_lookup
                if path.startswith(
                    BASE + "/" + name + "/"
                )
            ]

            owner = (
                max(candidates, key=len)
                if candidates
                else None
            )

            body_index = (
                body_lookup[owner]
                if owner is not None
                else None
            )

            record = {
                "usd_path": path,
                "geometry_type": prim.GetTypeName(),
                "physx_body": owner,
                "physx_body_index": body_index,
                "instance_proxy": bool(
                    prim.IsInstanceProxy()
                ),
            }

            visual_records.append(record)

        print(
            "\\n===== A1-3h-4a VISUAL GEOMETRY =====",
            flush=True,
        )

        print(
            "Visual primitive count:",
            len(visual_records),
            flush=True,
        )

        print(
            "Geometry types:",
            dict(geometry_types),
            flush=True,
        )

        unmapped = []

        for record in visual_records:

            name = record["physx_body"]
            index = record["physx_body_index"]

            print(
                f"{record['geometry_type']:9s} "
                f"body={str(name):35s} "
                f"index={str(index):4s} "
                f"{record['usd_path']}",
                flush=True,
            )

            if name is None:
                unmapped.append(record["usd_path"])

        output = (
            ROOT / "a1_zeng_visual_geometry_audit.json"
        )

        output.write_text(
            json.dumps(
                {
                    "visual_count": len(visual_records),
                    "geometry_types": dict(geometry_types),
                    "visuals": visual_records,
                    "unmapped": unmapped,
                },
                indent=2,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )

        assert len(visual_records) > 0, (
            "No visual geometry found"
        )

        required = [name for name, _ in TARGETS]

        missing = [
            name
            for name in required
            if not any(
                record["physx_body"] == name
                for record in visual_records
            )
        ]

        print(
            "\\nRequired body visual meshes missing:",
            missing,
            flush=True,
        )

        print(
            "Unmapped geometry count:",
            len(unmapped),
            flush=True,
        )

        print(
            "Saved:",
            output,
            flush=True,
        )

        if missing or unmapped:
            print(
                "[A1-3h-4a] AUDIT COMPLETE: "
                "MAPPING NEEDS REVIEW",
                flush=True,
            )
        else:
            print(
                "[A1-3h-4a] VISUAL BODY "
                "MAPPING CHECK PASSED",
                flush=True,
            )


        # --------------------------------------------------
        # A1-3h-4b-1:
        # 19 visual geometries + PhysX + Warp
        # --------------------------------------------------

        print(
            "\n===== A1-3h-4b-1 VISUAL WARP TEST =====",
            flush=True,
        )

        assert not unmapped
        assert not missing

        visual_meshes = []
        visual_body_ids = []
        visual_names = []
        test_origins_local = []
        test_directions_local = []

        # 1. Convert each visual geometry into its PhysX
        #    link-local frame.
        for record in visual_records:

            path = record["usd_path"]
            name = record["physx_body"]

            geom_prim = stage.GetPrimAtPath(path)

            body_prim = stage.GetPrimAtPath(
                BASE + "/" + name
            )

            vertices, faces = primitive_mesh(geom_prim)

            body_to_world = (
                cache.GetLocalToWorldTransform(body_prim)
            )

            geom_to_world = (
                cache.GetLocalToWorldTransform(geom_prim)
            )

            world_to_body = body_to_world.GetInverse()

            verts_link = np.array(
                [
                    list(
                        world_to_body.Transform(
                            geom_to_world.Transform(
                                Gf.Vec3d(*map(float, v))
                            )
                        )
                    )
                    for v in vertices
                ],
                dtype=np.float32,
            )

            assert np.isfinite(verts_link).all()

            assert len(verts_link) > 0
            assert len(faces) > 0

            warp_mesh = convert_to_warp_mesh(
                verts_link,
                faces,
                device=DEVICE,
            )

            visual_meshes.append(warp_mesh)

            visual_body_ids.append(
                int(record["physx_body_index"])
            )

            visual_names.append(name)

            # 2. Create two rays targeting a real triangle
            #    on this mesh, from opposite sides.
            tri = verts_link[faces]

            cross = np.cross(
                tri[:, 1] - tri[:, 0],
                tri[:, 2] - tri[:, 0],
            )

            areas = np.linalg.norm(cross, axis=1)

            triangle_idx = int(np.argmax(areas))

            assert areas[triangle_idx] > 1e-12, (
                f"Degenerate visual mesh: {path}"
            )

            triangle = tri[triangle_idx]

            center = triangle.mean(axis=0)

            normal = cross[triangle_idx]
            normal = normal / np.linalg.norm(normal)

            for sign in (1.0, -1.0):

                test_origins_local.append(
                    center + sign * 0.1 * normal
                )

                test_directions_local.append(
                    -sign * normal
                )

            print(
                f"[VISUAL] {name:30s} "
                f"vertices={len(verts_link):5d} "
                f"triangles={len(faces):5d}",
                flush=True,
            )

        mesh_count = len(visual_meshes)

        assert mesh_count == len(visual_records)

        mesh_ids = wp.array2d(
            [[m.id for m in visual_meshes]],
            dtype=wp.uint64,
            device=DEVICE,
        )

        origins_local = np.asarray(
            test_origins_local,
            dtype=np.float64,
        ).reshape(mesh_count, 2, 3)

        directions_local = np.asarray(
            test_directions_local,
            dtype=np.float64,
        ).reshape(mesh_count, 2, 3)

        # 3. Read matching live PhysX link poses.
        def read_visual_poses():

            return (
                robot.data.body_link_pose_w.torch[
                    0, visual_body_ids, :7
                ]
                .detach()
                .cpu()
                .numpy()
                .copy()
                .astype(np.float64)
            )

        # 4. Raycast against any selection of visual meshes.
        def cast(ids, poses, ray_origins, ray_directions):

            origins_t = torch.as_tensor(
                np.ascontiguousarray(
                    ray_origins[None],
                    dtype=np.float32,
                ),
                device=DEVICE,
            )

            directions_t = torch.as_tensor(
                np.ascontiguousarray(
                    ray_directions[None],
                    dtype=np.float32,
                ),
                device=DEVICE,
            )

            positions_t = torch.as_tensor(
                np.ascontiguousarray(
                    poses[None, :, :3],
                    dtype=np.float32,
                ),
                device=DEVICE,
            )

            rotations_t = torch.as_tensor(
                np.ascontiguousarray(
                    poses[None, :, 3:7],
                    dtype=np.float32,
                ),
                device=DEVICE,
            )

            hits, distances, _, _, _ = (
                raycast_dynamic_meshes(
                    ray_starts=origins_t,
                    ray_directions=directions_t,
                    mesh_ids_wp=ids,
                    mesh_positions_w=positions_t,
                    mesh_orientations_w=rotations_t,
                    max_dist=5.0,
                    return_distance=True,
                )
            )

            return (
                hits[0].detach().cpu().numpy(),
                distances[0].detach().cpu().numpy(),
            )


        # --------------------------------------------------
        # A1-3h-4c-2: Full-scene Mid-360 scanning with
        # 300Hz true-physics-pose cache and SO(3) interpolation
        # --------------------------------------------------
        import os
        from scipy.spatial.transform import Slerp

        n_rays = int(os.environ.get("A1_DESKEW_RAYS", "2000"))
        assert 100 <= n_rays <= 20000, n_rays

        cache_path = Path(os.environ.get(
            "A1_SCAN_POSE_CACHE",
            str(ROOT / "a1_47_mappo_eskf_500steps_300hz.npz"),
        )).expanduser()
        assert cache_path.exists(), cache_path
        with np.load(cache_path, allow_pickle=False) as data:
            body_names = [str(n) for n in data["body_names"].tolist()]
            trajectory_all = np.asarray(
                data["body_link_pose_w_xyzw_300hz"], dtype=np.float64
            ).copy()
            t_keys = np.asarray(data["time_s"], dtype=np.float64).copy()
            physics_dt = float(data["physics_dt_s"])

        # Windowed A1-48: select 31 real PhysX snapshots (30 substeps) per scan.
        # Keep the original validated 100 ms raycasting and deskew model.
        scan_window = int(os.environ.get("A1_SCAN_WINDOW", "0"))
        if scan_window < 0:
            raise ValueError("A1_SCAN_WINDOW must be nonnegative")
        if trajectory_all.ndim != 3 or trajectory_all.shape[1:] != (19, 7):
            raise ValueError(f"Unexpected full pose shape: {trajectory_all.shape}")
        if len(t_keys) != len(trajectory_all):
            raise ValueError("Pose and timestamp counts differ")
        begin = scan_window * 30
        end = begin + 31
        if end > len(t_keys):
            raise ValueError(f"Window {scan_window} requires {end} snapshots; "
                             f"cache contains {len(t_keys)}")
        full_start_time_s = float(t_keys[begin])
        # Validate source timing before rebasing the selected 100 ms window.
        if not np.allclose(np.diff(t_keys[begin:end]), physics_dt,
                           atol=1e-9, rtol=0):
            raise ValueError("Non-consecutive physics timestamps in scan window")
        # A3-B1: fixed global t=0 scene anchor
        anchor_indices = [
            body_names.index(f"Falcon{i}_base_link_inertia")
            for i in (1, 2, 3)
        ]
        scene_anchor_bases = trajectory_all[
            0, anchor_indices, :
        ].copy()

        trajectory_all = trajectory_all[begin:end].copy()
        t_keys = t_keys[begin:end].copy() - full_start_time_s
        print(f"[A1_48] window={scan_window} global_start_s={full_start_time_s:.6f} "
              f"source={cache_path}", flush=True)

        assert len(body_names) == 19 and len(set(body_names)) == 19, body_names
        assert trajectory_all.shape == (31, 19, 7), trajectory_all.shape
        assert t_keys.shape == (31,) and np.allclose(
            np.diff(t_keys), physics_dt, atol=1e-9, rtol=0
        )
        assert np.isclose(t_keys[0], 0) and np.isclose(t_keys[-1], .1)
        assert np.isclose(physics_dt, 1 / 300, atol=1e-8)
        assert np.isfinite(trajectory_all).all()
        assert np.max(np.abs(np.linalg.norm(trajectory_all[..., 3:7], axis=-1) - 1)) < 1e-3

        # The source visual geometry order is not the pose-cache order.
        # Align by the exact body name, rather than assuming fixed indices.
        assert len(visual_names) == 19 and len(visual_meshes) == 19
        missing_bodies = sorted(set(visual_names) - set(body_names))
        assert not missing_bodies, f"Cache missing: {missing_bodies}"
        cache_indices = [body_names.index(name) for name in visual_names]
        trajectory = trajectory_all[:, cache_indices, :]  # (31, 19, 7)

        # Static ground and wall have identity poses; their mesh vertices
        # are authored directly in the global frame.
        def to_world_mesh(prim):
            local_verts, faces = primitive_mesh(prim)
            xf = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            verts_W = np.asarray(
                [list(xf.Transform(Gf.Vec3d(*map(float, v))))
                 for v in local_verts],
                dtype=np.float32,
            )
            return convert_to_warp_mesh(verts_W, faces, device=DEVICE)

        ground_prim = stage.GetPrimAtPath("/World/ground/geometry/mesh")
        assert ground_prim.IsValid(), "Zeng ground USD prim is missing"
        ground_mesh = to_world_mesh(ground_prim)

        base_slots = [visual_names.index(f"Falcon{i}_base_link_inertia") for i in (1, 2, 3)]
        mount_B = np.array([0., 0., .08], dtype=np.float64)
        base_initial = scene_anchor_bases
        initial_rot = Rotation.from_quat(base_initial[:, 3:7].copy())
        lidar_initial = base_initial[:, :3] + initial_rot.apply(np.tile(mount_B, (3, 1)))
        wall_center = np.array([
            float(lidar_initial[:, 0].mean()),
            float(lidar_initial[:, 1].mean()) - 4.0,
            float(lidar_initial[:, 2].mean()),
        ])
        wall = UsdGeom.Cube.Define(stage, "/World/A1DeskewWall")
        wall.GetSizeAttr().Set(1.0)
        wall.AddTranslateOp().Set(Gf.Vec3d(*map(float, wall_center)))
        wall.AddScaleOp().Set(Gf.Vec3f(12., .2, 5.))
        wall_mesh = to_world_mesh(wall.GetPrim())

        # Reproduce the validated A2-B1 orthogonal wall
        wall_x_center = np.array([
            float(lidar_initial[:, 0].mean()) + 4.0,
            float(lidar_initial[:, 1].mean()),
            float(lidar_initial[:, 2].mean()),
        ], dtype=np.float64)

        wall_x = UsdGeom.Cube.Define(
            stage, "/World/A2B1WallX"
        )
        wall_x.GetSizeAttr().Set(1.0)
        wall_x.AddTranslateOp().Set(
            Gf.Vec3d(*map(float, wall_x_center))
        )
        wall_x.AddScaleOp().Set(
            Gf.Vec3f(.2, 12., 5.)
        )
        wall_x_mesh = to_world_mesh(wall_x.GetPrim())

        scene_meshes = list(visual_meshes) + [
            ground_mesh, wall_mesh, wall_x_mesh
        ]
        scene_names = list(visual_names) + [
            "ground", "test_wall", "test_wall_x"
        ]
        assert len(scene_meshes) == 22
        all_mesh_ids = wp.array2d(
            [[m.id for m in scene_meshes]], dtype=wp.uint64, device=DEVICE
        )
        static_poses = np.array([[0., 0., 0., 0., 0., 0., 1.]] * 3, dtype=np.float64)

        pattern_file = ROOT / (
            "livox_pattern_ref_aria2/livox_gz_sim-main/"
            "scan_patterns/mid360.csv"
        )
        assert pattern_file.is_file(), pattern_file
        # The downsampled test spreads samples across the entire 100ms scan,
        # instead of taking only the first 10ms of a 2k-ray prefix.
        beam_idx = np.linspace(0, 19999, n_rays, dtype=np.int64)
        assert len(np.unique(beam_idx)) == n_rays
        rows_full = np.loadtxt(pattern_file, delimiter=",", skiprows=1, max_rows=20000)
        assert rows_full.shape == (20000, 3), rows_full.shape
        rows = rows_full[beam_idx]
        az = np.deg2rad(rows[:, 1]); el = np.deg2rad(90.0 - rows[:, 2])
        dirs_sensor = np.column_stack((
            np.cos(el)*np.cos(az),
            np.cos(el)*np.sin(az),
            np.sin(el),
        )).astype(np.float64)
        beam_time = beam_idx / 200000.0
        assert beam_time.max() < t_keys[-1]
        which_bin = np.minimum(np.searchsorted(t_keys, beam_time, side="right") - 1, 29)
        which_bin = np.maximum(which_bin, 0)

        # Mesh transform is assumed constant within each 1/300s bin,
        # evaluated at the bin midpoint (not at the 100Hz control step).
        # LiDAR ray origin/direction IS interpolated at each firing time.
        midpoint = np.empty((30, 19, 7), dtype=np.float64)
        midpoint[..., :3] = .5 * (trajectory[:-1, :, :3] + trajectory[1:, :, :3])
        for j in range(19):
            r = Rotation.from_quat(trajectory[:, j, 3:7].copy())
            midpoint[:, j, 3:7] = Slerp(t_keys, r)(
                .5 * (t_keys[:-1] + t_keys[1:])
            ).as_quat()

        def as_tensor(array):
            return torch.as_tensor(
                np.ascontiguousarray(array, dtype=np.float32), device=DEVICE
            )

        def cast(poses_22, ray_o, ray_d):
            hits, dist, _, _, obj_idx = raycast_dynamic_meshes(
                ray_starts=as_tensor(ray_o[None]),
                ray_directions=as_tensor(ray_d[None]),
                mesh_ids_wp=all_mesh_ids,
                mesh_positions_w=as_tensor(poses_22[None, :, :3]),
                mesh_orientations_w=as_tensor(poses_22[None, :, 3:7]),
                max_dist=30.0,
                return_distance=True,
                return_mesh_id=True,
            )
            pts = hits[0].detach().cpu().numpy().copy()
            d = dist[0].detach().cpu().numpy().copy()
            obj = obj_idx[0].detach().cpu().numpy().copy()
            valid = np.isfinite(d) & (d > 0) & (obj >= 0)
            obj[~valid] = -1
            return pts, d, obj

        shape = (3, n_rays)
        static_ranges = np.full(shape, np.inf, np.float32)
        moving_ranges = np.full(shape, np.inf, np.float32)
        static_ids = np.full(shape, -1, np.int32)
        moving_ids = np.full(shape, -1, np.int32)
        dynamic_xyz_world = np.full((3, n_rays, 3), np.nan, np.float32)
        naive_xyz_S0 = np.full_like(dynamic_xyz_world, np.nan)
        deskewed_xyz_S0 = np.full_like(dynamic_xyz_world, np.nan)

        print("\n===== A1-3h-4c-2 300HZ MOTION DISTORTION =====", flush=True)
        print(f"real_PhysX_snapshots={len(t_keys)} time_span_s={t_keys[-1]:.6f} "
              f"total_visual_meshes={len(scene_meshes)} rays_per_drone={n_rays}", flush=True)

        for drone in range(3):
            base_slot = base_slots[drone]
            base_pose = trajectory[:, base_slot, :]
            base_R = Rotation.from_quat(base_pose[:, 3:7].copy())
            R0 = base_R[0]
            origin0 = base_pose[0, :3] + R0.apply(mount_B)
            ray_o0 = np.tile(origin0, (n_rays, 1))
            ray_d0 = R0.apply(dirs_sensor)

            frozen_poses = np.concatenate([trajectory[0], static_poses], axis=0)
            _, d0, id0 = cast(frozen_poses, ray_o0, ray_d0)
            static_ranges[drone] = d0
            static_ids[drone] = id0

            # Scan with moving sensor pose interpolated at every beam timestamp.
            rot_t = Slerp(t_keys, base_R)(beam_time)
            left = which_bin
            f = ((beam_time - t_keys[left]) / physics_dt)[:, None]
            pos_t = (1 - f) * base_pose[left, :3] + f * base_pose[left+1, :3]
            ray_o = pos_t + rot_t.apply(np.tile(mount_B, (n_rays, 1)))
            ray_d = rot_t.apply(dirs_sensor)

            for bin_id in range(30):
                sel = np.flatnonzero(which_bin == bin_id)
                if not len(sel):
                    continue
                poses_22 = np.concatenate([midpoint[bin_id], static_poses], axis=0)
                hit, dist, ids = cast(poses_22, ray_o[sel], ray_d[sel])
                moving_ranges[drone, sel] = dist
                moving_ids[drone, sel] = ids
                valid = np.isfinite(dist) & (dist > 0) & (ids >= 0)
                if valid.any():
                    matched = sel[valid]
                    dynamic_xyz_world[drone, matched] = hit[valid]
                    naive_xyz_S0[drone, matched] = (
                        dirs_sensor[matched] * dist[valid, None]
                    )
                    deskewed_xyz_S0[drone, matched] = R0.inv().apply(
                        hit[valid] - origin0
                    )

            v0 = np.isfinite(static_ranges[drone])
            vd = np.isfinite(moving_ranges[drone])
            common = v0 & vd
            assert vd.any(), f"No dynamic scan hits for Falcon{drone+1}"
            naive = naive_xyz_S0[drone, vd]
            correct = deskewed_xyz_S0[drone, vd]
            deskew_displacement = np.linalg.norm(correct - naive, axis=1)
            range_difference = np.abs(
                moving_ranges[drone, common] - static_ranges[drone, common]
            )
            env_ids = {19, 20, 21}
            moving_env = int(np.isin(moving_ids[drone], list(env_ids)).sum())
            print(
                f"Falcon{drone+1}: frozen_hits={v0.sum()}/{n_rays} "
                f"moving_hits={vd.sum()}/{n_rays} moving_environment_hits={moving_env} "
                f"common={common.sum()} "
                f"deskew_delta_p50={np.median(deskew_displacement):.6f}m "
                f"deskew_delta_p95={np.percentile(deskew_displacement,95):.6f}m "
                f"range_delta_p95={(np.percentile(range_difference,95) if common.any() else float('nan')):.6f}m",
                flush=True,
            )

        output = (
            ROOT / "A3_1/A3_B1_three_plane"
            / f"a3_b1_mid360_window_{scan_window:02d}_{n_rays}rays.npz"
        )
        if output.exists():
            raise FileExistsError(f"Refusing to overwrite existing scan: {output}")
        np.savez_compressed(
            output,
            scan_window_index=np.int64(scan_window),
            scan_start_time_s=np.float64(full_start_time_s),
            absolute_beam_time_s=full_start_time_s + beam_time,
            pose_cache_source=np.asarray(str(cache_path)),
            scan_uses_mappo_closed_loop_pose=np.asarray(True),
            time_offset_ns=np.rint(beam_time * 1e9).astype(np.int64),
            sampled_beam_indices=beam_idx,
            directions_sensor=dirs_sensor.astype(np.float32),
            ranges_frozen_m=static_ranges,
            ranges_moving_m=moving_ranges,
            object_id_frozen=static_ids,
            object_id_moving=moving_ids,
            xyz_world_at_fire=dynamic_xyz_world,
            xyz_naive_sensor_start=naive_xyz_S0,
            xyz_deskewed_sensor_start=deskewed_xyz_S0,
            scene_names=np.array(scene_names),
            body_names=np.array(visual_names),
            body_pose_times_s=t_keys,
            body_poses_300hz_xyzw=trajectory.astype(np.float32),
            mount_B_m=mount_B,
            wall_center_world_m=wall_center,
            wall_x_center_world_m=wall_x_center,
            geometry_variant=np.asarray(
                "A3-B1: fixed global anchor and A2-B1 orthogonal wall"
            ),
            max_dist_m=np.float64(30.),
            time_reference=np.array("scan start t=0; beam time=idx/200000s"),
            uses_physx_truth_for_sim_sensor_only=np.array(True),
        )
        print("Saved:", output, flush=True)
        print("[A1-3h-4c-2] 300HZ INTERPOLATED MID360 SCAN: PASS", flush=True)
        print("NOTE: 1/300s rigid-body midpoint approximation; not continuous physics.", flush=True)
        print("NOTE: A1-47 MAPPO+ESKF closed-loop trajectory; LiDAR raycast is offline.", flush=True)

    finally:
        env.close()

finally:
    simulation_app.close()
