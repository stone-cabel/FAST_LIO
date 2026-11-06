#!/usr/bin/env python3

from pathlib import Path
import numpy as np

ROOT = Path.home() / "research/isaac_a1_diagnostics"

OLD = "a2_4b_0_mid360_window"
SOURCE = "a2_4b_1_mid360_window"
OUTPUT = "a2_4b_3_mid360_window"

OBJECTS = ("ground", "test_wall", "test_wall_x")


def valid_mask(ranges, ids):
    return (
        np.isfinite(ranges)
        & (ranges > 0.5)
        & (ids >= 0)
    )


def temporal_sample(indices, count):
    """Deterministic sampling across beam acquisition order."""
    indices = np.sort(np.asarray(indices, dtype=np.int64))

    assert 0 <= count <= len(indices)

    if count == 0:
        return np.empty(0, dtype=np.int64)

    if count == len(indices):
        return indices.copy()

    # Since beam offsets are ordered, evenly spaced indices
    # approximate temporal stratification.
    positions = np.linspace(
        0, len(indices) - 1, count
    ).astype(np.int64)

    selected = indices[positions]

    assert len(np.unique(selected)) == count
    return selected


print("========== A2-4B-3 EQUAL-POINT DATASET ==========")

for w in range(8):

    old_path = ROOT / f"{OLD}_{w:02d}_2000rays.npz"
    src_path = ROOT / f"{SOURCE}_{w:02d}_2000rays.npz"
    out_path = ROOT / f"{OUTPUT}_{w:02d}_2000rays.npz"

    assert old_path.is_file(), old_path
    assert src_path.is_file(), src_path
    assert not out_path.exists(), (
        f"Refusing to overwrite: {out_path}"
    )

    with np.load(old_path, allow_pickle=False) as z:
        old_ranges = np.asarray(
            z["ranges_moving_m"]
        )
        old_ids = np.asarray(
            z["object_id_moving"]
        )

    with np.load(src_path, allow_pickle=False) as z:
        data = {key: z[key] for key in z.files}

    ranges = data["ranges_moving_m"].copy()
    ids = data["object_id_moving"].copy()

    names = [
        str(name) for name in data["scene_names"]
    ]

    offsets = np.asarray(data["time_offset_ns"])

    assert names[-3:] == list(OBJECTS)
    assert ranges.shape == (3, 2000)
    assert ids.shape == (3, 2000)
    assert old_ranges.shape == ranges.shape
    assert old_ids.shape == ids.shape
    assert len(offsets) == 2000
    assert np.all(np.diff(offsets) >= 0)

    original_last_times = []
    target_counts = []

    for drone in range(3):

        target = int(
            valid_mask(
                old_ranges[drone],
                old_ids[drone]
            ).sum()
        )

        source_valid = valid_mask(
            ranges[drone], ids[drone]
        )

        original_last_times.append(
            int(offsets[np.flatnonzero(source_valid)[-1]])
        )

        groups = {
            name: np.flatnonzero(
                source_valid
                & (ids[drone] == names.index(name))
            )
            for name in OBJECTS
        }

        # 本实验要求有效点均来自三个静态对象。
        all_env_count = sum(
            len(v) for v in groups.values()
        )

        assert all_env_count == int(source_valid.sum()), (
            f"Window {w}, drone {drone}: "
            "unexpected dynamic-object returns"
        )

        assert target >= 3 * 20
        assert all(len(v) >= 20 for v in groups.values())

        # Ground 回波最少，因此尽可能完整保留。
        n_ground = len(groups["ground"])

        # Y 墙保留约35%的目标点数。
        n_wall_y = min(
            len(groups["test_wall"]),
            max(20, int(round(target * 0.35)))
        )

        # 剩余点数分配给 X 墙。
        n_wall_x = target - n_ground - n_wall_y

        # 若 X 墙容量不足，把剩余配额转移到 Y 墙。
        if n_wall_x > len(groups["test_wall_x"]):
            extra = n_wall_x - len(groups["test_wall_x"])
            n_wall_x -= extra
            n_wall_y += extra

        assert 20 <= n_wall_x <= len(groups["test_wall_x"])
        assert 20 <= n_wall_y <= len(groups["test_wall"])
        assert n_ground >= 20

        selected = np.concatenate([
            temporal_sample(groups["ground"], n_ground),
            temporal_sample(groups["test_wall"], n_wall_y),
            temporal_sample(groups["test_wall_x"], n_wall_x),
        ])

        assert len(selected) == target
        assert len(np.unique(selected)) == target

        keep = np.zeros(2000, dtype=bool)
        keep[selected] = True

        # 仅屏蔽未保留的动态扫描回波。
        # 原始束索引与点时间偏移保持不变。
        ranges[drone, ~keep] = np.inf
        ids[drone, ~keep] = -1

        for key in (
            "xyz_world_at_fire",
            "xyz_naive_sensor_start",
            "xyz_deskewed_sensor_start",
        ):
            if key in data:
                array = data[key].copy()
                array[drone, ~keep] = np.nan
                data[key] = array

        target_counts.append(target)

        actual = int(
            valid_mask(
                ranges[drone], ids[drone]
            ).sum()
        )

        assert actual == target

        # 保留 B1 中最后一个有效回波的时刻，
        # 避免抽样额外改变扫描的末点时间。
        last_offset = int(
            offsets[np.flatnonzero(
                valid_mask(ranges[drone], ids[drone])
            )[-1]]
        )

        assert last_offset == original_last_times[-1], (
            f"Window {w}, drone {drone}: "
            "last return timestamp changed"
        )

        print(
            f"Window {w:02d} Falcon{drone + 1}: "
            f"target={target} actual={actual} "
            f"ground={n_ground} "
            f"wallY={n_wall_y} "
            f"wallX={n_wall_x}"
        )

    data["ranges_moving_m"] = ranges
    data["object_id_moving"] = ids

    data["scan_experiment"] = np.asarray("A2-4B-3")
    data["scene_geometry"] = np.asarray(
        "three_planes_equal_return_count"
    )
    data["subsample_source"] = np.asarray(str(src_path))
    data["subsample_reference"] = np.asarray(str(old_path))
    data["subsample_method"] = np.asarray(
        "per-plane deterministic temporal sampling"
    )
    data["subsample_target_counts_by_drone"] = np.asarray(
        target_counts, dtype=np.int64
    )

    np.savez_compressed(out_path, **data)

    print("Saved:", out_path.name)

print("\nA2-4B-3 DATASET GENERATION: PASS")
