"""Record an illustrative calibration sweep and run Dream's real upload preflight."""

from dataclasses import asdict
import json
import os
from pathlib import Path

os.environ["DREAM_LOG"] = "off"

import av
from dream.datasets import validate_dataset
import mujoco
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from dream_bridge import ACTION_HZ, EMBODIMENT, apply_chunk
from model import HERE, internal_to_logical


def run():
    root = HERE / "results" / "dataset_fixture"
    meta = root / "meta"
    video = root / "videos/chunk-000/observation.images.overhead/episode_000000.mp4"
    parquet = root / "data/chunk-000/episode_000000.parquet"
    for folder in (meta, video.parent, parquet.parent):
        folder.mkdir(parents=True, exist_ok=True)
    frames = 90
    task = "Follow the calibration sweep. Synthetic format fixture only."
    names = ["shoulder_bend", "elbow_bend"]
    features = {key:{"dtype":"float32", "shape":[2], "names":names}
                for key in ("observation.state", "action")}
    features["observation.images.overhead"] = {"dtype":"video", "shape":[240,320,3],
                                                "names":["height","width","channels"]}
    info = {"codebase_version":"v2.1", "robot_type":"illustrative_rolling_sample", "fps":ACTION_HZ,
            "total_episodes":1, "total_frames":frames, "total_tasks":1, "chunks_size":1000,
            "data_path":"data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path":"videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "features":features}
    (meta / "info.json").write_text(json.dumps(info, indent=2)+"\n")
    (meta / "tasks.jsonl").write_text(json.dumps({"task_index":0,"task":task})+"\n")
    (meta / "episodes.jsonl").write_text(json.dumps({"episode_index":0,"length":frames,"tasks":[task]})+"\n")
    model = mujoco.MjModel.from_xml_path(str(HERE / "arm.xml"))
    data = mujoco.MjData(model)
    states, actions = [], []
    with av.open(str(video), mode="w") as container, mujoco.Renderer(model, height=240, width=320) as renderer:
        stream = container.add_stream("libx264", rate=ACTION_HZ)
        stream.width, stream.height, stream.pix_fmt = 320, 240, "yuv420p"
        for index in range(frames):
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera="inspection")
            frame = av.VideoFrame.from_ndarray(renderer.render(), format="rgb24")
            for packet in stream.encode(frame):
                container.mux(packet)
            target = [.35*np.sin(index/30), .45*np.sin(index/24)]
            states.append(internal_to_logical(data.qpos).astype(np.float32).tolist())
            actions.append(target)
            apply_chunk(model, data, {"per_key":{"action.arm":[target]}})
        for packet in stream.encode():
            container.mux(packet)
    vector = pa.list_(pa.float32(), 2)
    table = pa.table({"observation.state":pa.array(states, type=vector),
                      "action":pa.array(actions, type=vector),
                      "timestamp":np.arange(frames, dtype=np.float64)/ACTION_HZ,
                      "frame_index":np.arange(frames, dtype=np.int64),
                      "episode_index":np.zeros(frames, dtype=np.int64),
                      "index":np.arange(frames, dtype=np.int64),
                      "task_index":np.zeros(frames, dtype=np.int64)})
    pq.write_table(table, parquet)
    report = validate_dataset(root, EMBODIMENT)
    assert report.ok, asdict(report)
    with av.open(str(video)) as recorded:
        decoded_frames = len(list(recorded.decode(video=0)))
    assert decoded_frames == frames
    malformed = table.set_column(1, "action", pa.array([[0.] for _ in range(frames)], type=pa.list_(pa.float32(),1)))
    pq.write_table(malformed, parquet)
    rejected = validate_dataset(root, EMBODIMENT)
    pq.write_table(table, parquet)
    assert not rejected.ok, "Dream validator accepted a wrong-width action dataset"
    restored = validate_dataset(root, EMBODIMENT)
    assert restored.ok, asdict(restored)
    result = {"verdict":"PASS for local dataset upload preflight", "report":asdict(report),
              "decoded_video_frames":decoded_frames, "malformed_width_rejected":rejected.error,
              "restored_dataset_passes":True, "cloud_uploads":0,
              "purpose":"Synthetic calibration fixture for format validation; not a task-training dataset",
              "alignment":"capture image/state before applying same-row absolute logical target for 1/30 second"}
    (HERE / "results/dataset_validation.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    run()
