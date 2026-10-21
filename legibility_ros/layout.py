"""Block-layout contracts shared by publishers, planner, optimizer and tests."""
from copy import deepcopy
import hashlib
import json
import math

import numpy as np

from .contracts import read_experiment, vector


def layout_payload(message):
    if not isinstance(message.header.frame_id, str) or not message.header.frame_id.strip():
        raise ValueError("layout requires an explicit frame")
    blocks = []
    for b in message.blocks:
        if type(b.id) is not int or not isinstance(b.name, str) or not b.name.strip():
            raise ValueError("each block requires an integer ID and a name")
        p, r = b.pose.position, b.pose.orientation
        position = vector([p.x, p.y, p.z], 3, "block position")
        quaternion = vector([r.x, r.y, r.z, r.w], 4, "block orientation")
        if not np.isclose(np.linalg.norm(quaternion), 1, atol=1e-6, rtol=0):
            raise ValueError("block orientation must be a unit quaternion")
        blocks.append(dict(id=b.id, name=b.name, position=position.tolist(),
                           quaternion=quaternion.tolist()))
    if len(blocks) < 2 or len({b['id'] for b in blocks}) != len(blocks):
        raise ValueError("layout requires at least two distinct block IDs")
    if len({b['name'] for b in blocks}) != len(blocks):
        raise ValueError("block names must be unique")
    z = float(message.goal_tool_z)
    if not np.isfinite(z):
        raise ValueError("goal_tool_z must be finite")
    return dict(frame=message.header.frame_id, blocks=blocks, goal_tool_z=z)


def layout_digest(message):
    text = json.dumps(layout_payload(message), sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def snapshot_layout(message, frame, safety):
    """Validate and freeze the complete layout, preserving ID/row ordering."""
    snapshot = deepcopy(message)
    data = layout_payload(snapshot)
    if data['frame'] != frame:
        raise ValueError("layout frame differs from configured FK frame; transform explicitly")
    if snapshot.layout_id != layout_digest(snapshot):
        raise ValueError("layout_id does not match the layout contents")
    ids = [b['id'] for b in data['blocks']]
    goals = np.array([[*b['position'][:2], data['goal_tool_z']] for b in data['blocks']])
    validate_goals(goals, safety)
    return snapshot, ids, goals


def validate_goals(goals, safety):
    goals = np.asarray(goals, dtype=float)
    if goals.ndim != 2 or goals.shape[1] != 3 or len(goals) < 2 or not np.all(np.isfinite(goals)):
        raise ValueError("goals must be a finite (M,3) array with M >= 2")
    if len(np.unique(goals, axis=0)) != len(goals):
        raise ValueError("candidate goals must be distinct")
    low = vector(safety['workspace_min'], 3, 'workspace minimum')
    high = vector(safety['workspace_max'], 3, 'workspace maximum')
    if np.any(goals < low) or np.any(goals > high):
        raise ValueError("candidate goals are outside the configured workspace")
    return goals


def layout_from_config(cfg, messages):
    read_experiment(cfg)
    layout = messages.BlockLayout()
    layout.header.frame_id = cfg['frame_id']
    layout.goal_tool_z = float(cfg['motion']['touch_z']) + float(cfg['motion']['hover_height'])
    for data in cfg['blocks']:
        block = messages.Block()
        block.id, block.name = data['id'], data['name']
        block.pose.position.x, block.pose.position.y, block.pose.position.z = map(float, data['position'])
        yaw = math.radians(float(data.get('yaw', 0))) / 2
        block.pose.orientation.z, block.pose.orientation.w = math.sin(yaw), math.cos(yaw)
        layout.blocks.append(block)
    layout.layout_id = layout_digest(layout)
    return layout
