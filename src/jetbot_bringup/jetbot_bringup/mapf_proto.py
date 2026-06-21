"""
Shared wire protocol for distributed (decentralized) LC-MAPF inference.

Pure Python (json + lists only — no ROS, no torch), so the SAME module can run on
the host (distributed_orchestrator) and on each robot (lcmapf_agent_node). Keep this
file identical in both packages; it defines the topics, payload (de)serialization,
map padding, and the MAPF transition rule used by the virtual-advance test mode.

Coordinate convention: MAPF frame [row, col], row 0 = top. All positions/goals/maps
carried over the wire are in the PADDED frame (the orchestrator pads once so the C++
observation generator never reads out of bounds — see INSTRUCTIONS.md caveat 1).

Action codes: 0=wait, 1=up, 2=down, 3=left, 4=right.
"""

import json
import math
from typing import Dict, List, Tuple

# MAPF action -> (d_row, d_col). Matches lc_mapf and follow_stagger.MAPF_ACTION_DELTA.
ACTION_DELTA: Dict[int, Tuple[int, int]] = {
    0: (0, 0),    # wait
    1: (-1, 0),   # up
    2: (1, 0),    # down
    3: (0, -1),   # left
    4: (0, 1),    # right
}

# Padding added around the maze before inference. agents_radius (5) + cost2go_radius (5):
# the C++ observation generator reads position +- both radii with no bounds checking.
DEFAULT_PAD = 10

# ── Topic names ──────────────────────────────────────────────────────────────
CONFIG_TOPIC = '/mapf_config'   # std_msgs/String (JSON), latched TRANSIENT_LOCAL
STEP_TOPIC = '/mapf_step'       # std_msgs/String (JSON)


def cell_topic(robot_id: int) -> str:
    """Per-robot current MAPF cell broadcast (std_msgs/String JSON)."""
    return f'/robot_{robot_id}/mapf_cell'


def message_topic(robot_id: int) -> str:
    """Per-robot communication-round message (std_msgs/Float32MultiArray)."""
    return f'/robot_{robot_id}/mapf_message'


def action_topic(robot_id: int) -> str:
    """Per-robot chosen action for the current step (std_msgs/String JSON)."""
    return f'/robot_{robot_id}/action'


# ── Map helpers ──────────────────────────────────────────────────────────────
def parse_map_string(map_str: str) -> List[List[int]]:
    """Parse a '#'/'.' literal-block map into a 0/1 grid (1 = obstacle)."""
    rows = [line for line in map_str.splitlines() if line.strip() != '']
    width = max(len(r) for r in rows)
    grid = []
    for r in rows:
        # Pad short rows on the right with obstacles so the grid is rectangular.
        grid.append([1 if (c >= len(r) or r[c] == '#') else 0 for c in range(width)])
    return grid


def pad_map(grid: List[List[int]], pad: int = DEFAULT_PAD) -> List[List[int]]:
    """Return a copy of `grid` surrounded by `pad` free (0) cells on every side."""
    h = len(grid)
    w = len(grid[0]) if h else 0
    new_w = w + 2 * pad
    padded = [[0] * new_w for _ in range(pad)]
    for row in grid:
        padded.append([0] * pad + list(row) + [0] * pad)
    padded.extend([[0] * new_w for _ in range(pad)])
    return padded


def pad_pos(pos, pad: int = DEFAULT_PAD) -> Tuple[int, int]:
    """Shift an unpadded [row, col] into the padded frame."""
    return (pos[0] + pad, pos[1] + pad)


def unpad_pos(pos, pad: int = DEFAULT_PAD) -> Tuple[int, int]:
    """Shift a padded [row, col] back to the original (unpadded) frame."""
    return (pos[0] - pad, pos[1] - pad)


def world_to_grid(x: float, y: float, params: dict) -> Tuple[int, int]:
    """
    World (m) -> grid (col, row). Exact replica of GridMapHelper.world_to_grid
    (grid_map_parser.py): translate by offset, inverse-rotate, floor-divide by step.
    `params` carries step, offset_x, offset_y, rotation_deg (degrees, like the YAML).
    """
    dx = x - params['offset_x']
    dy = y - params['offset_y']
    rot = math.radians(params['rotation_deg'])
    ca = math.cos(-rot)
    sa = math.sin(-rot)
    gx = dx * ca - dy * sa
    gy = dx * sa + dy * ca
    col = int(math.floor(gx / params['step']))
    row = int(math.floor(gy / params['step']))
    return (col, row)


def grid_to_mapf(grid_col: int, grid_row: int, map_h: int, map_w: int) -> Tuple[int, int]:
    """grid (col,row) -> MAPF (row,col). Replica of inference_utils.grid_to_mapf."""
    return ((map_h - 1) - grid_col, (map_w - 1) - grid_row)


def world_to_padded_cell(x: float, y: float, params: dict,
                         map_h: int, map_w: int, pad: int) -> Tuple[int, int]:
    """Full chain: robot world pose -> padded MAPF cell used by the agent."""
    col, row = world_to_grid(x, y, params)
    mr, mc = grid_to_mapf(col, row, map_h, map_w)
    return (mr + pad, mc + pad)


def advance(pos, action: int, grid: List[List[int]]) -> Tuple[int, int]:
    """
    Apply the MAPF transition rule (padded frame). Returns the new cell, or the
    same cell if the move would hit a wall or leave the grid (POGEMA semantics).
    """
    dr, dc = ACTION_DELTA.get(int(action), (0, 0))
    nr, nc = pos[0] + dr, pos[1] + dc
    if 0 <= nr < len(grid) and 0 <= nc < len(grid[0]) and grid[nr][nc] == 0:
        return (nr, nc)
    return (pos[0], pos[1])


# ── Payload (de)serialization ────────────────────────────────────────────────
def encode_config(sorted_robot_ids, obstacle_map, goals, initial_positions, *,
                  config_id='', pad=DEFAULT_PAD, device='cpu', weights='LC-MAPF-3M.pt',
                  num_rounds=None, simulate_motion=False, max_steps=256,
                  grid_transform=None, map_h=0, map_w=0) -> str:
    """
    Build the latched /mapf_config payload — the a-priori initialization for the whole
    team (which robots participate, the map, goals, initial cells). `config_id` is the
    epoch: a robot (re)initializes its LCMAPFAgent/observation generator whenever it sees
    a NEW config_id, and only acts on /mapf_step messages tagged with the SAME config_id.
    `obstacle_map`, `goals`, `initial_positions` are in the PADDED frame, keyed by
    str(robot_id). `grid_transform` + `map_h`/`map_w` let each robot convert TF -> cell.
    """
    return json.dumps({
        'config_id': str(config_id),
        'sorted_robot_ids': list(sorted_robot_ids),
        'obstacle_map': obstacle_map,
        'goals': {str(k): list(v) for k, v in goals.items()},
        'initial_positions': {str(k): list(v) for k, v in initial_positions.items()},
        'pad': pad,
        'device': device,
        'weights': weights,
        'num_rounds': num_rounds,
        'simulate_motion': bool(simulate_motion),
        'max_steps': int(max_steps),
        'grid_transform': grid_transform or {},
        'map_h': int(map_h),
        'map_w': int(map_w),
    })


def decode_config(payload: str) -> dict:
    d = json.loads(payload)
    d['sorted_robot_ids'] = [int(x) for x in d['sorted_robot_ids']]
    d['goals'] = {int(k): tuple(v) for k, v in d['goals'].items()}
    d['initial_positions'] = {int(k): tuple(v) for k, v in d['initial_positions'].items()}
    d.setdefault('config_id', '')
    d.setdefault('grid_transform', {})
    d.setdefault('map_h', 0)
    d.setdefault('map_w', 0)
    return d


def encode_step(step: int, last_actions: Dict[int, int], config_id: str = '') -> str:
    return json.dumps({'step': int(step), 'config_id': str(config_id),
                       'last_actions': {str(k): int(v) for k, v in last_actions.items()}})


def decode_step(payload: str) -> Tuple[int, Dict[int, int], str]:
    d = json.loads(payload)
    return (int(d['step']),
            {int(k): int(v) for k, v in d['last_actions'].items()},
            str(d.get('config_id', '')))


def encode_cell(step: int, row: int, col: int) -> str:
    return json.dumps({'step': int(step), 'row': int(row), 'col': int(col)})


def decode_cell(payload: str) -> Tuple[int, int, int]:
    d = json.loads(payload)
    return int(d['step']), int(d['row']), int(d['col'])


def encode_action(step: int, action: int) -> str:
    return json.dumps({'step': int(step), 'action': int(action)})


def decode_action(payload: str) -> Tuple[int, int]:
    d = json.loads(payload)
    return int(d['step']), int(d['action'])


def pack_message(step: int, comm_round: int, vec) -> List[float]:
    """Float32MultiArray.data layout: [step, round, *message_floats]."""
    return [float(step), float(comm_round)] + [float(x) for x in vec]


def unpack_message(data) -> Tuple[int, int, List[float]]:
    """Inverse of pack_message. Returns (step, round, message_floats)."""
    return int(data[0]), int(data[1]), [float(x) for x in data[2:]]
