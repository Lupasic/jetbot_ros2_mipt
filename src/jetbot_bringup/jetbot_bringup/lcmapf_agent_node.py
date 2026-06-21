#!/usr/bin/env python3
"""
Decentralized LC-MAPF agent — one per robot.

Wraps lc_mapf.agent.LCMAPFAgent and exposes it over ROS 2. Self-configures from the
host's latched /mapf_config, then on every /mapf_step:

  1. determine its current MAPF cell and broadcast /robot_{id}/mapf_cell
  2. wait until every robot's cell for this step has arrived (barrier, with timeout)
  3. prepare_step() and discover visible neighbors
  4. run n_comm_rounds communication rounds, publishing /robot_{id}/mapf_message and
     collecting neighbors' messages each round (timeout -> proceed without, graceful)
  5. publish the chosen action on /robot_{id}/action

Position source:
  * simulate_motion (test without driving): the agent advances its own cell virtually
    by the action it chose, so the full pipeline runs with the robot standing still.
  * real motion: the cell comes from the robot's SLAM pose via TF (TODO: wire TF; the
    host owns motion, so the agent only reports where it is).

Identical wire protocol on host and robot — see mapf_proto.py.
"""

import os
import sys
import threading
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from std_msgs.msg import String, Float32MultiArray
import tf2_ros

# mapf_proto ships in whichever package hosts this node (host: multi_robot_commander,
# robot: jetbot_bringup). Keep this file identical in both; only the import resolves differently.
try:
    from multi_robot_commander import mapf_proto as proto
except ImportError:
    try:
        from jetbot_bringup import mapf_proto as proto
    except ImportError:
        import mapf_proto as proto


def _import_lcmapf():
    """Import LCMAPFAgent, adding the lc_mapf source dir to sys.path if needed."""
    for p in (os.environ.get('LC_MAPF_DIR'),
              '/home/app/LC-MAPF', '/home/app/MAPF-GPT'):
        if p and Path(p).exists() and p not in sys.path:
            sys.path.insert(0, p)
    import cppimport.import_hook  # noqa: F401  (enables building observation_generator)
    import torch
    from lc_mapf.agent import LCMAPFAgent, AgentConfig
    return torch, LCMAPFAgent, AgentConfig


class LCMAPFAgentNode(Node):
    """Per-robot decentralized inference node."""

    def __init__(self, **kwargs) -> None:
        super().__init__('lcmapf_agent', **kwargs)

        self.declare_parameter('robot_id', -1)
        self.declare_parameter('weights_dir',
                               os.environ.get('MAPF_GPT_WEIGHTS_DIR',
                                              '/home/app/mapf_gpt_data/weights'))
        self.declare_parameter('msg_timeout', 0.5)   # s to wait per comm round
        self.declare_parameter('cell_timeout', 2.0)  # s to wait for all cells

        self.robot_id = int(self.get_parameter('robot_id').value)
        if self.robot_id < 0:
            raise RuntimeError('robot_id parameter is required (>= 0)')
        self.weights_dir = Path(self.get_parameter('weights_dir').value)
        self.msg_timeout = float(self.get_parameter('msg_timeout').value)
        self.cell_timeout = float(self.get_parameter('cell_timeout').value)

        self._torch, self._LCMAPFAgent, self._AgentConfig = _import_lcmapf()

        self._agent = None
        self._cfg: Optional[dict] = None
        self._config_id = None            # epoch of the a-priori config we initialized for
        self._sorted_ids = None           # current team (for re-init detection)
        self._obstacle_map = None
        self._subscribed_cell = set()     # robot_ids we already sub to (avoid duplicates)
        self._subscribed_msg = set()
        self._pos = None                  # my current padded cell
        self._lock = threading.Condition()
        self._cells: Dict[int, tuple] = {}            # step-scoped: robot_id -> (row,col)
        self._msgs: Dict[tuple, Dict[int, list]] = {} # (step,round) -> {sender: vec}
        self._cells_step = -1
        self._last_step_key = None         # (config_id, step) already processed
        self._last_action_msg = None       # cached action for duplicate-step re-publish

        self._cbg = ReentrantCallbackGroup()

        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(String, proto.CONFIG_TOPIC, self._on_config,
                                 latched, callback_group=self._cbg)
        self.create_subscription(String, proto.STEP_TOPIC, self._on_step,
                                 10, callback_group=self._cbg)

        # Publishers (created here; cross-robot subscriptions created once config arrives)
        self._cell_pub = self.create_publisher(String, proto.cell_topic(self.robot_id), 10)
        self._msg_pub = self.create_publisher(
            Float32MultiArray, proto.message_topic(self.robot_id), 10)
        self._action_pub = self.create_publisher(String, proto.action_topic(self.robot_id), 10)

        # TF (live mode): this robot reads its OWN SLAM pose and converts it to a MAPF
        # cell with the exact host conventions (mapf_proto.world_to_padded_cell). The
        # node runs in the robot's namespace, so the listener picks up /robot_{id}/tf.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=False)
        self._map_frame = f'robot_{self.robot_id}/map'
        self._base_frame = f'robot_{self.robot_id}/base_footprint'

        self.get_logger().info(
            f'[MAPF][r{self.robot_id}] agent node up — waiting for /mapf_config')

    # ── Config / (re)initialization ───────────────────────────────────────────
    def _on_config(self, msg: String) -> None:
        """
        A-priori initialization for the whole team. The agent (re)initializes its
        LCMAPFAgent + observation generator whenever it sees a NEW config_id (epoch),
        e.g. a new episode or a changed team. Same epoch => ignored.
        """
        cfg = proto.decode_config(msg.data)
        new_id = cfg['config_id']
        if new_id and new_id == self._config_id:
            return  # already initialized for this epoch
        if self.robot_id not in cfg['sorted_robot_ids']:
            self.get_logger().warn(
                f'[MAPF][r{self.robot_id}] not in team {cfg["sorted_robot_ids"]} '
                f'for config {new_id} — idle')
            return

        same_team = (self._agent is not None
                     and cfg['sorted_robot_ids'] == self._sorted_ids
                     and cfg['obstacle_map'] == self._obstacle_map)
        self._cfg = cfg
        self._config_id = new_id
        self._pos = tuple(cfg['initial_positions'][self.robot_id])
        with self._lock:
            self._cells = {}
            self._msgs = {}
            self._cells_step = -1

        if same_team:
            # Team + map unchanged: just reset episode state (no model reload).
            self._agent.reset()
            self.get_logger().info(
                f'[MAPF][r{self.robot_id}] re-initialized (same team) epoch={new_id} '
                f'init_pos={self._pos} goal={cfg["goals"][self.robot_id]}')
            return

        # New team/map: rebuild the agent (reloads weights, fresh observation generator).
        weights_path = self.weights_dir / cfg['weights']
        agent_cfg = self._AgentConfig(
            num_rounds=cfg['num_rounds']) if cfg['num_rounds'] is not None else None
        self._agent = self._LCMAPFAgent(
            path_to_weights=str(weights_path),
            robot_id=self.robot_id,
            sorted_robot_ids=cfg['sorted_robot_ids'],
            obstacle_map=cfg['obstacle_map'],
            device=cfg['device'],
            cfg=agent_cfg)
        self._agent.reset()
        self._sorted_ids = list(cfg['sorted_robot_ids'])
        self._obstacle_map = cfg['obstacle_map']

        # Subscribe to any newly-seen robots' cell/message topics (idempotent).
        for rid in cfg['sorted_robot_ids']:
            if rid not in self._subscribed_cell:
                self.create_subscription(
                    String, proto.cell_topic(rid),
                    lambda m, r=rid: self._on_cell(r, m), 10, callback_group=self._cbg)
                self._subscribed_cell.add(rid)
            if rid != self.robot_id and rid not in self._subscribed_msg:
                self.create_subscription(
                    Float32MultiArray, proto.message_topic(rid),
                    lambda m, r=rid: self._on_message(r, m), 10,
                    callback_group=self._cbg)
                self._subscribed_msg.add(rid)

        self.get_logger().info(
            f'[MAPF][r{self.robot_id}] INITIALIZED epoch={new_id} '
            f'team={cfg["sorted_robot_ids"]} | rounds={self._agent.n_comm_rounds} '
            f'| msg_dim={self._agent.message_dim} | init_pos={self._pos} '
            f'| goal={cfg["goals"][self.robot_id]} | simulate_motion={cfg["simulate_motion"]}')

    # ── Inbound buffers ────────────────────────────────────────────────────────
    def _on_cell(self, robot_id: int, msg: String) -> None:
        step, row, col = proto.decode_cell(msg.data)
        with self._lock:
            if step != self._cells_step:
                self._cells = {}
                self._cells_step = step
            self._cells[robot_id] = (row, col)
            self._lock.notify_all()

    def _on_message(self, sender_id: int, msg: Float32MultiArray) -> None:
        step, rnd, vec = proto.unpack_message(msg.data)
        with self._lock:
            self._msgs.setdefault((step, rnd), {})[sender_id] = vec
            self._lock.notify_all()

    # ── Step ────────────────────────────────────────────────────────────────────
    def _on_step(self, msg: String) -> None:
        step, last_actions, step_cfg = proto.decode_step(msg.data)
        if self._agent is None:
            self.get_logger().warn(
                f'[MAPF][r{self.robot_id}] step{step} before config — waiting for /mapf_config')
            return
        if step_cfg and step_cfg != self._config_id:
            self.get_logger().warn(
                f'[MAPF][r{self.robot_id}] step{step} for config {step_cfg} but initialized '
                f'for {self._config_id} — waiting for matching /mapf_config')
            return
        # Idempotency: the orchestrator re-publishes a step until it has all actions. Process
        # each (config_id, step) once; on a duplicate just re-send the cached action.
        key = (step_cfg, step)
        with self._lock:
            if key == self._last_step_key:
                if self._last_action_msg is not None:
                    self._action_pub.publish(self._last_action_msg)
                return
            self._last_step_key = key
        cfg = self._cfg
        ids: List[int] = cfg['sorted_robot_ids']

        # 0. Live mode: refresh my position from my own SLAM pose (TF). simulate_motion
        #    keeps the virtual cell advanced at the end of the previous step.
        if not cfg['simulate_motion']:
            self._pos = self._read_pose_cell()

        # 1. Broadcast my current cell.
        with self._lock:
            if step != self._cells_step:
                self._cells = {}
                self._cells_step = step
            self._cells[self.robot_id] = tuple(self._pos)
        self._cell_pub.publish(String(data=proto.encode_cell(step, self._pos[0], self._pos[1])))

        # 2. Barrier: wait for everyone's cell.
        all_positions = self._await_cells(step, ids)
        all_goals = [tuple(cfg['goals'][r]) for r in ids]
        all_last = [int(last_actions.get(r, -1)) for r in ids]

        # 3. Prepare step + discover neighbors.
        self._agent.prepare_step(all_positions, all_goals, all_last)
        neighbors = self._agent.get_neighbor_robot_ids()
        self.get_logger().info(
            f'[MAPF][r{self.robot_id}][step{step}] pos={self._pos} '
            f'neighbors={neighbors} last_actions={all_last}')

        # 4. Communication rounds.
        n_rounds = self._agent.n_comm_rounds
        received: Dict[int, object] = {}
        for rnd in range(n_rounds):
            out_msg = self._agent.communication_round(received)
            vec = out_msg.tolist()
            self._msg_pub.publish(Float32MultiArray(data=proto.pack_message(step, rnd, vec)))
            self.get_logger().info(
                f'[MAPF][r{self.robot_id}][step{step}][round{rnd}] sent msg '
                f'|v|={sum(x * x for x in vec) ** 0.5:.3f} to {neighbors}')
            if rnd < n_rounds - 1:
                received = self._await_messages(step, rnd, neighbors)

        # 5. Publish action (cache it so a re-published step re-sends the same action).
        action = self._agent.get_action()
        action_msg = String(data=proto.encode_action(step, action))
        self._last_action_msg = action_msg
        self._action_pub.publish(action_msg)
        self.get_logger().info(
            f'[MAPF][r{self.robot_id}][step{step}] ACTION={action} '
            f'({proto.ACTION_DELTA[action]})')

        # 6. Virtual advance (test-without-driving). Real motion is owned by the host.
        if cfg['simulate_motion']:
            self._pos = proto.advance(self._pos, action, cfg['obstacle_map'])

    def _read_pose_cell(self):
        """Live mode: my SLAM pose (TF map->base_footprint) -> padded MAPF cell."""
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            cell = proto.world_to_padded_cell(
                x, y, self._cfg['grid_transform'],
                self._cfg['map_h'], self._cfg['map_w'], self._cfg['pad'])
            self.get_logger().info(
                f'[MAPF][r{self.robot_id}] TF pose=({x:.3f},{y:.3f}) -> cell {cell}')
            return cell
        except Exception as e:  # noqa: BLE001 — TF may be momentarily unavailable
            self.get_logger().warn(
                f'[MAPF][r{self.robot_id}] TF lookup failed ({e}); keeping cell {self._pos}')
            return tuple(self._pos)

    # ── Barriers ──────────────────────────────────────────────────────────────
    def _await_cells(self, step: int, ids: List[int]) -> List[tuple]:
        deadline = self._now() + self.cell_timeout
        with self._lock:
            while any(r not in self._cells for r in ids):
                remaining = deadline - self._now()
                if remaining <= 0:
                    missing = [r for r in ids if r not in self._cells]
                    self.get_logger().warn(
                        f'[MAPF][r{self.robot_id}][step{step}] cell timeout, missing '
                        f'{missing}; using last-known/self')
                    break
                self._lock.wait(timeout=remaining)
            # Fall back to own position / config initial for any still-missing robot.
            out = []
            for r in ids:
                if r in self._cells:
                    out.append(tuple(self._cells[r]))
                else:
                    out.append(tuple(self._cfg['initial_positions'][r]))
            return out

    def _await_messages(self, step: int, rnd: int, neighbors: List[int]) -> Dict[int, object]:
        if not neighbors:
            return {}
        deadline = self._now() + self.msg_timeout
        key = (step, rnd)
        with self._lock:
            while any(n not in self._msgs.get(key, {}) for n in neighbors):
                remaining = deadline - self._now()
                if remaining <= 0:
                    got = list(self._msgs.get(key, {}).keys())
                    missing = [n for n in neighbors if n not in got]
                    if missing:
                        self.get_logger().warn(
                            f'[MAPF][r{self.robot_id}][step{step}][round{rnd}] '
                            f'msg timeout, missing {missing} — proceeding without')
                    break
                self._lock.wait(timeout=remaining)
            raw = dict(self._msgs.get(key, {}))
        return {sid: self._torch.tensor(vec, dtype=self._torch.float32)
                for sid, vec in raw.items() if sid in neighbors}

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LCMAPFAgentNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        print(f'Failed to start lcmapf_agent: {e}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
