#!/usr/bin/env python3
"""
warehouse_manager.py — ROS 2 Warehouse Management System Node
=============================================================
Publishes:
  /warehouse/task_queue        (String JSON)
  /warehouse/fleet_status      (String JSON)
  /warehouse/robot_{i}/goal    (geometry_msgs/PoseStamped)

Subscribes:
  /model/tugbot_{i}/odometry   (nav_msgs/Odometry)
  /warehouse/dispatch_order    (String)

Run standalone (no ROS):
    python3 warehouse_manager.py --standalone

With ROS 2:
    ros2 run warehouse_simulation warehouse_manager
"""

import sys, math, json, time, random, threading, argparse

# ── Try ROS 2, fall back gracefully ──────────────────────────────
try:
    import rclpy
    from rclpy.node import Node as RosNode
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# ══════════════════════════════════════════════════════════════════
#  WAREHOUSE MAP  (matches standalone_sim.py exactly)
# ══════════════════════════════════════════════════════════════════
PICK_LOCATIONS = {
    # Row A (y=10 in Gazebo)
    "A1":(-22,10),"A2":(-16,10),"A3":(-10,10),"A4":(-4,10),
    "A5":(4,10),"A6":(10,10),"A7":(16,10),"A8":(22,10),
    # Row B
    "B1":(-22,1),"B2":(-16,1),"B3":(-10,1),"B4":(-4,1),
    "B5":(4,1),"B6":(10,1),"B7":(16,1),"B8":(22,1),
    # Row C
    "C1":(-22,-10),"C2":(-16,-10),"C3":(-10,-10),"C4":(-4,-10),
    "C5":(4,-10),"C6":(10,-10),"C7":(16,-10),"C8":(22,-10),
    # Dispatch/staging
    "DISPATCH":(28,0),"STAGING":(28,6),"INBOUND":(28,-6),
    # Charging
    "CHG1":(-26,-16),"CHG2":(-22,-16),"CHG3":(-18,-16),
}

TASK_TEMPLATES = [
    {"id":"T001","type":"pick","item":"Box-A01","from":"A1","to":"DISPATCH","priority":1},
    {"id":"T002","type":"pick","item":"Box-B03","from":"B3","to":"DISPATCH","priority":1},
    {"id":"T003","type":"pick","item":"Pallet-C2","from":"C2","to":"STAGING","priority":2},
    {"id":"T004","type":"pick","item":"Box-A05","from":"A5","to":"DISPATCH","priority":1},
    {"id":"T005","type":"pick","item":"Box-B07","from":"B7","to":"INBOUND","priority":3},
    {"id":"T006","type":"pick","item":"Box-C06","from":"C6","to":"STAGING","priority":2},
    {"id":"T007","type":"pick","item":"Pallet-A3","from":"A3","to":"INBOUND","priority":2},
    {"id":"T008","type":"pick","item":"Box-B01","from":"B1","to":"DISPATCH","priority":1},
    {"id":"T009","type":"patrol","from":"A1","to":"A8","priority":3},
    {"id":"T010","type":"pick","item":"Box-C04","from":"C4","to":"DISPATCH","priority":1},
]

# ══════════════════════════════════════════════════════════════════
#  PURE-PYTHON ROBOT STATE (mirrors simulation state for ROS node)
# ══════════════════════════════════════════════════════════════════
class RobotState:
    def __init__(self, rid):
        self.id       = rid
        self.name     = f"tugbot_{rid}"
        self.x        = 0.0
        self.y        = 0.0
        self.yaw      = 0.0
        self.battery  = 100.0
        self.state    = "idle"
        self.task_id  = None
        self.carrying = None

    def to_dict(self):
        return {
            "id": self.id, "name": self.name,
            "x": round(self.x,2), "y": round(self.y,2),
            "yaw": round(self.yaw,2),
            "battery": round(self.battery,1),
            "state": self.state,
            "task_id": self.task_id,
            "carrying": self.carrying,
        }

# ══════════════════════════════════════════════════════════════════
#  WAREHOUSE MANAGEMENT SYSTEM
# ══════════════════════════════════════════════════════════════════
class WarehouseManagementSystem:
    """Core WMS logic — works standalone or as part of ROS node."""

    def __init__(self, num_robots=4):
        self.robots     = {i: RobotState(i) for i in range(1, num_robots+1)}
        self.task_queue = list(TASK_TEMPLATES)
        self.active_tasks  = {}   # robot_id → task
        self.completed     = []
        self._lock         = threading.Lock()

    def assign_tasks(self):
        """Assign pending tasks to idle robots."""
        with self._lock:
            pending = [t for t in self.task_queue
                       if t["id"] not in self.active_tasks.values()]
            for rid, robot in self.robots.items():
                if robot.state == "idle" and pending:
                    task = pending.pop(0)
                    self.active_tasks[rid] = task["id"]
                    robot.state   = "moving"
                    robot.task_id = task["id"]
                    print(f"[WMS] Assigned {task['id']} ({task['item'] if 'item' in task else task['type']}) → robot {rid}")

    def update_odom(self, rid, x, y, yaw):
        with self._lock:
            r = self.robots.get(rid)
            if r:
                r.x, r.y, r.yaw = x, y, yaw

    def fleet_status_json(self):
        with self._lock:
            return json.dumps({
                "robots": [r.to_dict() for r in self.robots.values()],
                "tasks_queued": len(self.task_queue),
                "tasks_active": len(self.active_tasks),
                "tasks_done":   len(self.completed),
                "timestamp":    time.time(),
            }, indent=2)

    def task_queue_json(self):
        with self._lock:
            return json.dumps(self.task_queue, indent=2)


# ══════════════════════════════════════════════════════════════════
#  ROS 2 NODE
# ══════════════════════════════════════════════════════════════════
if HAS_ROS:
    class WarehouseManagerNode(RosNode):
        def __init__(self):
            super().__init__('warehouse_manager')
            self.declare_parameter('num_robots', 4)
            n = self.get_parameter('num_robots').value
            self.wms = WarehouseManagementSystem(num_robots=n)

            # Publishers
            self.pub_status = self.create_publisher(String, '/warehouse/fleet_status', 10)
            self.pub_queue  = self.create_publisher(String, '/warehouse/task_queue',   10)
            self.goal_pubs  = {
                i: self.create_publisher(PoseStamped, f'/tugbot_{i}/goal_pose', 10)
                for i in range(1, n+1)
            }

            # Odom subscribers
            for i in range(1, n+1):
                self.create_subscription(
                    Odometry, f'/model/tugbot_{i}/odometry',
                    lambda msg, rid=i: self._odom_cb(rid, msg), 10
                )

            # Order subscriber
            self.create_subscription(String, '/warehouse/dispatch_order', self._order_cb, 10)

            # Timer: assign tasks + broadcast status every 1s
            self.create_timer(1.0, self._timer_cb)
            self.get_logger().info('Warehouse Management System started')

        def _odom_cb(self, rid, msg):
            x   = msg.pose.pose.position.x
            y   = msg.pose.pose.position.y
            q   = msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.wms.update_odom(rid, x, y, yaw)

        def _order_cb(self, msg):
            try:
                order = json.loads(msg.data)
                self.wms.task_queue.append(order)
                self.get_logger().info(f'New order received: {order}')
            except Exception as e:
                self.get_logger().error(f'Bad order JSON: {e}')

        def _timer_cb(self):
            self.wms.assign_tasks()
            self.pub_status.publish(String(data=self.wms.fleet_status_json()))
            self.pub_queue.publish(String(data=self.wms.task_queue_json()))
            self._send_goals()

        def _send_goals(self):
            for rid, task_id in list(self.wms.active_tasks.items()):
                task = next((t for t in self.wms.task_queue if t["id"]==task_id), None)
                if not task: continue
                loc = PICK_LOCATIONS.get(task.get("from","DISPATCH"), (0,0))
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp    = self.get_clock().now().to_msg()
                goal.pose.position.x = float(loc[0])
                goal.pose.position.y = float(loc[1])
                goal.pose.position.z = 0.0
                goal.pose.orientation.w = 1.0
                self.goal_pubs[rid].publish(goal)


def main_ros(args=None):
    rclpy.init(args=args)
    node = WarehouseManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ══════════════════════════════════════════════════════════════════
#  STANDALONE (no ROS) — prints status to terminal
# ══════════════════════════════════════════════════════════════════
def main_standalone():
    print("=" * 60)
    print("  Warehouse Management System — Standalone Mode")
    print("  (No ROS installation required)")
    print("=" * 60)
    wms = WarehouseManagementSystem(num_robots=4)

    # Simulate robot odom updates
    def fake_odom():
        while True:
            for rid, r in wms.robots.items():
                r.x += random.uniform(-0.1, 0.1)
                r.y += random.uniform(-0.1, 0.1)
                r.battery = max(0, r.battery - 0.01)
            time.sleep(0.5)

    threading.Thread(target=fake_odom, daemon=True).start()

    tick = 0
    while True:
        wms.assign_tasks()
        if tick % 5 == 0:
            status = json.loads(wms.fleet_status_json())
            print(f"\n[{time.strftime('%H:%M:%S')}] Fleet Status:")
            for r in status["robots"]:
                print(f"  Robot {r['id']:} | state:{r['state']:8} | bat:{r['battery']:.1f}% | pos:({r['x']:.1f},{r['y']:.1f})")
            print(f"  Tasks queued:{status['tasks_queued']} active:{status['tasks_active']} done:{status['tasks_done']}")
        tick += 1
        time.sleep(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--standalone", action="store_true",
                        help="Run without ROS (terminal output only)")
    args = parser.parse_args()

    if args.standalone or not HAS_ROS:
        main_standalone()
    else:
        main_ros()
