#!/usr/bin/env python3
"""
gazebo_sim.py  —  Warehouse Simulation v5: Flask + Gazebo Harmonic Bridge
=========================================================================
Runs the full Flask / Three.js web dashboard (standalone_sim.py logic)
AND optionally bridges all robot / arm / conveyor state into Gazebo via
ROS 2 Humble + Gazebo Harmonic transport.

Architecture
────────────
  ┌──────────────────────────────┐
  │  Python simulation core      │  A* pathfinding, robot FSM,
  │  (Fleet / Robot / Arm /      │  arm 10-state machine,
  │   Conveyor classes)          │  collision avoidance
  └──────────┬───────────────────┘
             │ in-process
  ┌──────────▼───────────────────┐
  │  Flask + Socket.IO dashboard │  http://localhost:5001
  │  (Three.js 3D viewer)        │
  └──────────┬───────────────────┘
             │ optional, runs in daemon thread
  ┌──────────▼───────────────────┐
  │  GazeboBridge (ROS 2 node)   │  Publishes poses / joints
  │  ─────────────────────────── │  to Gazebo via:
  │  /model/tugbot_N/cmd_vel     │    gz_set_model_pose() service
  │  /model/forklift_N/cmd_vel   │    /arm/{joint}_cmd topics
  │  /arm/joint_states           │    /conveyor/state topic
  └──────────────────────────────┘

Running (no ROS – web dashboard only):
  python3 gazebo_sim.py

Running (with ROS 2 + Gazebo):
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  python3 gazebo_sim.py --ros

Launching everything together:
  ros2 launch warehouse_simulation warehouse_full.launch.py
"""

import math, heapq, time, json, threading, random, re, argparse, sys, os

# ── Optional ROS 2 imports ────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node as RosNode
    from geometry_msgs.msg import Twist, Pose, PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64, String, Bool
    from sensor_msgs.msg import JointState, LaserScan
    from tf2_ros import TransformBroadcaster
    import tf_transformations
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# ── Flask / Socket.IO ─────────────────────────────────────────────
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit

# ══════════════════════════════════════════════════════════════════
#  A* GRID  (60 × 36 m world, 0.5 m cells)
#  Gazebo uses (X=left-right, Y=forward-back, Z=up)
#  standalone_sim uses (x=X, z=Y for top-down)
#  Coordinate mapping: sim.x → gz.x,  sim.z → gz.y,  height = gz.z
# ══════════════════════════════════════════════════════════════════
class Grid:
    CELL, W, H = 0.5, 60, 36
    SHELVES = [
        (-24, 8,-20,12),(-18, 8,-14,12),(-12, 8,-8,12),(-6, 8,-2,12),
        ( 2, 8, 6,12),  ( 8, 8,12,12), (14, 8,18,12), (20, 8,24,12),
        (-24,-1,-20, 3),(-18,-1,-14,3),(-12,-1,-8, 3),(-6,-1,-2, 3),
        ( 2,-1, 6, 3),  ( 8,-1,12, 3), (14,-1,18, 3), (20,-1,24, 3),
        (-24,-12,-20,-8),(-18,-12,-14,-8),(-12,-12,-8,-8),(-6,-12,-2,-8),
        ( 2,-12, 6,-8),  ( 8,-12,12,-8),(14,-12,18,-8),(20,-12,24,-8),
        (25,-6, 31, 6),           # dispatch/conveyor block
    ]
    def __init__(self):
        self.cols = int(self.W / self.CELL)
        self.rows = int(self.H / self.CELL)
        self.g = [[0]*self.cols for _ in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                wx = -self.W/2 + (c+.5)*self.CELL
                wy = -self.H/2 + (r+.5)*self.CELL
                for s in self.SHELVES:
                    if s[0]-.3 <= wx <= s[2]+.3 and s[1]-.3 <= wy <= s[3]+.3:
                        self.g[r][c] = 1; break
    def wc(self, wx, wy):
        return (max(0, min(int((wx+self.W/2)/self.CELL), self.cols-1)),
                max(0, min(int((wy+self.H/2)/self.CELL), self.rows-1)))
    def cw(self, c, r):
        return (-self.W/2+(c+.5)*self.CELL, -self.H/2+(r+.5)*self.CELL)
    def plan(self, s, g):
        sc,sr = self.wc(*s); gc,gr = self.wc(*g)
        if self.g[sr][sc]: sc,sr = self.wc(s[0]+0.6, s[1])
        if self.g[gr][gc]: gc,gr = self.wc(g[0]-0.6, g[1])
        open_set = [(0, sc, sr, [])]; vis = {}
        while open_set:
            cost, c, r, path = heapq.heappop(open_set)
            if (c,r) in vis: continue
            vis[(c,r)] = True; path = path + [self.cw(c,r)]
            if c==gc and r==gr: return path
            for dc,dr in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nc, nr = c+dc, r+dr
                if 0 <= nc < self.cols and 0 <= nr < self.rows and not self.g[nr][nc]:
                    h = math.hypot(nc-gc, nr-gr)
                    step = 1.414 if dc and dr else 1.0
                    heapq.heappush(open_set, (cost+step+h, nc, nr, path))
        return []

GRID = Grid()

# ══════════════════════════════════════════════════════════════════
#  WAYPOINTS
# ══════════════════════════════════════════════════════════════════
WP = {
    "A1":(-22,6),"A2":(-16,6),"A3":(-10,6),"A4":(-4,6),
    "A5":(4,6),  "A6":(10,6), "A7":(16,6), "A8":(22,6),
    "B1":(-22,-3),"B2":(-16,-3),"B3":(-10,-3),"B4":(-4,-3),
    "B5":(4,-3), "B6":(10,-3),"B7":(16,-3), "B8":(22,-3),
    "C1":(-22,-14),"C2":(-16,-14),"C3":(-10,-14),"C4":(-4,-14),
    "C5":(4,-14),"C6":(10,-14),"C7":(16,-14),"C8":(22,-14),
    "DISPATCH":(22,0),"DROP":(22,0),"STAGING":(22,6),"INBOUND":(22,-6),
    "CONVEYOR":(28.5,0),
    "CHG1":(-26,-16),"CHG2":(-22,-16),"CHG3":(-18,-16),
    "HOME1":(-26,14),"HOME2":(-26,10),"HOME3":(-26,6),
    "HOME4":(-26,2), "HOME5":(-26,-2),"HOME6":(-26,-6),
}

# ══════════════════════════════════════════════════════════════════
#  CONVEYOR + DROP ZONE
# ══════════════════════════════════════════════════════════════════
class Conveyor:
    def __init__(self):
        self.belt_offset = 0.0
        self.items = []
    def add(self, name):
        self.items.append({"z": -5.5, "name": name, "age": 0.0})
    def tick(self, dt):
        self.belt_offset = (self.belt_offset + dt*1.4) % 2.0
        for it in self.items: it["z"] += 1.4*dt; it["age"] += dt
        self.items = [i for i in self.items if i["z"] < 7.0]
    def to_dict(self):
        return {"belt_offset": round(self.belt_offset, 3),
                "items": [{"z": round(i["z"],2),"name":i["name"]} for i in self.items]}

class DropZone:
    def __init__(self): self.pending = []
    def add(self, name): self.pending.append(name)
    def take(self): return self.pending.pop(0) if self.pending else None

DROP_ZONE = DropZone()
CONVEYOR  = Conveyor()

# ══════════════════════════════════════════════════════════════════
#  SENSORS
# ══════════════════════════════════════════════════════════════════
class Sensor:
    MAX_RANGE = 8.0
    def __init__(self):
        self.ultrasonic = {"front":8.0,"rear":8.0,"left":8.0,"right":8.0}
        self.ir = {"floor":"clear","line":False}
        self.impact = False

# ══════════════════════════════════════════════════════════════════
#  ROBOT
# ══════════════════════════════════════════════════════════════════
class Robot:
    SPEED = 3.0; DRAIN = 0.010; CHARGE_SPD = 7.0
    RADIUS = {"tugbot": 0.65, "forklift": 0.95}

    def __init__(self, rid, x, z, color, name, vtype="tugbot"):
        self.id = rid; self.x = float(x); self.z = float(z)
        self.yaw = 0.0; self.battery = random.uniform(55,100)
        self.color = color; self.name = name; self.vtype = vtype
        self.state = "idle"; self.path = []; self.path_idx = 0
        self.task = None; self.carrying = None
        self.fork_height = 0.0
        self.manual_vx = 0.0; self.manual_vz = 0.0
        self.sensor = Sensor(); self.log_entries = []
        self._stop_timer = 0.0

    def log(self, msg):
        self.log_entries.append({"t": time.strftime("%H:%M:%S"),"msg":msg,"name":self.name})
        if len(self.log_entries) > 80: self.log_entries.pop(0)

    def go_to(self, wx, wz, reason=""):
        path = GRID.plan((self.x, self.z), (wx, wz))
        if path:
            self.path = path; self.path_idx = 0; self.state = "moving"
            self.log(f"→ {reason or f'({wx:.1f},{wz:.1f})'}")
            return True
        self.log(f"⚠ No path to ({wx:.1f},{wz:.1f})")
        return False

    def update_sensors(self, all_robots):
        us = {"front":8.0,"rear":8.0,"left":8.0,"right":8.0}
        for other in all_robots:
            if other.id == self.id: continue
            dx = other.x - self.x; dz = other.z - self.z
            dist = math.hypot(dx, dz)
            if dist > 8.0: continue
            rel = math.atan2(dz, dx) - self.yaw
            while rel >  math.pi: rel -= 2*math.pi
            while rel < -math.pi: rel += 2*math.pi
            if   -math.pi/4 < rel < math.pi/4:      us["front"] = min(us["front"], dist)
            elif  rel >  3*math.pi/4 or rel < -3*math.pi/4: us["rear"] = min(us["rear"], dist)
            elif  math.pi/4 < rel < 3*math.pi/4:    us["left"]  = min(us["left"],  dist)
            else:                                    us["right"] = min(us["right"], dist)
        us["front"] = min(us["front"], abs(30  - self.x))
        us["rear"]  = min(us["rear"],  abs(-30 - self.x))
        self.sensor.ultrasonic = {k: round(v,2) for k,v in us.items()}
        near = None
        for nm, pos in WP.items():
            if re.match(r'^[ABC]\d', nm) and math.hypot(self.x-pos[0], self.z-pos[1]) < 2.0:
                near = nm; break
        self.sensor.ir = {"floor": f"near {near}" if near else "clear","line": near is not None}
        self.sensor.impact = any(v < 1.4 for v in self.sensor.ultrasonic.values())

    def _robot_radius(self): return self.RADIUS.get(self.vtype, 0.65)

    def _avoidance(self, all_robots):
        fx, fz = 0.0, 0.0
        min_gap = 999.0
        my_r = self._robot_radius()
        for o in all_robots:
            if o.id == self.id: continue
            dx = self.x - o.x; dz = self.z - o.z
            dist = math.hypot(dx, dz)
            if dist < 0.01: dist = 0.01
            combined_r = my_r + o._robot_radius()
            gap = dist - combined_r
            min_gap = min(min_gap, gap)
            if dist > combined_r * 3.8: continue
            priority = 1.6 if self.id > o.id else 0.35
            overlap = combined_r * 3.0 - dist
            if overlap > 0:
                strength = priority * overlap / (combined_r * 3.0)
                fx += (dx/dist)*strength; fz += (dz/dist)*strength
            if gap < 0:
                push = 4.0 * abs(gap) / combined_r
                fx += (dx/dist)*push; fz += (dz/dist)*push
        # Shelf wall repulsion
        wall_margin = my_r + 0.45
        for sx0,sz0,sx1,sz1 in GRID.SHELVES:
            cx = max(sx0, min(self.x, sx1)); cz = max(sz0, min(self.z, sz1))
            dx = self.x - cx; dz = self.z - cz
            wall_dist = math.hypot(dx, dz)
            if wall_dist < 0.01:
                dx = self.x - (sx0+sx1)*0.5; dz = self.z - (sz0+sz1)*0.5
                wall_dist = max(math.hypot(dx, dz), 0.01)
            if wall_dist > wall_margin + 1.2: continue
            overlap = wall_margin - wall_dist
            if overlap > 0:
                strength = 3.5 * overlap / wall_margin
                fx += (dx/wall_dist)*strength; fz += (dz/wall_dist)*strength
        stop_gap = 0.35; slow_gap = 2.2
        if min_gap <= stop_gap: speed = 0.0
        elif min_gap < slow_gap: speed = (min_gap - stop_gap)/(slow_gap - stop_gap)
        else: speed = 1.0
        return fx, fz, speed

    def _clamp_from_shelves(self):
        r = self._robot_radius() + 0.05
        for sx0,sz0,sx1,sz1 in GRID.SHELVES:
            ex0,ez0 = sx0-r, sz0-r; ex1,ez1 = sx1+r, sz1+r
            if ex0 < self.x < ex1 and ez0 < self.z < ez1:
                opts = [(abs(self.x-ex0),'x',ex0),(abs(self.x-ex1),'x',ex1),
                        (abs(self.z-ez0),'z',ez0),(abs(self.z-ez1),'z',ez1)]
                _,axis,val = min(opts)
                if axis=='x': self.x = val
                else:         self.z = val

    def _replan_around(self, all_robots):
        if not self.path: return
        dest = self.path[-1]; blocked = []
        for o in all_robots:
            if o.id == self.id: continue
            if math.hypot(o.x-self.x, o.z-self.z) > 10.0: continue
            for ddx in range(-2,3):
                for ddz in range(-2,3):
                    wx = o.x + ddx*GRID.CELL; wz = o.z + ddz*GRID.CELL
                    c,r = GRID.wc(wx, wz)
                    if 0<=c<GRID.cols and 0<=r<GRID.rows and GRID.g[r][c]==0:
                        GRID.g[r][c]=2; blocked.append((c,r))
        new = GRID.plan((self.x,self.z), dest)
        for c,r in blocked:
            if GRID.g[r][c]==2: GRID.g[r][c]=0
        if new and len(new)>2:
            self.path=new; self.path_idx=0; self.log("↺ Re-routing around traffic")

    def tick(self, dt, all_robots=None):
        if self.state == "charging":
            self.battery = min(100, self.battery + self.CHARGE_SPD*dt)
            if self.battery >= 100: self.state="idle"; self.log("✓ Fully charged")
            return
        if self.state == "manual":
            spd = 3.6
            nx = max(-29, min(29, self.x + self.manual_vx*spd*dt))
            nz = max(-17, min(17, self.z + self.manual_vz*spd*dt))
            self.battery = max(0, self.battery - self.DRAIN*math.hypot(nx-self.x,nz-self.z))
            if self.manual_vx or self.manual_vz:
                self.yaw = math.atan2(self.manual_vz, self.manual_vx)
            self.x,self.z = nx,nz; return

        if self.state=="moving" and self.path:
            fx,fz,speed = self._avoidance(all_robots) if all_robots else (0,0,1)
            if speed < 0.05:
                push = 1.0
                self.x = max(-29, min(29, self.x + fx*push*dt))
                self.z = max(-17, min(17, self.z + fz*push*dt))
                self._clamp_from_shelves()
                self._stop_timer += dt
                if self._stop_timer > 2.5:
                    self._replan_around(all_robots or [])
                    self._stop_timer = 0.0
                return
            self._stop_timer = 0.0
            tx,tz = self.path[self.path_idx]; dx,dz = tx-self.x, tz-self.z
            dist = math.hypot(dx,dz)
            if dist < 0.12:
                self.path_idx += 1
                if self.path_idx >= len(self.path):
                    self.path=[]; self.state="idle"; self._on_arrive()
            else:
                step = min(self.SPEED*dt*speed, dist)
                self.x = max(-29, min(29, self.x + dx/dist*step + fx*dt*0.3))
                self.z = max(-17, min(17, self.z + dz/dist*step + fz*dt*0.3))
                self._clamp_from_shelves()
                self.yaw = math.atan2(dz,dx)
                self.battery = max(0, self.battery - self.DRAIN*step)
                if self.battery < 5 and self.state != "charging":
                    self.log("⚡ CRITICAL battery!")
                    chg = random.choice(["CHG1","CHG2","CHG3"])
                    self.task={"type":"charge"}; self.go_to(*WP[chg],"EMERGENCY CHG")

        elif self.state == "picking":
            if not hasattr(self,'_ptimer'): self._ptimer = 2.4
            self._ptimer -= dt
            if self.vtype=="forklift": self.fork_height = min(1.6, self.fork_height + dt*0.9)
            if self._ptimer <= 0:
                del self._ptimer
                self.log(f"📦 Picked {self.carrying}")
                if self.vtype=="forklift": self.fork_height = 0.8
                drop = self.task.get("drop", WP["STAGING"]) if self.task else WP["DISPATCH"]
                self.go_to(*drop, "→ drop zone")
                if self.task: self.task["type"] = "drop"

        elif self.state == "dropping":
            if not hasattr(self,'_dtimer'): self._dtimer = 1.6
            self._dtimer -= dt
            if self.vtype=="forklift": self.fork_height = max(0.05, self.fork_height - dt*0.9)
            if self._dtimer <= 0:
                del self._dtimer
                itm = self.carrying; self.log(f"✓ Dropped {itm}")
                if self.task and self.task.get("drop") in (WP["DISPATCH"], WP["DROP"]):
                    DROP_ZONE.add(itm)
                self.carrying=None; self.task=None; self.state="idle"
                if self.vtype=="forklift": self.fork_height = 0.05

    def _on_arrive(self):
        if self.task:
            t = self.task.get("type")
            if t=="pick":
                self.state="picking"; self.carrying=self.task.get("item","Box")
                self.log(f"⏳ Picking {self.carrying}…")
            elif t=="drop": self.state="dropping"; self.log(f"⏳ Dropping {self.carrying}…")
            elif t=="charge": self.state="charging"; self.task=None; self.log("⚡ Charging…")
        else: self.state="idle"

    def to_dict(self):
        return {"id":self.id,"name":self.name,"x":round(self.x,2),"z":round(self.z,2),
                "yaw":round(self.yaw,3),"battery":round(self.battery,1),"state":self.state,
                "color":self.color,"carrying":self.carrying,"vtype":self.vtype,
                "fork_height":round(self.fork_height,3),
                "path":self.path[self.path_idx:self.path_idx+60],
                "sensor":{"ultrasonic":self.sensor.ultrasonic,"ir":self.sensor.ir,
                          "impact":self.sensor.impact}}

# ══════════════════════════════════════════════════════════════════
#  ROBOTIC ARM  (10-state industrial arm, identical to standalone_sim)
# ══════════════════════════════════════════════════════════════════
class RoboticArm:
    P = {
        "rest":      ( 0.00, 0.55, 0.40, 0.00),
        "over_drop": (-1.57, 0.85, 0.70, 0.10),
        "at_drop":   (-1.57, 0.18, 0.06, 0.00),
        "lifted":    (-1.57, 1.05, 0.95, 0.20),
        "mid_swing": ( 0.00, 1.10, 0.90, 0.15),
        "over_belt": ( 1.45, 0.80, 0.55, 0.00),
        "at_belt":   ( 1.45, 0.28, 0.12, 0.00),
    }
    def __init__(self):
        self.x=25.5; self.z=0.0
        self.base=0.0; self.shoulder=0.55; self.elbow=0.40; self.wrist=0.0
        self.gripper=0.0; self.state="idle"; self._timer=0.0
        self.has_item=None; self.grip_x=self.x; self.grip_y=2.5; self.grip_z=self.z

    def _a(self, cur, tgt, spd, dt):
        d = tgt - cur
        if abs(d) < 0.003: return tgt
        return cur + math.copysign(min(abs(d), spd*dt), d)

    def _pose(self, name, dt, spd=1.6):
        b,s,e,w = self.P[name]
        self.base     = self._a(self.base,     b, spd,     dt)
        self.shoulder = self._a(self.shoulder, s, spd,     dt)
        self.elbow    = self._a(self.elbow,    e, spd*0.9, dt)
        self.wrist    = self._a(self.wrist,    w, spd*0.7, dt)
        return (abs(self.base-b)<0.04 and abs(self.shoulder-s)<0.04 and abs(self.elbow-e)<0.04)

    def _compute_grip_pos(self):
        L1,L2 = 1.45,1.05; sy = 0.75
        reach  = L1*math.sin(self.shoulder) + L2*math.sin(self.shoulder+self.elbow)
        height = L1*math.cos(self.shoulder) + L2*math.cos(self.shoulder+self.elbow)
        self.grip_x = round(self.x + math.cos(self.base)*reach, 2)
        self.grip_y = round(max(0.3, sy+height), 2)
        self.grip_z = round(self.z + math.sin(self.base)*reach, 2)

    def auto_pick(self, ix, iz, name):
        if self.state != "idle": return False
        self.has_item=name; self.state="open_grip"; self._timer=0.0; return True

    def command(self, action):
        if action=="arm_pick" and self.state=="idle":
            self.has_item="Manual-Item"; self.state="open_grip"; self._timer=0.0
        elif action=="arm_drop" and self.state not in ("idle",):
            if self.has_item: CONVEYOR.add(self.has_item); self.has_item=None
            self.state="return"; self._timer=0.0

    def tick(self, dt):
        self._timer += dt; t = time.time()
        if self.state=="idle":
            self.base     = 0.22*math.sin(t*0.24)
            self.shoulder = 0.55+0.10*math.sin(t*0.38)
            self.elbow    = 0.40+0.08*math.sin(t*0.51)
            self.wrist    = 0.12*math.sin(t*0.67)
            self.gripper  = max(0.0, self.gripper - dt*1.2)
            self._compute_grip_pos(); return
        if self.state=="open_grip":
            self.gripper = max(0.0, self.gripper - dt*3.5)
            if self.gripper < 0.05 and self._timer > 0.35:
                self.state="swing_drop"; self._timer=0.0
        elif self.state=="swing_drop":
            done = self._pose("over_drop", dt, 1.8)
            if done and self._timer > 0.6: self.state="descend"; self._timer=0.0
        elif self.state=="descend":
            done = self._pose("at_drop", dt, 1.0)
            if done and self._timer > 0.4: self.state="close_grip"; self._timer=0.0
        elif self.state=="close_grip":
            self.gripper = min(1.0, self.gripper + dt*2.8)
            if self.gripper >= 0.95 and self._timer > 0.5:
                self.state="ascend"; self._timer=0.0
        elif self.state=="ascend":
            done = self._pose("lifted", dt, 1.4)
            if done and self._timer > 0.5: self.state="swing_belt"; self._timer=0.0
        elif self.state=="swing_belt":
            if self._timer < 1.0: self._pose("mid_swing", dt, 2.0)
            else:
                done = self._pose("over_belt", dt, 1.8)
                if done and self._timer > 1.8: self.state="lower_belt"; self._timer=0.0
        elif self.state=="lower_belt":
            done = self._pose("at_belt", dt, 1.0)
            if done and self._timer > 0.4: self.state="release"; self._timer=0.0
        elif self.state=="release":
            self.gripper = max(0.0, self.gripper - dt*3.5)
            if self._timer > 0.3 and self.has_item:
                CONVEYOR.add(self.has_item); self.has_item=None
            if self.gripper < 0.05 and self._timer > 0.8:
                self.state="return"; self._timer=0.0
        elif self.state=="return":
            self._pose("over_belt", dt, 1.6)
            if self._timer > 0.5:
                done2 = self._pose("rest", dt, 1.6)
                if done2 and self._timer > 1.4: self.state="idle"; self._timer=0.0
        self._compute_grip_pos()

    def to_dict(self):
        return {"x":self.x,"z":self.z,
                "base":round(self.base,3),"shoulder":round(self.shoulder,3),
                "elbow":round(self.elbow,3),"wrist":round(self.wrist,3),
                "gripper":round(self.gripper,3),"state":self.state,
                "has_item":self.has_item,
                "grip_x":self.grip_x,"grip_y":self.grip_y,"grip_z":self.grip_z}

# ══════════════════════════════════════════════════════════════════
#  TASK CATALOG
# ══════════════════════════════════════════════════════════════════
BOX_TASKS = [
    {"type":"pick","item":"Box-A01","pick":WP["A1"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-A05","pick":WP["A5"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-B03","pick":WP["B3"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-B07","pick":WP["B7"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-A03","pick":WP["A3"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-B05","pick":WP["B5"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-A07","pick":WP["A7"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-B01","pick":WP["B1"],"drop":WP["DROP"],"for":"tugbot"},
]
CRATE_TASKS = [
    {"type":"pick","item":"Crate-C02","pick":WP["C2"],"drop":WP["STAGING"],"for":"forklift"},
    {"type":"pick","item":"Crate-C06","pick":WP["C6"],"drop":WP["STAGING"],"for":"forklift"},
    {"type":"pick","item":"Crate-C04","pick":WP["C4"],"drop":WP["STAGING"],"for":"forklift"},
    {"type":"pick","item":"Crate-C08","pick":WP["C8"],"drop":WP["STAGING"],"for":"forklift"},
]
ALL_TASKS = BOX_TASKS + CRATE_TASKS

# ══════════════════════════════════════════════════════════════════
#  COMMAND PARSER  (unchanged from standalone_sim)
# ══════════════════════════════════════════════════════════════════
def parse_cmd(raw):
    s = raw.lower().strip()
    if re.search(r'\b(estop|e-stop|emergency|all stop|stop all)\b',s): return {'action':'estop'}
    if re.search(r'\b(pause|resume|unpause)\b',s): return {'action':'pause'}
    if re.search(r'\b(cam|camera|view)\b.*\b(top|overhead|bird|down)\b',s): return {'action':'camera','mode':'top'}
    m = re.search(r'\b(cam|camera|view)\b.*\b(follow|track)\b.*?(\d)',s)
    if m: return {'action':'camera','mode':'follow','rid':int(m.group(3))-1}
    if re.search(r'\b(iso|isometric)\b',s): return {'action':'camera','mode':'iso'}
    if re.search(r'\b(dock|side|front)\b.*\b(view|cam)\b',s): return {'action':'camera','mode':'dock'}
    if re.search(r'\b(forklift|fl).*(pov|driver|cab)',s):
        m2=re.search(r'(\d)',s); n=int(m2.group(1)) if m2 else 1
        return {'action':'camera','mode':f'fl{n}pov'}
    if re.search(r'\b(arm|conveyor).*(cam|view)',s): return {'action':'camera','mode':'arm'}
    for i,mode in enumerate(['free','top','follow','iso','dock','fl1pov','fl2pov','arm']):
        if re.match(rf'^cam\s*{i+1}',s): return {'action':'camera','mode':mode,'rid':0}
    if re.search(r'\b(show|hide|toggle)\b.*\bpath',s): return {'action':'toggle_paths'}
    rid=None; vtype=None
    m=re.search(r'\b(forklift|lift)\s*(\d)',s)
    if m: vtype='forklift'; rid=2+int(m.group(2))
    m2=re.search(r'\b(robot|tugbot|bot)\s*(\d)',s)
    if m2 and vtype is None: vtype='tugbot'; rid=int(m2.group(2))-1
    m3=re.search(r'\b(arm|robotic.arm)\b',s)
    if m3 and vtype is None: vtype='arm'; rid=6
    if rid is None:
        m4=re.match(r'(\d)\s',s)
        if m4: rid=int(m4.group(1))-1; vtype='any'
    if vtype in ('forklift','any') and rid is not None:
        if re.search(r'\b(lift|raise|forks up)\b',s): return {'action':'fork_up','rid':rid}
        if re.search(r'\b(lower|forks down|drop forks)\b',s): return {'action':'fork_down','rid':rid}
    mc=re.search(r'\(?\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)?',s)
    if mc and rid is not None and re.search(r'\b(go|goto|navigate|move|send|drive)\b',s):
        return {'action':'goto','rid':rid,'x':float(mc.group(1)),'z':float(mc.group(2)),'loc':'coords'}
    m=re.search(r'\b(go|goto|navigate|move|send|drive|take)\b.*?\b([A-C][1-8]|dispatch|staging|inbound|drop|chg\d?|home\d?)\b',s,re.I)
    if m and rid is not None:
        loc=m.group(2).upper()
        if loc in WP: return {'action':'goto','rid':rid,'x':WP[loc][0],'z':WP[loc][1],'loc':loc}
    m=re.search(r'\b([A-C][1-8]|dispatch|staging|inbound)\b',s,re.I)
    if m and rid is not None and not re.search(r'\b(pick|drop|patrol)\b',s):
        loc=m.group(1).upper()
        if loc in WP: return {'action':'goto','rid':rid,'x':WP[loc][0],'z':WP[loc][1],'loc':loc}
    m=re.search(r'\b(pick|pickup|collect|get|fetch|grab)\b.*?\b([A-C][1-8])\b',s,re.I)
    if m and rid is not None:
        loc=m.group(2).upper()
        if loc in WP:
            item=f"Crate-{loc}" if loc.startswith('C') else f"Box-{loc}"
            return {'action':'pick','rid':rid,'loc':loc,'item':item}
    m=re.search(r'\b(drop|deliver|place|put|bring)\b.*?\b(dispatch|staging|inbound|drop)\b',s,re.I)
    if m and rid is not None:
        return {'action':'drop','rid':rid,'loc':m.group(2).upper()}
    if re.search(r'\b(charge|charging|battery|recharge)\b',s) and rid is not None:
        return {'action':'charge','rid':rid}
    if vtype=='arm':
        if re.search(r'\b(pick|grab|grasp)\b',s): return {'action':'arm_pick','rid':rid}
        if re.search(r'\b(drop|place|release)\b',s): return {'action':'arm_drop','rid':rid}
    if re.search(r'\b(help|commands|\?)\b',s): return {'action':'help'}
    return {'action':'unknown','raw':raw}

# ══════════════════════════════════════════════════════════════════
#  FLEET
# ══════════════════════════════════════════════════════════════════
class Fleet:
    def __init__(self):
        self.robots = [
            Robot(0,-26,14,"#00c8ff","TugBot-1","tugbot"),
            Robot(1,-26,10,"#7fff00","TugBot-2","tugbot"),
            Robot(2,-26, 6,"#ff6b35","TugBot-3","tugbot"),
            Robot(3,-26, 2,"#ffd700","Forklift-1","forklift"),
            Robot(4,-26,-2,"#ff9500","Forklift-2","forklift"),
            Robot(5,-26,-6,"#ff3af0","TugBot-M","tugbot"),
        ]
        self.arm = RoboticArm()
        self.task_queue = list(ALL_TASKS)
        self.paused = False
        self._lock = threading.Lock()
        self._t = time.time()
        threading.Thread(target=self._loop, daemon=True).start()
        for i,r in enumerate(self.robots[:5]):
            threading.Timer(i*1.5, lambda rb=r: self._assign(rb)).start()

    def _assign(self, robot):
        if robot.state != "idle" or robot.battery < 20:
            if robot.battery < 20 and robot.state == "idle":
                chg = random.choice(["CHG1","CHG2","CHG3"])
                robot.task = {"type":"charge"}; robot.go_to(*WP[chg],"charge")
                robot.log(f"⚡ Battery low ({robot.battery:.0f}%) → charging")
            return
        eligible = [t for t in self.task_queue if t.get("for","any") in (robot.vtype,"any")]
        if eligible:
            task = eligible[0]; self.task_queue.remove(task)
            robot.task = {**task,"type":"pick","drop":task.get("drop",WP["DROP"])}
            robot.go_to(*task["pick"], f"pick {task['item']}")

    def _loop(self):
        while True:
            now = time.time(); dt = min(now-self._t, 0.1); self._t = now
            if not self.paused:
                with self._lock:
                    for r in self.robots: r.update_sensors(self.robots)
                    for r in self.robots: r.tick(dt, self.robots)
                    self.arm.tick(dt); CONVEYOR.tick(dt)
                    if self.arm.state=="idle" and DROP_ZONE.pending:
                        item = DROP_ZONE.take(); self.arm.auto_pick(22.0, 0.0, item)
                    for r in self.robots[:5]:
                        if r.state == "idle":
                            if not self.task_queue: self.task_queue = list(ALL_TASKS)
                            self._assign(r)
            time.sleep(0.04)

    def snapshot(self):
        with self._lock:
            return ([r.to_dict() for r in self.robots],
                    self.arm.to_dict(), CONVEYOR.to_dict())

    def dispatch(self, a):
        action = a.get("action")
        with self._lock:
            if action=="estop":
                for r in self.robots:
                    r.path=[]; r.state="idle"; r.manual_vx=0; r.manual_vz=0; r.log("⛔ E-STOP")
                return "⛔ All robots stopped"
            if action=="pause":
                self.paused = not self.paused
                return f"{'⏸ Paused' if self.paused else '▶ Resumed'}"
            rid = a.get("rid")
            if action=="goto" and rid is not None and 0<=rid<len(self.robots):
                r=self.robots[rid]; r.task=None; r.go_to(a["x"],a["z"],a.get("loc",""))
                return f"✓ {r.name} → {a.get('loc','')}"
            if action=="pick" and rid is not None and 0<=rid<len(self.robots):
                r=self.robots[rid]; loc=a["loc"]; item=a.get("item",f"Box-{loc}")
                if r.vtype=="forklift" and not item.startswith("Crate"):
                    return f"⚠ {r.name} handles CRATES only."
                if r.vtype=="tugbot" and item.startswith("Crate"):
                    return f"⚠ {r.name} can't lift heavy CRATES. Use a Forklift!"
                r.task={"type":"pick","item":item,"drop":WP.get("DROP",(22,0))}
                if item.startswith("Crate"): r.task["drop"]=WP["STAGING"]
                r.go_to(*WP[loc],f"pick {item}")
                return f"✓ {r.name} → pick {item} @ {loc}"
            if action=="drop" and rid is not None and 0<=rid<len(self.robots):
                r=self.robots[rid]; loc=a.get("loc","DISPATCH")
                if r.carrying:
                    r.go_to(*WP.get(loc,WP["DISPATCH"]),f"drop {r.carrying}")
                    return f"✓ {r.name} dropping at {loc}"
                return f"⚠ {r.name} not carrying anything"
            if action=="charge" and rid is not None and 0<=rid<len(self.robots):
                r=self.robots[rid]; chg=random.choice(["CHG1","CHG2","CHG3"])
                r.task={"type":"charge"}; r.go_to(*WP[chg],"charge")
                return f"⚡ {r.name} → charging"
            if action=="fork_up" and rid is not None:
                r=self.robots[rid]; r.fork_height=min(2.0,r.fork_height+0.8)
                return f"↑ {r.name} forks → {r.fork_height:.1f}m"
            if action=="fork_down" and rid is not None:
                r=self.robots[rid]; r.fork_height=max(0.0,r.fork_height-0.8)
                return f"↓ {r.name} forks → {r.fork_height:.1f}m"
            if action in ("arm_pick","arm_drop"):
                self.arm.command(action)
                return f"🦾 Arm: {'picking' if action=='arm_pick' else 'placing'}"
            if action=="manual":
                r=self.robots[a.get("rid",5)]
                r.state="manual"; r.manual_vx=a.get("vx",0); r.manual_vz=a.get("vz",0)
                return None
            if action=="help":
                return ("Commands: robot N go to [A1-C8], robot N pick from [A1-C8],\n"
                        "forklift N pick from [C1-C8], arm pick/drop, camera top/follow/iso,\n"
                        "all stop, pause/resume")
            if action=="toggle_paths": return "__toggle_paths__"
            if action=="camera": return "__camera__"+json.dumps(a)
            if action=="unknown": return f"⚠ Unknown command. Type 'help'."
        return None

FLEET = Fleet()

# ══════════════════════════════════════════════════════════════════
#  GAZEBO BRIDGE  (optional – only active when --ros flag is used)
# ══════════════════════════════════════════════════════════════════
class GazeboBridge:
    """
    ROS 2 node that bridges the Python fleet simulation into Gazebo.

    Strategy: use Gazebo's /world/{world}/set_pose service to teleport
    each robot model every simulation tick (10 Hz).  Joint commands are
    published to the arm's joint-position-controller topics.

    The bridge runs in its own thread so the Flask server isn't blocked.
    """
    # Gazebo world name must match the name in warehouse.world
    WORLD = "warehouse"
    # Height (gz Z) of each robot type when on the floor
    Z_HEIGHT = {"tugbot": 0.26, "forklift": 0.45}
    # Gazebo model names: index maps to Fleet.robots list
    MODEL_NAMES = ["tugbot_1","tugbot_2","tugbot_3","forklift_1","forklift_2","tugbot_manual"]

    def __init__(self):
        self._node = None
        self._set_pose_cli = {}   # model_name → service client
        self._arm_pubs = {}       # joint_name → publisher
        self._odom_pubs = {}      # model_name → Odometry publisher
        self._tf_broadcaster = None

    def init_ros(self):
        """Must be called after rclpy.init()."""
        if not HAS_ROS: return
        self._node = rclpy.create_node('warehouse_gazebo_bridge')

        # ── Set-pose service clients ─────────────────────────────
        try:
            from ros_gz_interfaces.srv import SetEntityPose
            self._SetEntityPose = SetEntityPose
            for name in self.MODEL_NAMES:
                cli = self._node.create_client(
                    SetEntityPose,
                    f'/world/{self.WORLD}/set_pose')
                self._set_pose_cli[name] = cli
        except ImportError:
            self._node.get_logger().warn(
                "ros_gz_interfaces not found — model teleportation disabled. "
                "Install: sudo apt install ros-humble-ros-gz-interfaces")
            self._set_pose_cli = {}

        # ── Arm joint publishers ─────────────────────────────────
        for jnt in ["turret","shoulder","elbow","wrist","gripper"]:
            pub = self._node.create_publisher(Float64, f'/arm/{jnt}_cmd', 10)
            self._arm_pubs[jnt] = pub

        # ── Odometry publishers ──────────────────────────────────
        for name in self.MODEL_NAMES:
            pub = self._node.create_publisher(Odometry, f'/{name}/odom', 10)
            self._odom_pubs[name] = pub

        # ── TF broadcaster ──────────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self._node)

        # ── Fleet status publisher ───────────────────────────────
        self._status_pub = self._node.create_publisher(String, '/fleet/status', 10)
        self._arm_pub = self._node.create_publisher(String, '/fleet/arm', 10)

        # ── Command subscriber ───────────────────────────────────
        self._node.create_subscription(
            String, '/fleet/command', self._on_ros_cmd, 10)

        self._node.get_logger().info("✅ GazeboBridge initialised")

    def _on_ros_cmd(self, msg):
        """Receive commands from ROS topic and dispatch to Fleet."""
        try:
            parsed = parse_cmd(msg.data)
            FLEET.dispatch(parsed)
        except Exception as e:
            self._node.get_logger().error(f"Command error: {e}")

    def _euler_to_quat(self, yaw):
        """Convert yaw (rad) to quaternion (x,y,z,w)."""
        hy = yaw / 2.0
        return (0.0, 0.0, math.sin(hy), math.cos(hy))

    def set_model_pose(self, model_name, x, y, z, yaw):
        """
        Teleport a Gazebo model.  Uses /world/warehouse/set_pose service.
        Note: sim.x → gz.x, sim.z → gz.y (ROS REP-103).
        """
        if not self._set_pose_cli or model_name not in self._set_pose_cli:
            return
        cli = self._set_pose_cli[model_name]
        if not cli.service_is_ready(): return
        req = self._SetEntityPose.Request()
        req.entity.name = model_name
        req.entity.type = 2   # MODEL = 2 in gz_interfaces
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        qx,qy,qz,qw = self._euler_to_quat(yaw)
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw
        cli.call_async(req)   # non-blocking

    def publish_odom(self, model_name, x, y, z, yaw, now):
        """Publish odometry for a robot."""
        if model_name not in self._odom_pubs: return
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.child_frame_id = f"{model_name}/base_link"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        qx,qy,qz,qw = self._euler_to_quat(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self._odom_pubs[model_name].publish(msg)

    def publish_tf(self, model_name, x, y, z, yaw, now):
        """Broadcast TF transform for a robot."""
        if not self._tf_broadcaster: return
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = f"{model_name}/base_link"
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        qx,qy,qz,qw = self._euler_to_quat(yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

    def publish_arm_joints(self, arm: RoboticArm):
        """Send arm joint positions to Gazebo joint position controllers."""
        if not self._arm_pubs: return
        # Map Python arm angles → Gazebo controller topics
        joints = {
            "turret":   arm.base,
            "shoulder": arm.shoulder,
            "elbow":    arm.elbow,
            "wrist":    arm.wrist,
            "gripper":  arm.gripper * 0.18,  # 0…1 → 0…0.18 m
        }
        for jnt, val in joints.items():
            msg = Float64(); msg.data = float(val)
            self._arm_pubs[jnt].publish(msg)

    def run_loop(self):
        """Main bridge loop — runs in its own daemon thread at 10 Hz."""
        if not HAS_ROS or not self._node: return
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self._node)

        while rclpy.ok():
            robots, arm, conv = FLEET.snapshot()
            now = self._node.get_clock().now().to_msg()

            for i, r in enumerate(robots):
                if i >= len(self.MODEL_NAMES): break
                name = self.MODEL_NAMES[i]
                gz_z = self.Z_HEIGHT.get(r['vtype'], 0.26) + r.get('fork_height',0)*0.0
                # sim.x → gz.x,  sim.z → gz.y  (ROS convention)
                self.set_model_pose(name, r['x'], r['z'], gz_z, r['yaw'])
                self.publish_odom(name, r['x'], r['z'], gz_z, r['yaw'], now)
                self.publish_tf(name, r['x'], r['z'], gz_z, r['yaw'], now)

            # Arm joints
            arm_obj = FLEET.arm   # access actual RoboticArm object
            self.publish_arm_joints(arm_obj)

            # Fleet status JSON on ROS topic
            status_msg = String(); status_msg.data = json.dumps(robots)
            self._status_pub.publish(status_msg)
            arm_msg = String(); arm_msg.data = json.dumps(arm)
            self._arm_pub.publish(arm_msg)

            executor.spin_once(timeout_sec=0.08)  # ~10 Hz with overhead
            time.sleep(0.02)

BRIDGE = GazeboBridge()

# ══════════════════════════════════════════════════════════════════
#  FLASK / SOCKETIO  (identical to standalone_sim.py)
# ══════════════════════════════════════════════════════════════════
app = Flask(__name__)
app.config["SECRET_KEY"] = "wh2025v5"
sio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

def broadcast_loop():
    while True:
        robots, arm, conv = FLEET.snapshot()
        sio.emit("robots", robots)
        sio.emit("arm", arm)
        sio.emit("conveyor", conv)
        time.sleep(0.05)
threading.Thread(target=broadcast_loop, daemon=True).start()

@sio.on("cmd")
def on_cmd(data):
    a = data.get("action"); rid = int(data.get("rid", 5))
    if a=="manual":
        FLEET.dispatch({"action":"manual","rid":rid,
                        "vx":float(data.get("vx",0)),"vz":float(data.get("vz",0))})
    elif a=="estop":
        msg = FLEET.dispatch({"action":"estop"})
        emit("log",{"msg":msg,"type":"error"},broadcast=True)
    elif a=="pause":
        msg = FLEET.dispatch({"action":"pause"})
        emit("log",{"msg":msg},broadcast=True)
    elif a=="goto":
        msg = FLEET.dispatch({"action":"goto","rid":rid,
                               "x":float(data["x"]),"z":float(data["z"]),"loc":"click"})
        if msg: emit("log",{"msg":msg,"type":"cmd"},broadcast=True)

@sio.on("command")
def on_command(data):
    raw = data.get("text","")
    if not raw.strip(): return
    parsed = parse_cmd(raw)
    result = FLEET.dispatch(parsed)
    if result and result.startswith("__camera__"):
        emit("client_cmd", json.loads(result[10:]), broadcast=True)
    elif result == "__toggle_paths__":
        emit("client_cmd", {"action":"toggle_paths"}, broadcast=True)
    elif result:
        emit("log", {"msg":result,"type":"cmd"}, broadcast=True)
    elif parsed.get("action") not in ("manual",):
        emit("log", {"msg":f"CMD: {raw}","type":"cmd"}, broadcast=True)

# ── Import full Three.js HTML from standalone_sim (reuse it) ─────
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_STANDALONE = os.path.join(_THIS_DIR, "standalone_sim.py")

def _load_html():
    """Extract the HTML string from standalone_sim.py."""
    try:
        with open(_STANDALONE) as f:
            src = f.read()
        # Extract HTML=r"""..."""
        import re as _re
        m = _re.search(r'HTML\s*=\s*r"""(.*?)"""', src, _re.DOTALL)
        if m: return m.group(1)
    except Exception:
        pass
    return "<h1>Dashboard HTML not found — run standalone_sim.py for the full view</h1>"

HTML = _load_html()

@app.route("/")
def index(): return render_template_string(HTML)

# ══════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ros", action="store_true",
                    help="Enable ROS 2 / Gazebo bridge (requires rclpy + Gazebo Harmonic)")
    ap.add_argument("--port", type=int, default=5001,
                    help="Flask web server port (default 5001)")
    args = ap.parse_args()

    print("=" * 60)
    print("  Warehouse Automation Sim v5  —  Gazebo Edition")
    print("=" * 60)
    print(f"  Web dashboard: http://localhost:{args.port}")
    print(f"  ROS 2 bridge : {'ENABLED' if (args.ros and HAS_ROS) else 'DISABLED'}")
    if not HAS_ROS and args.ros:
        print("  ⚠  rclpy not found — install ROS 2 Humble for Gazebo bridge")
    print("=" * 60)

    if args.ros and HAS_ROS:
        # Init ROS and start Gazebo bridge thread
        rclpy.init()
        BRIDGE.init_ros()
        t = threading.Thread(target=BRIDGE.run_loop, daemon=True, name="gz_bridge")
        t.start()
        print("  🔗 Gazebo bridge thread started")

    sio.run(app, host="0.0.0.0", port=args.port, debug=False)
