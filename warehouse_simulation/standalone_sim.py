#!/usr/bin/env python3
"""
Warehouse Automation Sim v4 — Sensor & Conveyor Edition
========================================================
NEW IN v4:
  ✦ Ultrasonic / IR / Impact sensors per robot (real-time HUD)
  ✦ Collision avoidance — robots slow/stop, dynamic re-routing
  ✦ Full workflow: pick → carry (box follows robot) → drop zone → arm picks → conveyor
  ✦ Animated conveyor belt with items traveling along it
  ✦ Realistic 5-DOF robotic arm (shoulder + elbow + wrist + gripper)
  ✦ Forklifts pick CRATES only (C-row racks); TugBots pick BOXES (A,B rows)
  ✦ Smart charging — only when battery < 20%
  ✦ North & South walls removed — open warehouse
  ✦ 8 camera modes incl. Forklift-1 & Forklift-2 driver POV
  ✦ Sensor HUD overlay in 3D canvas
  ✦ Better robot models: detailed TugBot & Forklift

Run:  python3 standalone_sim.py
Open: http://localhost:5001
"""

import math, heapq, time, json, threading, random, re
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit

# ══════════════════════════════════════════════════════════════════
#  A* GRID  (60 × 36 m world, 0.5 m cells)
# ══════════════════════════════════════════════════════════════════
class Grid:
    CELL, W, H = 0.5, 60, 36
    SHELVES = [
        (-24,8,-20,12),(-18,8,-14,12),(-12,8,-8,12),(-6,8,-2,12),
        (2,8,6,12),(8,8,12,12),(14,8,18,12),(20,8,24,12),
        (-24,-1,-20,3),(-18,-1,-14,3),(-12,-1,-8,3),(-6,-1,-2,3),
        (2,-1,6,3),(8,-1,12,3),(14,-1,18,3),(20,-1,24,3),
        (-24,-12,-20,-8),(-18,-12,-14,-8),(-12,-12,-8,-8),(-6,-12,-2,-8),
        (2,-12,6,-8),(8,-12,12,-8),(14,-12,18,-8),(20,-12,24,-8),
        (25,-6,31,6),  # dispatch/conveyor block
    ]
    def __init__(self):
        self.cols=int(self.W/self.CELL); self.rows=int(self.H/self.CELL)
        self.g=[[0]*self.cols for _ in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                wx=-self.W/2+(c+.5)*self.CELL; wy=-self.H/2+(r+.5)*self.CELL
                for s in self.SHELVES:
                    if s[0]-.3<=wx<=s[2]+.3 and s[1]-.3<=wy<=s[3]+.3:
                        self.g[r][c]=1; break
    def wc(self,wx,wy):
        return (max(0,min(int((wx+self.W/2)/self.CELL),self.cols-1)),
                max(0,min(int((wy+self.H/2)/self.CELL),self.rows-1)))
    def cw(self,c,r): return (-self.W/2+(c+.5)*self.CELL,-self.H/2+(r+.5)*self.CELL)
    def plan(self,s,g):
        sc,sr=self.wc(*s); gc,gr=self.wc(*g)
        if self.g[sr][sc]: sc,sr=self.wc(s[0]+0.6,s[1])
        if self.g[gr][gc]: gc,gr=self.wc(g[0]-0.6,g[1])
        open_set=[(0,sc,sr,[])]; vis={}
        while open_set:
            cost,c,r,path=heapq.heappop(open_set)
            if (c,r) in vis: continue
            vis[(c,r)]=True; path=path+[self.cw(c,r)]
            if c==gc and r==gr: return path
            for dc,dr in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nc,nr=c+dc,r+dr
                if 0<=nc<self.cols and 0<=nr<self.rows and not self.g[nr][nc]:
                    h=math.hypot(nc-gc,nr-gr); step=1.414 if dc and dr else 1.0
                    heapq.heappush(open_set,(cost+step+h,nc,nr,path))
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
    "HOME4":(-26,2),"HOME5":(-26,-2),"HOME6":(-26,-6),
}

# ══════════════════════════════════════════════════════════════════
#  CONVEYOR + DROP ZONE
# ══════════════════════════════════════════════════════════════════
class Conveyor:
    def __init__(self):
        self.belt_offset=0.0
        self.items=[]   # [{z, name, age}]
    def add(self,name):
        self.items.append({"z":-5.5,"name":name,"age":0.0})
    def tick(self,dt):
        self.belt_offset=(self.belt_offset+dt*1.4)%2.0
        for it in self.items:
            it["z"]+=1.4*dt; it["age"]+=dt
        self.items=[i for i in self.items if i["z"]<7.0]
    def to_dict(self):
        return {"belt_offset":round(self.belt_offset,3),
                "items":[{"z":round(i["z"],2),"name":i["name"]} for i in self.items]}

class DropZone:
    def __init__(self): self.pending=[]
    def add(self,name): self.pending.append(name)
    def take(self):
        return self.pending.pop(0) if self.pending else None

DROP_ZONE = DropZone()
CONVEYOR  = Conveyor()

# ══════════════════════════════════════════════════════════════════
#  SENSORS
# ══════════════════════════════════════════════════════════════════
class Sensor:
    MAX_RANGE=8.0
    def __init__(self):
        self.ultrasonic={"front":8.0,"rear":8.0,"left":8.0,"right":8.0}
        self.ir={"floor":"clear","line":False}
        self.impact=False

# ══════════════════════════════════════════════════════════════════
#  ROBOT
# ══════════════════════════════════════════════════════════════════
class Robot:
    SPEED=3.0; DRAIN=0.010; CHARGE_SPD=7.0
    AVOID_STOP=1.6; AVOID_SLOW=3.8

    def __init__(self,rid,x,z,color,name,vtype="tugbot"):
        self.id=rid; self.x=float(x); self.z=float(z)
        self.yaw=0.0; self.battery=random.uniform(55,100)
        self.color=color; self.name=name; self.vtype=vtype
        self.state="idle"; self.path=[]; self.path_idx=0
        self.task=None; self.carrying=None
        self.fork_height=0.0
        self.manual_vx=0.0; self.manual_vz=0.0
        self.sensor=Sensor()
        self.log_entries=[]
        self._stop_timer=0.0

    def log(self,msg):
        self.log_entries.append({"t":time.strftime("%H:%M:%S"),"msg":msg,"name":self.name})
        if len(self.log_entries)>80: self.log_entries.pop(0)

    def go_to(self,wx,wz,reason=""):
        path=GRID.plan((self.x,self.z),(wx,wz))
        if path:
            self.path=path; self.path_idx=0; self.state="moving"
            self.log(f"→ {reason or f'({wx:.1f},{wz:.1f})'}")
            return True
        else:
            self.log(f"⚠ No path to ({wx:.1f},{wz:.1f})")
            return False

    def update_sensors(self,all_robots):
        """Update ultrasonic/IR/impact sensors from world state."""
        us={"front":8.0,"rear":8.0,"left":8.0,"right":8.0}
        for other in all_robots:
            if other.id==self.id: continue
            dx=other.x-self.x; dz=other.z-self.z
            dist=math.hypot(dx,dz)
            if dist>8.0: continue
            rel=math.atan2(dz,dx)-self.yaw
            while rel> math.pi: rel-=2*math.pi
            while rel<-math.pi: rel+=2*math.pi
            if   -math.pi/4<rel< math.pi/4: us["front"]=min(us["front"],dist)
            elif  rel> 3*math.pi/4 or rel<-3*math.pi/4: us["rear"]=min(us["rear"],dist)
            elif  math.pi/4<rel< 3*math.pi/4: us["left"]=min(us["left"],dist)
            else: us["right"]=min(us["right"],dist)
        # wall distances (east/west walls at x=±30)
        us["front"]=min(us["front"],abs(30-self.x))
        us["rear"] =min(us["rear"],abs(-30-self.x))
        self.sensor.ultrasonic={k:round(v,2) for k,v in us.items()}
        # IR – proximity to rack
        near=None
        for nm,pos in WP.items():
            if re.match(r'^[ABC]\d',nm) and math.hypot(self.x-pos[0],self.z-pos[1])<2.0:
                near=nm; break
        self.sensor.ir={"floor":f"near {near}" if near else "clear","line":near is not None}
        # Impact
        self.sensor.impact=any(v<1.4 for v in self.sensor.ultrasonic.values())

    # Physical radii: tugbot ~0.6m, forklift ~0.9m
    RADIUS = {"tugbot": 0.65, "forklift": 0.95}

    def _robot_radius(self): return self.RADIUS.get(self.vtype, 0.65)

    def _avoidance(self, all_robots):
        """Social-force avoidance + shelf wall repulsion.
        Returns (fx, fz, speed_factor).
        Lower robot-ID has right-of-way; higher ID yields harder."""
        fx, fz = 0.0, 0.0
        min_gap = 999.0          # gap = dist - combined_r  (space between surfaces)
        my_r = self._robot_radius()

        # ── Robot-robot repulsion ──────────────────────────────────
        for o in all_robots:
            if o.id == self.id: continue
            dx = self.x - o.x;  dz = self.z - o.z
            dist = math.hypot(dx, dz)
            if dist < 0.01: dist = 0.01
            combined_r = my_r + o._robot_radius()
            gap = dist - combined_r
            min_gap = min(min_gap, gap)
            if dist > combined_r * 3.8: continue
            # higher-ID robot yields harder
            priority = 1.6 if self.id > o.id else 0.35
            overlap = combined_r * 3.0 - dist
            if overlap > 0:
                strength = priority * overlap / (combined_r * 3.0)
                fx += (dx / dist) * strength
                fz += (dz / dist) * strength
            # Extra hard push when actually overlapping bodies
            if gap < 0:
                push = 4.0 * abs(gap) / combined_r
                fx += (dx / dist) * push
                fz += (dz / dist) * push

        # ── Shelf wall repulsion ───────────────────────────────────
        wall_margin = my_r + 0.45
        for sx0, sz0, sx1, sz1 in GRID.SHELVES:
            # Closest point on shelf AABB to robot centre
            cx = max(sx0, min(self.x, sx1))
            cz = max(sz0, min(self.z, sz1))
            dx = self.x - cx;  dz = self.z - cz
            wall_dist = math.hypot(dx, dz)
            if wall_dist < 0.01:
                # Robot centroid is inside the shelf box – push from centroid outward
                dx = self.x - (sx0 + sx1) * 0.5
                dz = self.z - (sz0 + sz1) * 0.5
                wall_dist = max(math.hypot(dx, dz), 0.01)
            if wall_dist > wall_margin + 1.2: continue
            overlap = wall_margin - wall_dist
            if overlap > 0:
                strength = 3.5 * overlap / wall_margin
                fx += (dx / wall_dist) * strength
                fz += (dz / wall_dist) * strength

        # ── Speed factor (based on surface-gap to nearest robot) ───
        stop_gap = 0.35     # hard stop when surfaces < 0.35 m apart
        slow_gap = 2.2      # start slowing at 2.2 m surface gap
        if min_gap <= stop_gap: speed = 0.0
        elif min_gap < slow_gap:
            speed = (min_gap - stop_gap) / (slow_gap - stop_gap)
        else: speed = 1.0
        return fx, fz, speed

    def _clamp_from_shelves(self):
        """Hard-push robot out of any shelf AABB it is overlapping."""
        r = self._robot_radius() + 0.05   # tiny extra safety margin
        for sx0, sz0, sx1, sz1 in GRID.SHELVES:
            ex0, ez0 = sx0 - r, sz0 - r
            ex1, ez1 = sx1 + r, sz1 + r
            if ex0 < self.x < ex1 and ez0 < self.z < ez1:
                # Find nearest expanded-box edge and push to it
                opts = [
                    (abs(self.x - ex0), 'x', ex0),
                    (abs(self.x - ex1), 'x', ex1),
                    (abs(self.z - ez0), 'z', ez0),
                    (abs(self.z - ez1), 'z', ez1),
                ]
                _, axis, val = min(opts)
                if axis == 'x': self.x = val
                else:           self.z = val

    def _replan_around(self, all_robots):
        """Block nearby robots as temp obstacles and replan."""
        if not self.path: return
        dest = self.path[-1]
        blocked = []
        for o in all_robots:
            if o.id == self.id: continue
            if math.hypot(o.x - self.x, o.z - self.z) > 10.0: continue
            # Mark a 3×3 cell block around each nearby robot
            for ddx in range(-2, 3):
                for ddz in range(-2, 3):
                    wx = o.x + ddx * GRID.CELL
                    wz = o.z + ddz * GRID.CELL
                    c, r = GRID.wc(wx, wz)
                    if 0 <= c < GRID.cols and 0 <= r < GRID.rows and GRID.g[r][c] == 0:
                        GRID.g[r][c] = 2; blocked.append((c, r))
        new = GRID.plan((self.x, self.z), dest)
        for c, r in blocked:
            if GRID.g[r][c] == 2: GRID.g[r][c] = 0
        if new and len(new) > 2:
            self.path = new; self.path_idx = 0
            self.log("↺ Re-routing around traffic")

    def tick(self,dt,all_robots=None):
        if self.state=="charging":
            self.battery=min(100,self.battery+self.CHARGE_SPD*dt)
            if self.battery>=100: self.state="idle"; self.log("✓ Fully charged")
            return
        if self.state=="manual":
            spd=3.6
            nx=max(-29,min(29,self.x+self.manual_vx*spd*dt))
            nz=max(-17,min(17,self.z+self.manual_vz*spd*dt))
            dist=math.hypot(nx-self.x,nz-self.z)
            self.battery=max(0,self.battery-self.DRAIN*dist)
            if self.manual_vx or self.manual_vz:
                self.yaw=math.atan2(self.manual_vz,self.manual_vx)
            self.x,self.z=nx,nz; return

        if self.state=="moving" and self.path:
            fx, fz, speed = self._avoidance(all_robots) if all_robots else (0,0,1)

            if speed < 0.05:
                # Hard stop – apply soft separation push so robots don't overlap
                push = 1.0
                self.x = max(-29, min(29, self.x + fx * push * dt))
                self.z = max(-17, min(17, self.z + fz * push * dt))
                self._clamp_from_shelves()
                self._stop_timer += dt
                if self._stop_timer > 2.5:
                    self._replan_around(all_robots or [])
                    self._stop_timer = 0.0
                return
            self._stop_timer = 0.0

            tx,tz=self.path[self.path_idx]
            dx,dz=tx-self.x,tz-self.z; dist=math.hypot(dx,dz)
            if dist<0.12:
                self.path_idx+=1
                if self.path_idx>=len(self.path):
                    self.path=[]; self.state="idle"; self._on_arrive()
            else:
                step = min(self.SPEED * dt * speed, dist)
                # Move toward waypoint + small lateral separation nudge
                self.x = max(-29, min(29, self.x + dx/dist*step + fx*dt*0.3))
                self.z = max(-17, min(17, self.z + dz/dist*step + fz*dt*0.3))
                self._clamp_from_shelves()
                self.yaw = math.atan2(dz, dx)
                self.battery=max(0,self.battery-self.DRAIN*step)
                if self.battery<5 and self.state!="charging":
                    self.log("⚡ CRITICAL battery — emergency charge!")
                    chg=random.choice(["CHG1","CHG2","CHG3"])
                    self.task={"type":"charge"}; self.go_to(*WP[chg],"EMERGENCY CHG")

        elif self.state=="picking":
            if not hasattr(self,'_ptimer'): self._ptimer=2.4
            self._ptimer-=dt
            if self.vtype=="forklift":
                self.fork_height=min(1.6,self.fork_height+dt*0.9)
            if self._ptimer<=0:
                del self._ptimer
                self.log(f"📦 Picked {self.carrying}")
                if self.vtype=="forklift": self.fork_height=0.8
                drop=self.task.get("drop",WP["STAGING"]) if self.task else WP["DISPATCH"]
                self.go_to(*drop,"→ drop zone")
                if self.task: self.task["type"]="drop"

        elif self.state=="dropping":
            if not hasattr(self,'_dtimer'): self._dtimer=1.6
            self._dtimer-=dt
            if self.vtype=="forklift":
                self.fork_height=max(0.05,self.fork_height-dt*0.9)
            if self._dtimer<=0:
                del self._dtimer
                itm=self.carrying
                self.log(f"✓ Dropped {itm}")
                # Add to drop zone so arm picks it up
                if self.task and self.task.get("drop") in (WP["DISPATCH"],WP["DROP"]):
                    DROP_ZONE.add(itm)
                self.carrying=None; self.task=None; self.state="idle"
                if self.vtype=="forklift": self.fork_height=0.05

    def _on_arrive(self):
        if self.task:
            t=self.task.get("type")
            if t=="pick":
                self.state="picking"; self.carrying=self.task.get("item","Box")
                self.log(f"⏳ Picking {self.carrying}…")
            elif t=="drop":
                self.state="dropping"
                self.log(f"⏳ Dropping {self.carrying}…")
            elif t=="charge":
                self.state="charging"; self.task=None; self.log("⚡ Charging…")
        else:
            self.state="idle"

    def to_dict(self):
        return {"id":self.id,"name":self.name,"x":round(self.x,2),"z":round(self.z,2),
                "yaw":round(self.yaw,3),"battery":round(self.battery,1),"state":self.state,
                "color":self.color,"carrying":self.carrying,"vtype":self.vtype,
                "fork_height":round(self.fork_height,3),
                "path":self.path[self.path_idx:self.path_idx+60],
                "sensor":{"ultrasonic":self.sensor.ultrasonic,
                          "ir":self.sensor.ir,"impact":self.sensor.impact}}

# ══════════════════════════════════════════════════════════════════
#  ROBOTIC ARM  — 10-state industrial arm
#  Pose angles: base=turret rotation, shoulder/elbow=joint bend (rad)
#  Drop zone at x≈22,z=0  |  Conveyor at x≈28.5,z=0
#  Arm sits at x=25.5, z=0 — equidistant so it can reach both sides
# ══════════════════════════════════════════════════════════════════
class RoboticArm:
    # Named poses {base, shoulder, elbow, wrist}
    # base: -π/2 ≈ -1.57 faces DROP side; +π/2 ≈ +1.57 faces CONVEYOR side
    P = {
        "rest":       ( 0.00, 0.55, 0.40, 0.00),
        "over_drop":  (-1.57, 0.85, 0.70, 0.10),  # above drop zone, arm extended
        "at_drop":    (-1.57, 0.18, 0.06, 0.00),  # low — grasping height
        "lifted":     (-1.57, 1.05, 0.95, 0.20),  # lifted high with item
        "mid_swing":  ( 0.00, 1.10, 0.90, 0.15),  # halfway through swing (clear height)
        "over_belt":  ( 1.45, 0.80, 0.55, 0.00),  # above conveyor belt
        "at_belt":    ( 1.45, 0.28, 0.12, 0.00),  # lowered onto belt
    }
    STATES = [
        "idle",          # 0 gentle sway
        "open_grip",     # 1 open gripper before swing
        "swing_drop",    # 2 rotate to drop zone side
        "descend",       # 3 lower arm to item
        "close_grip",    # 4 close gripper (pick up item)
        "ascend",        # 5 lift item clear
        "swing_belt",    # 6 rotate 180° to conveyor side (via mid_swing)
        "lower_belt",    # 7 lower onto belt
        "release",       # 8 open gripper, item → belt
        "return",        # 9 return to rest pose
    ]

    def __init__(self):
        self.x=25.5; self.z=0.0
        self.base=0.0; self.shoulder=0.55; self.elbow=0.40; self.wrist=0.0
        self.gripper=0.0          # 0=fully open, 1=fully closed
        self.state="idle"
        self._timer=0.0
        self.has_item=None
        # gripper world position computed in tick for JS carry mesh
        self.grip_x=self.x; self.grip_y=2.5; self.grip_z=self.z

    # ── helpers ─────────────────────────────────────────────────
    def _a(self, cur, tgt, spd, dt):
        """Smoothly move cur toward tgt at speed spd rad/s."""
        d = tgt - cur
        if abs(d) < 0.003: return tgt
        return cur + math.copysign(min(abs(d), spd * dt), d)

    def _pose(self, name, dt, spd=1.6):
        """Drive all joints toward named pose. Returns True when done."""
        b,s,e,w = self.P[name]
        self.base     = self._a(self.base,     b, spd,       dt)
        self.shoulder = self._a(self.shoulder, s, spd,       dt)
        self.elbow    = self._a(self.elbow,    e, spd*0.9,   dt)
        self.wrist    = self._a(self.wrist,    w, spd*0.7,   dt)
        return (abs(self.base-b)<0.04 and abs(self.shoulder-s)<0.04
                and abs(self.elbow-e)<0.04)

    def _compute_grip_pos(self):
        """Approximate world position of gripper tip for carry-mesh display."""
        # Forward kinematics (2-link planar in the arm's vertical plane)
        L1, L2 = 1.45, 1.05          # upper-arm and lower-arm lengths
        # Shoulder in world: arm base lifted by 0.55m on turret
        sx = self.x + math.cos(self.base) * 0.0   # shoulder pivot is on turret axis
        sz = self.z + math.sin(self.base) * 0.0
        sy = 0.75                                   # shoulder pivot height
        # Shoulder angle (shoulder bends arm outward in world xz-plane)
        # shoulder = elevation above horizontal
        elev = math.pi/2 - self.shoulder           # 0=pointing up, π/2=horizontal
        reach = L1 * math.sin(self.shoulder) + L2 * math.sin(self.shoulder + self.elbow)
        height = L1 * math.cos(self.shoulder) + L2 * math.cos(self.shoulder + self.elbow)
        gx = self.x + math.cos(self.base) * reach
        gy = sy + height
        gz = self.z + math.sin(self.base) * reach
        self.grip_x = round(gx, 2)
        self.grip_y = round(max(0.3, gy), 2)
        self.grip_z = round(gz, 2)

    def auto_pick(self, ix, iz, name):
        if self.state != "idle": return False
        self.has_item = name
        self.state = "open_grip"; self._timer = 0.0
        return True

    def command(self, action):
        if action == "arm_pick" and self.state == "idle":
            self.has_item = "Manual-Item"
            self.state = "open_grip"; self._timer = 0.0
        elif action == "arm_drop" and self.state not in ("idle",):
            # Force immediate release onto belt
            if self.has_item:
                CONVEYOR.add(self.has_item); self.has_item = None
            self.state = "return"; self._timer = 0.0

    def tick(self, dt):
        self._timer += dt
        t = time.time()

        if self.state == "idle":
            self.base     = 0.22 * math.sin(t * 0.24)
            self.shoulder = 0.55 + 0.10 * math.sin(t * 0.38)
            self.elbow    = 0.40 + 0.08 * math.sin(t * 0.51)
            self.wrist    = 0.12 * math.sin(t * 0.67)
            self.gripper  = max(0.0, self.gripper - dt * 1.2)
            self._compute_grip_pos()
            return

        if self.state == "open_grip":
            # Fully open gripper before moving
            self.gripper = max(0.0, self.gripper - dt * 3.5)
            if self.gripper < 0.05 and self._timer > 0.35:
                self.state = "swing_drop"; self._timer = 0.0

        elif self.state == "swing_drop":
            # Swing base toward drop zone while raising arm to clear height
            done = self._pose("over_drop", dt, 1.8)
            if done and self._timer > 0.6:
                self.state = "descend"; self._timer = 0.0

        elif self.state == "descend":
            # Lower slowly to item height
            done = self._pose("at_drop", dt, 1.0)
            if done and self._timer > 0.4:
                self.state = "close_grip"; self._timer = 0.0

        elif self.state == "close_grip":
            # Close gripper firmly around item
            self.gripper = min(1.0, self.gripper + dt * 2.8)
            if self.gripper >= 0.95 and self._timer > 0.5:
                self.state = "ascend"; self._timer = 0.0

        elif self.state == "ascend":
            # Lift item high before swinging
            done = self._pose("lifted", dt, 1.4)
            if done and self._timer > 0.5:
                self.state = "swing_belt"; self._timer = 0.0

        elif self.state == "swing_belt":
            # Two-phase swing: go through mid_swing first to avoid hitting racks
            # Phase A (timer < 1.0): swing through mid_swing
            # Phase B (timer ≥ 1.0): continue to over_belt
            if self._timer < 1.0:
                done_mid = self._pose("mid_swing", dt, 2.0)
            else:
                done = self._pose("over_belt", dt, 1.8)
                if done and self._timer > 1.8:
                    self.state = "lower_belt"; self._timer = 0.0

        elif self.state == "lower_belt":
            done = self._pose("at_belt", dt, 1.0)
            if done and self._timer > 0.4:
                self.state = "release"; self._timer = 0.0

        elif self.state == "release":
            # Open gripper — item drops onto belt
            self.gripper = max(0.0, self.gripper - dt * 3.5)
            if self._timer > 0.3 and self.has_item:
                CONVEYOR.add(self.has_item)
                self.has_item = None
            if self.gripper < 0.05 and self._timer > 0.8:
                self.state = "return"; self._timer = 0.0

        elif self.state == "return":
            done = self._pose("over_belt", dt, 1.6)
            if self._timer > 0.5:
                done2 = self._pose("rest", dt, 1.6)
                if done2 and self._timer > 1.4:
                    self.state = "idle"; self._timer = 0.0

        self._compute_grip_pos()

    def to_dict(self):
        return {"x":self.x,"z":self.z,
                "base":round(self.base,3),"shoulder":round(self.shoulder,3),
                "elbow":round(self.elbow,3),"wrist":round(self.wrist,3),
                "gripper":round(self.gripper,3),"state":self.state,
                "has_item":self.has_item,
                "grip_x":self.grip_x,"grip_y":self.grip_y,"grip_z":self.grip_z}

# ══════════════════════════════════════════════════════════════════
#  TASK CATALOG  (boxes for tugbots, crates for forklifts)
# ══════════════════════════════════════════════════════════════════
BOX_TASKS=[
    {"type":"pick","item":"Box-A01","pick":WP["A1"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-A05","pick":WP["A5"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-B03","pick":WP["B3"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-B07","pick":WP["B7"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-A03","pick":WP["A3"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-B05","pick":WP["B5"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-A07","pick":WP["A7"],"drop":WP["DROP"],"for":"tugbot"},
    {"type":"pick","item":"Box-B01","pick":WP["B1"],"drop":WP["DROP"],"for":"tugbot"},
]
CRATE_TASKS=[
    {"type":"pick","item":"Crate-C02","pick":WP["C2"],"drop":WP["STAGING"],"for":"forklift"},
    {"type":"pick","item":"Crate-C06","pick":WP["C6"],"drop":WP["STAGING"],"for":"forklift"},
    {"type":"pick","item":"Crate-C04","pick":WP["C4"],"drop":WP["STAGING"],"for":"forklift"},
    {"type":"pick","item":"Crate-C08","pick":WP["C8"],"drop":WP["STAGING"],"for":"forklift"},
]
ALL_TASKS=BOX_TASKS+CRATE_TASKS

# ══════════════════════════════════════════════════════════════════
#  COMMAND PARSER
# ══════════════════════════════════════════════════════════════════
def parse_cmd(raw):
    s=raw.lower().strip()
    if re.search(r'\b(estop|e-stop|emergency|all stop|stop all)\b',s):
        return {'action':'estop'}
    if re.search(r'\b(pause|resume|unpause)\b',s):
        return {'action':'pause'}
    # camera
    if re.search(r'\b(cam|camera|view)\b.*\b(top|overhead|bird|down)\b',s):
        return {'action':'camera','mode':'top'}
    m=re.search(r'\b(cam|camera|view)\b.*\b(follow|track)\b.*?(\d)',s)
    if m: return {'action':'camera','mode':'follow','rid':int(m.group(3))-1}
    if re.search(r'\b(iso|isometric)\b',s): return {'action':'camera','mode':'iso'}
    if re.search(r'\b(dock|side|front)\b.*\b(view|cam)\b',s): return {'action':'camera','mode':'dock'}
    if re.search(r'\b(forklift|fl).*(pov|driver|cab|cockpit)',s):
        m2=re.search(r'(\d)',s); n=int(m2.group(1)) if m2 else 1
        return {'action':'camera','mode':f'fl{n}pov'}
    if re.search(r'\b(arm|conveyor).*(cam|view)',s): return {'action':'camera','mode':'arm'}
    if re.search(r'^cam\s*1',s): return {'action':'camera','mode':'free'}
    if re.search(r'^cam\s*2',s): return {'action':'camera','mode':'top'}
    if re.search(r'^cam\s*3',s): return {'action':'camera','mode':'follow','rid':0}
    if re.search(r'^cam\s*4',s): return {'action':'camera','mode':'iso'}
    if re.search(r'^cam\s*5',s): return {'action':'camera','mode':'dock'}
    if re.search(r'^cam\s*6',s): return {'action':'camera','mode':'fl1pov'}
    if re.search(r'^cam\s*7',s): return {'action':'camera','mode':'fl2pov'}
    if re.search(r'^cam\s*8',s): return {'action':'camera','mode':'arm'}
    # paths toggle
    if re.search(r'\b(show|hide|toggle)\b.*\bpath',s): return {'action':'toggle_paths'}
    # identify vehicle
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
    # fork up/down
    if vtype in ('forklift','any') and rid is not None:
        if re.search(r'\b(lift|raise|forks up)\b',s): return {'action':'fork_up','rid':rid}
        if re.search(r'\b(lower|forks down|drop forks)\b',s): return {'action':'fork_down','rid':rid}
    # goto coordinates
    mc=re.search(r'\(?\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)?',s)
    if mc and rid is not None and re.search(r'\b(go|goto|navigate|move|send|drive)\b',s):
        return {'action':'goto','rid':rid,'x':float(mc.group(1)),'z':float(mc.group(2)),'loc':'coords'}
    # goto named location
    m=re.search(r'\b(go|goto|navigate|move|send|drive|take)\b.*?\b([A-C][1-8]|dispatch|staging|inbound|drop|chg\d?|home\d?)\b',s,re.I)
    if m and rid is not None:
        loc=m.group(2).upper()
        if loc in WP: return {'action':'goto','rid':rid,'x':WP[loc][0],'z':WP[loc][1],'loc':loc}
    # bare location shorthand
    m=re.search(r'\b([A-C][1-8]|dispatch|staging|inbound)\b',s,re.I)
    if m and rid is not None and not re.search(r'\b(pick|drop|patrol)\b',s):
        loc=m.group(1).upper()
        if loc in WP: return {'action':'goto','rid':rid,'x':WP[loc][0],'z':WP[loc][1],'loc':loc}
    # pick
    m=re.search(r'\b(pick|pickup|collect|get|fetch|grab)\b.*?\b([A-C][1-8])\b',s,re.I)
    if m and rid is not None:
        loc=m.group(2).upper()
        if loc in WP:
            item=f"Crate-{loc}" if loc.startswith('C') else f"Box-{loc}"
            return {'action':'pick','rid':rid,'loc':loc,'item':item}
    # drop/deliver
    m=re.search(r'\b(drop|deliver|place|put|bring)\b.*?\b(dispatch|staging|inbound|drop)\b',s,re.I)
    if m and rid is not None:
        loc=m.group(2).upper()
        return {'action':'drop','rid':rid,'loc':loc}
    # charge
    if re.search(r'\b(charge|charging|battery|recharge)\b',s) and rid is not None:
        return {'action':'charge','rid':rid}
    # arm commands
    if vtype=='arm':
        if re.search(r'\b(pick|grab|grasp)\b',s): return {'action':'arm_pick','rid':rid}
        if re.search(r'\b(drop|place|release)\b',s): return {'action':'arm_drop','rid':rid}
    # patrol
    m=re.search(r'\bpatrol\b.*?\b([A-C][1-8])\b.*?\b([A-C][1-8])\b',s,re.I)
    if m and rid is not None:
        return {'action':'patrol','rid':rid,'from':m.group(1).upper(),'to':m.group(2).upper()}
    if re.search(r'\b(help|commands|\?)\b',s): return {'action':'help'}
    return {'action':'unknown','raw':raw}

HELP_TEXT="""
Commands:
  robot N go to [A1–C8 / DISPATCH / STAGING]
  robot N pick from [A1–C8]    (rows A,B=boxes; C=crates)
  robot N drop at [DISPATCH / STAGING]
  robot N charge
  forklift N go to [location]
  forklift N forks up / forks down
  forklift N pick from [C1–C8]   (crates only!)
  arm pick / arm drop
  camera top / follow N / iso / dock / fl1pov / fl2pov / arm
  cam 1–8   (shortcut)
  all stop  /  pause  /  resume
  help
"""

# ══════════════════════════════════════════════════════════════════
#  FLEET
# ══════════════════════════════════════════════════════════════════
class Fleet:
    def __init__(self):
        self.robots=[
            Robot(0,-26,14,"#00c8ff","TugBot-1","tugbot"),
            Robot(1,-26,10,"#7fff00","TugBot-2","tugbot"),
            Robot(2,-26, 6,"#ff6b35","TugBot-3","tugbot"),
            Robot(3,-26, 2,"#ffd700","Forklift-1","forklift"),
            Robot(4,-26,-2,"#ff9500","Forklift-2","forklift"),
            Robot(5,-26,-6,"#ff3af0","TugBot-M","tugbot"),
        ]
        self.arm=RoboticArm()
        self.task_queue=list(ALL_TASKS)
        self.paused=False
        self._lock=threading.Lock()
        self._t=time.time()
        threading.Thread(target=self._loop,daemon=True).start()
        # Stagger initial assignments
        for i,r in enumerate(self.robots[:5]):
            threading.Timer(i*1.5, lambda rb=r: self._assign(rb)).start()

    def _assign(self,robot):
        if robot.state!="idle" or robot.battery<20:
            if robot.battery<20 and robot.state=="idle":
                chg=random.choice(["CHG1","CHG2","CHG3"])
                robot.task={"type":"charge"}
                robot.go_to(*WP[chg],"charge")
                robot.log(f"⚡ Battery low ({robot.battery:.0f}%) → charging")
            return
        eligible=[t for t in self.task_queue if t.get("for","any") in (robot.vtype,"any")]
        if eligible:
            task=eligible[0]; self.task_queue.remove(task)
            robot.task={**task,"type":"pick","drop":task.get("drop",WP["DROP"])}
            robot.go_to(*task["pick"],f"pick {task['item']}")

    def _loop(self):
        while True:
            now=time.time(); dt=min(now-self._t,0.1); self._t=now
            if not self.paused:
                with self._lock:
                    for r in self.robots: r.update_sensors(self.robots)
                    for r in self.robots: r.tick(dt,self.robots)
                    self.arm.tick(dt)
                    CONVEYOR.tick(dt)
                    # Auto-trigger arm from drop zone
                    if self.arm.state=="idle" and DROP_ZONE.pending:
                        item=DROP_ZONE.take()
                        self.arm.auto_pick(22.0, 0.0, item)
                    # Auto-assign idle robots
                    for r in self.robots[:5]:
                        if r.state=="idle":
                            if not self.task_queue: self.task_queue=list(ALL_TASKS)
                            self._assign(r)
            time.sleep(0.04)

    def snapshot(self):
        with self._lock:
            return ([r.to_dict() for r in self.robots],
                    self.arm.to_dict(), CONVEYOR.to_dict())

    def dispatch(self,a):
        action=a.get("action")
        with self._lock:
            if action=="estop":
                for r in self.robots:
                    r.path=[]; r.state="idle"; r.manual_vx=0; r.manual_vz=0; r.log("⛔ E-STOP")
                return "⛔ All robots stopped"
            if action=="pause":
                self.paused=not self.paused
                return f"{'⏸ Paused' if self.paused else '▶ Resumed'}"
            rid=a.get("rid")
            if action=="goto" and rid is not None and 0<=rid<len(self.robots):
                r=self.robots[rid]; r.task=None
                r.go_to(a["x"],a["z"],a.get("loc",""))
                return f"✓ {r.name} → {a.get('loc','')}"
            if action=="pick" and rid is not None and 0<=rid<len(self.robots):
                r=self.robots[rid]; loc=a["loc"]; item=a.get("item",f"Box-{loc}")
                # Vehicle type restriction
                if r.vtype=="forklift" and not item.startswith("Crate"):
                    return f"⚠ {r.name} handles CRATES only (C-row). Use a TugBot for boxes."
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
            if action=="help": return HELP_TEXT
            if action=="toggle_paths": return "__toggle_paths__"
            if action=="camera": return "__camera__"+json.dumps(a)
            if action=="unknown": return f"⚠ Unknown command. Type 'help'."
        return None

FLEET=Fleet()

# ══════════════════════════════════════════════════════════════════
#  FLASK / SOCKETIO
# ══════════════════════════════════════════════════════════════════
app=Flask(__name__)
app.config["SECRET_KEY"]="wh2025v4"
sio=SocketIO(app,cors_allowed_origins="*",async_mode="threading")

def broadcast_loop():
    while True:
        robots,arm,conv=FLEET.snapshot()
        sio.emit("robots",robots)
        sio.emit("arm",arm)
        sio.emit("conveyor",conv)
        time.sleep(0.05)
threading.Thread(target=broadcast_loop,daemon=True).start()

@sio.on("cmd")
def on_cmd(data):
    a=data.get("action"); rid=int(data.get("rid",5))
    if a=="manual":
        FLEET.dispatch({"action":"manual","rid":rid,"vx":float(data.get("vx",0)),"vz":float(data.get("vz",0))})
    elif a=="estop":
        msg=FLEET.dispatch({"action":"estop"})
        emit("log",{"msg":msg,"type":"error"},broadcast=True)
    elif a=="pause":
        msg=FLEET.dispatch({"action":"pause"})
        emit("log",{"msg":msg},broadcast=True)
    elif a=="goto":
        msg=FLEET.dispatch({"action":"goto","rid":rid,"x":float(data["x"]),"z":float(data["z"]),"loc":"click"})
        if msg: emit("log",{"msg":msg,"type":"cmd"},broadcast=True)

@sio.on("command")
def on_command(data):
    raw=data.get("text","")
    if not raw.strip(): return
    parsed=parse_cmd(raw)
    result=FLEET.dispatch(parsed)
    if result and result.startswith("__camera__"):
        emit("client_cmd",json.loads(result[10:]),broadcast=True)
    elif result=="__toggle_paths__":
        emit("client_cmd",{"action":"toggle_paths"},broadcast=True)
    elif result:
        emit("log",{"msg":result,"type":"cmd"},broadcast=True)
    elif parsed.get("action") not in ("manual",):
        emit("log",{"msg":f"CMD: {raw}","type":"cmd"},broadcast=True)

@app.route("/")
def index(): return render_template_string(HTML)

# ══════════════════════════════════════════════════════════════════
#  HTML / THREE.JS FRONTEND
# ══════════════════════════════════════════════════════════════════
HTML=r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Warehouse Sim v4 — Sensor Edition</title>
<style>
*{margin:0;padding:0;box-sizing:border-box;}
body{background:#0d1117;color:#e0e0e0;font-family:'Segoe UI',monospace;display:flex;flex-direction:column;height:100vh;overflow:hidden;}
#topbar{height:42px;background:linear-gradient(90deg,#0a1f3d,#0f2d4a,#0a1f3d);display:flex;align-items:center;gap:6px;padding:0 12px;border-bottom:2px solid #00c8ff;flex-shrink:0;}
#topbar h1{font-size:12px;color:#00c8ff;letter-spacing:1px;white-space:nowrap;margin-right:4px;}
.tb-btn{background:#0c2040;border:1px solid #1e4a7a;color:#aac;padding:3px 9px;border-radius:3px;cursor:pointer;font-size:11px;white-space:nowrap;transition:all .15s;}
.tb-btn:hover{background:#1a3a6a;color:#00c8ff;border-color:#00c8ff88;}
.tb-btn.red{border-color:#ff4444;color:#ff6666;}.tb-btn.red:hover{background:#4a0f0f;color:#ff4444;}
.tb-btn.active{background:#00c8ff22;border-color:#00c8ff;color:#00c8ff;}
.cam-btn{background:#0a1628;border:1px solid #1a3050;color:#889;padding:3px 8px;border-radius:3px;cursor:pointer;font-size:10px;transition:all .15s;}
.cam-btn:hover{border-color:#00c8ff66;color:#aac;}.cam-btn.active{background:#00c8ff22;border-color:#00c8ff;color:#00c8ff;}
.sep{width:1px;height:24px;background:#1a3050;margin:0 2px;}
#timer{margin-left:auto;font-size:10px;color:#446;}
#main{display:flex;flex:1;overflow:hidden;}
#canvas-wrap{flex:1;position:relative;overflow:hidden;}
canvas{display:block;}
#scene-label{position:absolute;top:8px;left:8px;background:#00000080;border:1px solid #00c8ff44;color:#00c8ff;padding:2px 10px;font-size:11px;border-radius:2px;pointer-events:none;}
#cam-label{position:absolute;top:8px;left:50%;transform:translateX(-50%);background:#00000099;border:1px solid #00c8ff33;color:#aaa;padding:3px 16px;font-size:11px;border-radius:10px;pointer-events:none;}
#coords{position:absolute;bottom:8px;left:8px;color:#445;font-size:10px;pointer-events:none;}
/* SENSOR HUD */
#sensor-hud{position:absolute;bottom:36px;left:8px;background:#00000099;border:1px solid #00c8ff33;color:#bbb;font-family:monospace;font-size:10px;padding:7px 10px;border-radius:4px;width:194px;pointer-events:none;line-height:1.7;}
#sensor-hud .sh-title{color:#00c8ff;font-size:10px;letter-spacing:1px;margin-bottom:3px;}
.sh-row{display:flex;align-items:center;gap:5px;margin:1px 0;}
.sh-lbl{width:40px;color:#668;font-size:9px;}
.sh-val{width:32px;color:#eee;font-size:10px;}
.sh-bar-wrap{flex:1;height:5px;background:#112233;border-radius:2px;}
.sh-bar-fill{height:5px;border-radius:2px;transition:width .3s;}
.sh-ir{margin-top:3px;color:#aaa;font-size:9px;}
.sh-impact-ok{color:#3f3;}.sh-impact-warn{color:#f44;animation:blink .5s infinite;}
@keyframes blink{50%{opacity:0.3;}}
/* RIGHT PANEL */
#panel{width:285px;background:#0e1826;border-left:1px solid #1a3050;display:flex;flex-direction:column;overflow:hidden;flex-shrink:0;}
.psec{border-bottom:1px solid #0e2235;}
.phdr{background:#0c1d30;padding:5px 10px;font-size:10px;color:#00c8ff;letter-spacing:1px;cursor:pointer;display:flex;justify-content:space-between;user-select:none;}
.phdr:hover{background:#112840;}
.pbody{padding:6px;overflow-y:auto;}
#entity-tree{max-height:150px;overflow-y:auto;}
.e-row{display:flex;align-items:center;gap:5px;padding:2px 4px;cursor:pointer;border-radius:2px;font-size:11px;}
.e-row:hover,.e-row.sel{background:#0c2040;color:#00c8ff;}
.e-dot{width:9px;height:9px;border-radius:50%;flex-shrink:0;}
.e-type{font-size:9px;color:#446;margin-left:auto;}
#teleop{padding:8px;}
.tgrid{display:grid;grid-template-columns:repeat(3,40px);grid-template-rows:repeat(3,40px);gap:3px;justify-content:center;}
.tbtn{background:#0d2040;border:1px solid #1a3a5c;color:#aaa;border-radius:4px;cursor:pointer;font-size:17px;display:flex;align-items:center;justify-content:center;user-select:none;transition:all .1s;}
.tbtn:active,.tbtn.active{background:#00c8ff22;border-color:#00c8ff;color:#00c8ff;}
.tbtn.empty{background:transparent;border:none;cursor:default;}
#rsel{width:100%;background:#0c1d2e;color:#ccc;border:1px solid #1a3050;padding:4px;border-radius:3px;font-size:11px;margin-bottom:5px;}
.frow{display:flex;gap:5px;margin-top:4px;}
.fbtn{flex:1;background:#0c1d2e;border:1px solid #1a3050;color:#aaa;padding:3px;border-radius:3px;cursor:pointer;font-size:10px;text-align:center;}
.fbtn:hover{border-color:#ffd700;color:#ffd700;}
#fleet-cards{max-height:200px;overflow-y:auto;}
.rcard{background:#0a1628;border:1px solid #1a3050;border-radius:3px;padding:6px;margin-bottom:4px;cursor:pointer;transition:border-color .15s;}
.rcard:hover,.rcard.sel{border-color:#00c8ff;}
.rcard-top{display:flex;justify-content:space-between;align-items:center;}
.rname{font-size:11px;font-weight:bold;}
.rstate{font-size:9px;padding:1px 6px;border-radius:8px;}
.st-idle{background:#091e09;color:#7fff00;}.st-moving{background:#091422;color:#00c8ff;}
.st-picking{background:#1e1400;color:#ffd700;}.st-charging{background:#091030;color:#ff6b35;}
.st-manual{background:#1e0920;color:#ff3af0;}.st-dropping{background:#1e1900;color:#ffaa00;}
.rinfo{display:flex;gap:6px;margin-top:3px;font-size:9px;color:#446;}
.bat-bar{flex:1;height:4px;border-radius:2px;background:#1a3050;margin-top:4px;position:relative;overflow:hidden;}
.bat-fill{height:100%;border-radius:2px;transition:width .3s;}
.carrying-tag{font-size:8px;color:#ffd700;margin-top:2px;}
.impact-dot{width:6px;height:6px;border-radius:50%;background:#3f3;display:inline-block;margin-left:4px;}
.impact-dot.warn{background:#f44;animation:blink .4s infinite;}
#log-scroll{max-height:120px;overflow-y:auto;}
.log-entry{font-size:10px;padding:1px 2px;border-bottom:1px solid #0d1e30;line-height:1.5;}
.log-cmd{color:#00c8ff;}.log-error{color:#ff6644;}.log-warn{color:#ffd700;}
#cmd-bar{display:flex;gap:4px;padding:6px;}
#cmd-in{flex:1;background:#0c1d2e;border:1px solid #1a3050;color:#ccc;padding:4px 8px;border-radius:3px;font-size:11px;outline:none;}
#cmd-in:focus{border-color:#00c8ff66;}
#cmd-send{background:#0c2040;border:1px solid #00c8ff44;color:#00c8ff;padding:4px 10px;border-radius:3px;cursor:pointer;font-size:11px;}
#cmd-send:hover{background:#00c8ff22;}
</style>
</head>
<body>
<div id="topbar">
  <h1>🏭 WAREHOUSE SIM v4</h1>
  <div class="sep"></div>
  <button class="tb-btn red" onclick="estop()">⛔ E-STOP</button>
  <button class="tb-btn" id="btn-pause" onclick="togglePause()">⏸ Pause</button>
  <button class="tb-btn" id="btn-paths" onclick="togglePaths()">🗺 Paths</button>
  <div class="sep"></div>
  <span style="font-size:10px;color:#446;">CAM:</span>
  <button class="cam-btn active" id="cb0" onclick="setCamera(0)">Free</button>
  <button class="cam-btn" id="cb1" onclick="setCamera(1)">Top</button>
  <button class="cam-btn" id="cb2" onclick="setCamera(2)">Follow</button>
  <button class="cam-btn" id="cb3" onclick="setCamera(3)">Iso</button>
  <button class="cam-btn" id="cb4" onclick="setCamera(4)">Dock</button>
  <button class="cam-btn" id="cb5" onclick="setCamera(5)">FL1 POV</button>
  <button class="cam-btn" id="cb6" onclick="setCamera(6)">FL2 POV</button>
  <button class="cam-btn" id="cb7" onclick="setCamera(7)">Arm</button>
  <div class="sep"></div>
  <span id="timer">00:00</span>
</div>
<div id="main">
  <div id="canvas-wrap">
    <canvas id="c"></canvas>
    <div id="scene-label">WAREHOUSE — LIVE</div>
    <div id="cam-label">🎥 Free Orbit</div>
    <div id="coords"></div>
    <div id="sensor-hud">
      <div class="sh-title">📡 SENSORS: <span id="sh-name">—</span></div>
      <div class="sh-row"><span class="sh-lbl">▲ FRONT</span><span class="sh-val" id="sv-f">—</span><div class="sh-bar-wrap"><div class="sh-bar-fill" id="sb-f" style="background:#00c8ff;width:0%"></div></div></div>
      <div class="sh-row"><span class="sh-lbl">▼ REAR</span><span class="sh-val" id="sv-r">—</span><div class="sh-bar-wrap"><div class="sh-bar-fill" id="sb-r" style="background:#00c8ff;width:0%"></div></div></div>
      <div class="sh-row"><span class="sh-lbl">◀ LEFT</span><span class="sh-val" id="sv-l">—</span><div class="sh-bar-wrap"><div class="sh-bar-fill" id="sb-l" style="background:#00c8ff;width:0%"></div></div></div>
      <div class="sh-row"><span class="sh-lbl">▶ RIGHT</span><span class="sh-val" id="sv-rr">—</span><div class="sh-bar-wrap"><div class="sh-bar-fill" id="sb-rr" style="background:#00c8ff;width:0%"></div></div></div>
      <div class="sh-ir">IR: <span id="sh-ir">—</span> &nbsp; IMPACT: <span id="sh-impact" class="sh-impact-ok">○ OK</span></div>
    </div>
  </div>
  <div id="panel">
    <div class="psec">
      <div class="phdr" onclick="toggle('entity-tree')">ENTITY TREE <span>▾</span></div>
      <div class="pbody" id="entity-tree"></div>
    </div>
    <div class="psec">
      <div class="phdr" onclick="toggle('teleop')">TELEOP CONTROL <span>▾</span></div>
      <div id="teleop">
        <select id="rsel" onchange="selRid=+this.value"></select>
        <div class="tgrid">
          <div class="tbtn empty"></div>
          <div class="tbtn" id="btn-fwd" onmousedown="mv(0,-1)" onmouseup="mv(0,0)" ontouchstart="mv(0,-1)" ontouchend="mv(0,0)">▲</div>
          <div class="tbtn empty"></div>
          <div class="tbtn" id="btn-lft" onmousedown="mv(-1,0)" onmouseup="mv(0,0)" ontouchstart="mv(-1,0)" ontouchend="mv(0,0)">◀</div>
          <div class="tbtn" onmousedown="mv(0,0)" onclick="stopRobot()">■</div>
          <div class="tbtn" id="btn-rgt" onmousedown="mv(1,0)" onmouseup="mv(0,0)" ontouchstart="mv(1,0)" ontouchend="mv(0,0)">▶</div>
          <div class="tbtn empty"></div>
          <div class="tbtn" id="btn-bwd" onmousedown="mv(0,1)" onmouseup="mv(0,0)" ontouchstart="mv(0,1)" ontouchend="mv(0,0)">▼</div>
          <div class="tbtn empty"></div>
        </div>
        <div class="frow">
          <div class="fbtn" onclick="forkCmd('up')">⬆ Forks Up</div>
          <div class="fbtn" onclick="forkCmd('down')">⬇ Forks Down</div>
        </div>
        <div class="frow">
          <div class="fbtn" onclick="sio.emit('command',{text:'arm pick'})">🦾 Arm Pick</div>
          <div class="fbtn" onclick="sio.emit('command',{text:'arm drop'})">📦 Arm Drop</div>
        </div>
      </div>
    </div>
    <div class="psec">
      <div class="phdr" onclick="toggle('fleet-cards')">FLEET STATUS <span>▾</span></div>
      <div class="pbody" id="fleet-cards"></div>
    </div>
    <div class="psec" style="flex:1;display:flex;flex-direction:column;overflow:hidden;">
      <div class="phdr" onclick="toggle('log-scroll')">EVENT LOG <span>▾</span></div>
      <div class="pbody" id="log-scroll" style="flex:1;overflow-y:auto;padding:4px;"></div>
    </div>
    <div id="cmd-bar">
      <input id="cmd-in" placeholder='e.g. "robot 1 go to A3"  or  "help"' onkeydown="if(event.key==='Enter')sendCmd()">
      <button id="cmd-send" onclick="sendCmd()">▶ Send</button>
    </div>
  </div>
</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.6.1/socket.io.min.js"></script>
<script>
// ─── Socket.IO ───────────────────────────────────────────────────
const sio=io();
let FLEET_DATA=[],ARM_DATA={},CONV_DATA={belt_offset:0,items:[]};

sio.on('robots',d=>{FLEET_DATA=d; updatePanel();});
sio.on('arm',d=>{ARM_DATA=d;});
sio.on('conveyor',d=>{CONV_DATA=d;});
sio.on('log',d=>addLog(d.msg,d.type||''));
sio.on('client_cmd',d=>{
  if(d.action==='camera'){
    const mmap={free:0,top:1,follow:2,iso:3,dock:4,fl1pov:5,fl2pov:6,arm:7};
    if(mmap[d.mode]!==undefined) setCamera(mmap[d.mode],d.rid);
  }
  if(d.action==='toggle_paths') showPaths=!showPaths;
});

// ─── Three.js Setup ──────────────────────────────────────────────
const canvas=document.getElementById('c');
const wrap=document.getElementById('canvas-wrap');
const renderer=new THREE.WebGLRenderer({canvas,antialias:true});
renderer.shadowMap.enabled=true;
renderer.shadowMap.type=THREE.PCFSoftShadowMap;
renderer.setPixelRatio(window.devicePixelRatio);

const scene=new THREE.Scene();
scene.background=new THREE.Color(0x070d16);
scene.fog=new THREE.FogExp2(0x070d16,0.008);

const camera=new THREE.PerspectiveCamera(52,1,0.3,400);
camera.position.set(-20,55,-5);
camera.lookAt(0,0,0);

// Orbit controls (manual implementation)
let orb={theta:-0.4,phi:0.68,r:70,tx:0,ty:0,tz:0,dragging:false,px:0,py:0,rmb:false};
canvas.addEventListener('mousedown',e=>{orb.dragging=true;orb.px=e.clientX;orb.py=e.clientY;orb.rmb=e.button===2;});
canvas.addEventListener('contextmenu',e=>e.preventDefault());
window.addEventListener('mouseup',()=>orb.dragging=false);
window.addEventListener('mousemove',e=>{
  if(!orb.dragging||camMode!==0)return;
  const dx=(e.clientX-orb.px)*0.005,dy=(e.clientY-orb.py)*0.005;
  if(orb.rmb){orb.tx-=dx*orb.r*0.4;orb.tz-=dy*orb.r*0.4;}
  else{orb.theta-=dx;orb.phi=Math.max(0.05,Math.min(1.55,orb.phi-dy));}
  orb.px=e.clientX;orb.py=e.clientY;
  updCam();
});
canvas.addEventListener('wheel',e=>{
  if(camMode!==0)return;
  orb.r=Math.max(8,Math.min(180,orb.r+e.deltaY*0.06));
  updCam(); e.preventDefault();
},{passive:false});

function updCam(){
  const s=Math.sin(orb.phi),c=Math.cos(orb.phi);
  camera.position.set(
    orb.tx+orb.r*s*Math.sin(orb.theta),
    orb.ty+orb.r*c,
    orb.tz+orb.r*s*Math.cos(orb.theta)
  );
  camera.lookAt(orb.tx,orb.ty,orb.tz);
}
updCam();

// ─── Lighting ────────────────────────────────────────────────────
scene.add(new THREE.AmbientLight(0x334455,0.7));
const sun=new THREE.DirectionalLight(0xfff0e0,1.1);
sun.position.set(20,50,15);
sun.castShadow=true;
sun.shadow.mapSize.width=2048; sun.shadow.mapSize.height=2048;
sun.shadow.camera.near=0.1; sun.shadow.camera.far=200;
sun.shadow.camera.left=-50; sun.shadow.camera.right=50;
sun.shadow.camera.top=50; sun.shadow.camera.bottom=-50;
scene.add(sun);
// Warehouse ceiling lights (point lights in a grid)
const lights=[];
[[-18,8],[0,8],[18,8],[-18,-4],[0,-4],[18,-4]].forEach(([x,z])=>{
  const pl=new THREE.PointLight(0xeeeeff,0.9,30);
  pl.position.set(x,9.5,z);
  scene.add(pl);
  // Light fixture geometry
  const fix=new THREE.Mesh(new THREE.BoxGeometry(1.2,0.2,0.5),new THREE.MeshBasicMaterial({color:0xffffee}));
  fix.position.set(x,9.7,z);
  scene.add(fix);
});

// ─── Warehouse Geometry ──────────────────────────────────────────
function box3(x,y,z,w,h,d,col,op){
  const m=new THREE.Mesh(
    new THREE.BoxGeometry(w,h,d),
    new THREE.MeshPhongMaterial({color:col,opacity:op||1,transparent:op<1})
  );
  m.position.set(x,y,z);
  m.receiveShadow=true; m.castShadow=(op||1)>=1;
  scene.add(m); return m;
}

// Floor (two-tone tiles)
const floorGeo=new THREE.PlaneGeometry(62,38,62,38);
const floorMat=new THREE.MeshLambertMaterial({color:0x1a2230});
const floor=new THREE.Mesh(floorGeo,floorMat);
floor.rotation.x=-Math.PI/2; floor.receiveShadow=true;
scene.add(floor);
// Floor grid lines
const gridH=new THREE.GridHelper(62,62,0x223344,0x1a2a3a);
gridH.position.y=0.01; scene.add(gridH);

// Safety lanes (yellow stripes)
[[0,0.02,0,60,0.01,0.5,0xffcc00,0.6],
 [-14,0.02,0,60,0.01,0.5,0xffcc00,0.6],
 [10,0.02,0,60,0.01,0.5,0xffcc00,0.6]
].forEach(([x,y,z,w,h,d,c,o])=>box3(x,y,z,w,h,d,c,o));

// East wall (dispatch side) – keep
box3(30.15,5,0,0.3,10,38,0x1a2540,0.9);
// West wall (home side) – keep
box3(-30.15,5,0,0.3,10,38,0x1a2540,0.9);
// North & South walls REMOVED – open warehouse
// Partial structural columns along open sides
[-24,-12,0,12,24].forEach(x=>{
  box3(x,5,18.15,1.2,10,0.5,0x22334a);
  box3(x,5,-18.15,1.2,10,0.5,0x22334a);
});
// Roof structure (partial – only the beams visible)
for(let x=-24;x<=24;x+=12){
  box3(x,10.3,0,0.4,0.4,38,0x1a2844);
}
box3(0,10.3,0,62,0.4,0.4,0x1a2844);
box3(0,10.3,-9,62,0.4,0.4,0x1a2844);
box3(0,10.3,9,62,0.4,0.4,0x1a2844);

// ─── Shelving Racks ──────────────────────────────────────────────
const SHELF_DEFS=[
  {row:'A',color:0xcc8800,z:10.0},
  {row:'B',color:0x5588aa,z:1.0},
  {row:'C',color:0x667788,z:-10.0},
];
const RACK_XS=[-22,-16,-10,-4,4,10,16,22];
let RACK_DEFS_3D=[];
RACK_XS.forEach((rx,ci)=>{
  SHELF_DEFS.forEach(sd=>{
    // Upright posts
    [rx-2,rx+2].forEach(px=>{
      box3(px,2.25,sd.z,0.15,4.5,0.15,0x334455);
    });
    // Shelf boards (3 levels)
    [0.8,2.0,3.4].forEach(sh=>{
      box3(rx,sh,sd.z,4.2,0.12,0.9,sd.color===0xcc8800?0xaa6600:0x334a5c);
    });
    // Boxes/Crates on shelves
    if(sd.row==='C'){
      // Large crates
      const crat=new THREE.Mesh(
        new THREE.BoxGeometry(1.8,1.1,0.8),
        new THREE.MeshPhongMaterial({color:0x7a5230,specular:0x442200})
      );
      crat.position.set(rx,1.5,sd.z);
      crat.castShadow=true;
      scene.add(crat);
      // Crate label
    } else {
      // Small boxes
      for(let bx=-0.7;bx<=0.7;bx+=0.7){
        const b=new THREE.Mesh(
          new THREE.BoxGeometry(0.55,0.5,0.4),
          new THREE.MeshPhongMaterial({color:0x8B6914+Math.random()*0x111111})
        );
        b.position.set(rx+bx,1.0,sd.z);
        b.castShadow=true;
        scene.add(b);
      }
    }
    // Rack label sign
    const lbl=String.fromCharCode(65+SHELF_DEFS.indexOf(sd))+(ci+1);
    RACK_DEFS_3D.push({label:lbl,x:rx,z:sd.z,row:sd.row});
  });
});

// Rack label signs
RACK_DEFS_3D.forEach(r=>{
  const sg=new THREE.Mesh(
    new THREE.PlaneGeometry(0.9,0.35),
    new THREE.MeshBasicMaterial({color:r.row==='C'?0xff6622:0x00ccff,side:THREE.DoubleSide})
  );
  sg.position.set(r.x,4.2,r.z+(r.row==='A'?-1.5:r.row==='C'?1.5:0));
  sg.rotation.y=r.row==='A'?Math.PI:0;
  scene.add(sg);
});

// ─── Charging Stations ───────────────────────────────────────────
[[-26,-16],[-22,-16],[-18,-16]].forEach(([x,z])=>{
  box3(x,0.05,z,2.5,0.1,2.5,0x0033ff,0.35);
  box3(x,0.8,z,2.4,1.4,2.4,0x0044aa,0.18);
  const pl=new THREE.PointLight(0x0066ff,1.2,5);
  pl.position.set(x,2,z);
  scene.add(pl);
  box3(x,2.0,z,0.3,0.6,0.3,0x0066ff);
});

// ─── Dispatch / Conveyor Area ────────────────────────────────────
// Drop zone marker
box3(22,0.02,0,3,0.04,3,0x00ff44,0.3);
// Dispatch platform
box3(27,0.2,0,5,0.4,14,0x1e3050);

// Conveyor belt frame
const convFrame=new THREE.Mesh(
  new THREE.BoxGeometry(2.5,0.6,13),
  new THREE.MeshPhongMaterial({color:0x2a3a4a})
);
convFrame.position.set(28.5,0.5,0);
scene.add(convFrame);
// Conveyor legs
[[-5.5,-2,0,2,5.5],].flat(); // simplified
[[-5,-0.6],[5,-0.6]].forEach(([z,y])=>{
  box3(28.5,0.5+y,z,0.3,1.5,0.3,0x334455);
});
// Belt surface (animated via UV offset)
const beltGeo=new THREE.PlaneGeometry(2,12.6,1,40);
const beltMat=new THREE.MeshLambertMaterial({color:0x1a1a1a});
const beltMesh=new THREE.Mesh(beltGeo,beltMat);
beltMesh.rotation.x=-Math.PI/2;
beltMesh.position.set(28.5,0.82,0);
scene.add(beltMesh);
// Conveyor rollers
for(let z=-5.5;z<=5.5;z+=1.2){
  const rol=new THREE.Mesh(
    new THREE.CylinderGeometry(0.08,0.08,2.1,8),
    new THREE.MeshPhongMaterial({color:0x556677})
  );
  rol.rotation.z=Math.PI/2;
  rol.position.set(28.5,0.8,z);
  scene.add(rol);
}
// Conveyor side rails
box3(28.5,1.1,-6.5,2.1,0.15,0.1,0x4a5a6a);
box3(28.5,1.1,6.5,2.1,0.15,0.1,0x4a5a6a);
box3(27.45,1.0,0,0.05,0.5,13,0x3a4a5a);
box3(29.55,1.0,0,0.05,0.5,13,0x3a4a5a);

// Conveyor items pool
const convItemMeshes=[];
for(let i=0;i<12;i++){
  const isBox=i<8;
  const m=new THREE.Mesh(
    isBox?new THREE.BoxGeometry(0.6,0.5,0.5):new THREE.BoxGeometry(0.9,0.7,0.7),
    new THREE.MeshPhongMaterial({color:isBox?0x8B6914:0x7a5230})
  );
  m.visible=false;
  m.castShadow=true;
  scene.add(m);
  convItemMeshes.push(m);
}

// ─── Robot Models ────────────────────────────────────────────────
function mkTugbot(color){
  const g=new THREE.Group();
  const c=new THREE.Color(color);
  // Main body
  const body=new THREE.Mesh(new THREE.BoxGeometry(0.9,0.72,1.1),
    new THREE.MeshPhongMaterial({color:0x18203a,specular:0x334466}));
  body.position.y=0.46; body.castShadow=true; g.add(body);
  // Top panel
  const tp=new THREE.Mesh(new THREE.BoxGeometry(0.86,0.06,1.06),
    new THREE.MeshPhongMaterial({color:0x0a1426}));
  tp.position.y=0.85; g.add(tp);
  // LED ring (emissive)
  const led=new THREE.Mesh(new THREE.TorusGeometry(0.42,0.04,4,20),
    new THREE.MeshBasicMaterial({color:c}));
  led.rotation.x=Math.PI/2; led.position.y=0.5; g.add(led);
  // Drive wheels
  const wGeo=new THREE.CylinderGeometry(0.21,0.21,0.13,14);
  const wMat=new THREE.MeshPhongMaterial({color:0x111118,specular:0x333333});
  [[-0.55,0.21,0],[0.55,0.21,0]].forEach(([x,y,z])=>{
    const w=new THREE.Mesh(wGeo,wMat); w.rotation.z=Math.PI/2;
    w.position.set(x,y,z); w.castShadow=true; g.add(w);
  });
  // Caster balls
  const cMat=new THREE.MeshPhongMaterial({color:0x333344});
  [0,1].forEach(i=>{
    const cas=new THREE.Mesh(new THREE.SphereGeometry(0.11,8,6),cMat);
    cas.position.set(0,0.11,i===0?0.46:-0.46); g.add(cas);
  });
  // LIDAR unit
  const lid=new THREE.Mesh(new THREE.CylinderGeometry(0.15,0.15,0.14,12),
    new THREE.MeshPhongMaterial({color:0x0a0a0a}));
  lid.position.y=0.97; g.add(lid);
  const disc=new THREE.Mesh(new THREE.CylinderGeometry(0.13,0.13,0.04,12),
    new THREE.MeshBasicMaterial({color:c,transparent:true,opacity:0.8}));
  disc.position.y=1.06; g.add(disc);
  // Ultrasonic sensors (4 cones)
  const usgeo=new THREE.ConeGeometry(0.035,0.1,6);
  const usmat=new THREE.MeshBasicMaterial({color:0xffd700});
  [{x:0,z:0.58,ry:0},{x:0,z:-0.58,ry:Math.PI},{x:0.48,z:0,ry:-Math.PI/2},{x:-0.48,z:0,ry:Math.PI/2}].forEach(d=>{
    const us=new THREE.Mesh(usgeo,usmat);
    us.rotation.x=Math.PI/2; us.rotation.y=d.ry;
    us.position.set(d.x,0.46,d.z); g.add(us);
  });
  // Safety bumper
  const bmp=new THREE.Mesh(new THREE.BoxGeometry(0.84,0.12,0.06),
    new THREE.MeshBasicMaterial({color:0xff5500}));
  bmp.position.set(0,0.38,0.59); g.add(bmp);
  // Impact sensors (red dots at corners)
  const impMat=new THREE.MeshBasicMaterial({color:0x00ff44});
  const impMats=[];
  [[0.38,0.34,0.58],[-0.38,0.34,0.58]].forEach(([x,y,z])=>{
    const im=new THREE.Mesh(new THREE.SphereGeometry(0.055,6,4),impMat.clone());
    im.position.set(x,y,z); g.add(im); impMats.push(im.material);
  });
  return {group:g, disc, impMats, ledMat:led.material};
}

function mkForklift(color){
  const g=new THREE.Group();
  const c=new THREE.Color(color);
  // Chassis/counterweight (rear heavy)
  const chassis=new THREE.Mesh(new THREE.BoxGeometry(1.3,1.3,2.0),
    new THREE.MeshPhongMaterial({color:0xffd700,specular:0x886600}));
  chassis.position.set(0,0.75,-0.1); chassis.castShadow=true; g.add(chassis);
  // Engine cover hump
  const hump=new THREE.Mesh(new THREE.BoxGeometry(1.1,0.45,0.8),
    new THREE.MeshPhongMaterial({color:0xffcc00}));
  hump.position.set(0,1.6,-0.5); g.add(hump);
  // Cab frame
  const cab=new THREE.Mesh(new THREE.BoxGeometry(1.05,0.85,1.0),
    new THREE.MeshPhongMaterial({color:0xffcc00,specular:0x886600,
      transparent:true,opacity:0.85}));
  cab.position.set(0,1.55,0.4); g.add(cab);
  // ROPS cage posts
  const rMat=new THREE.MeshPhongMaterial({color:0xffaa00});
  const rGeo=new THREE.BoxGeometry(0.07,1.5,0.07);
  [[-0.52,2.3,-0.1],[0.52,2.3,-0.1],[-0.52,2.3,0.9],[0.52,2.3,0.9]].forEach(([x,y,z])=>{
    const p=new THREE.Mesh(rGeo,rMat); p.position.set(x,y,z); g.add(p);
  });
  const roofMesh=new THREE.Mesh(new THREE.BoxGeometry(1.15,0.07,1.1),rMat);
  roofMesh.position.set(0,3.08,0.4); g.add(roofMesh);
  // Driver seat (visible in POV)
  const seat=new THREE.Mesh(new THREE.BoxGeometry(0.5,0.15,0.4),
    new THREE.MeshPhongMaterial({color:0x221100}));
  seat.position.set(0,1.25,0.4); g.add(seat);
  const seatBk=new THREE.Mesh(new THREE.BoxGeometry(0.5,0.5,0.1),
    new THREE.MeshPhongMaterial({color:0x221100}));
  seatBk.position.set(0,1.55,0.17); g.add(seatBk);
  // Steering wheel
  const steer=new THREE.Mesh(new THREE.TorusGeometry(0.18,0.025,6,16),
    new THREE.MeshPhongMaterial({color:0x111111}));
  steer.position.set(0.2,1.65,0.65); steer.rotation.x=0.6; g.add(steer);
  // Large rear drive wheels
  const rwGeo=new THREE.CylinderGeometry(0.42,0.42,0.24,16);
  const wMat=new THREE.MeshPhongMaterial({color:0x0f0f0f,specular:0x222222});
  [[-0.76,0.42,-0.7],[0.76,0.42,-0.7]].forEach(([x,y,z])=>{
    const w=new THREE.Mesh(rwGeo,wMat); w.rotation.z=Math.PI/2;
    w.position.set(x,y,z); w.castShadow=true; g.add(w);
    // Hubcap
    const hc=new THREE.Mesh(new THREE.CylinderGeometry(0.18,0.18,0.02,8),
      new THREE.MeshPhongMaterial({color:0x888844}));
    hc.rotation.z=Math.PI/2; hc.position.set(x+(x<0?-0.13:0.13),y,z); g.add(hc);
  });
  // Front steer wheels
  const fwGeo=new THREE.CylinderGeometry(0.3,0.3,0.2,12);
  [[-0.68,0.3,0.9],[0.68,0.3,0.9]].forEach(([x,y,z])=>{
    const w=new THREE.Mesh(fwGeo,wMat); w.rotation.z=Math.PI/2;
    w.position.set(x,y,z); w.castShadow=true; g.add(w);
  });
  // Warning beacon
  const beacon=new THREE.Mesh(new THREE.CylinderGeometry(0.08,0.11,0.22,8),
    new THREE.MeshBasicMaterial({color:0xff6600}));
  beacon.position.set(0,3.22,0.4); g.add(beacon);
  // Mast assembly
  const mastMat=new THREE.MeshPhongMaterial({color:0xaaaaaa,specular:0x666666});
  const mastGeo=new THREE.BoxGeometry(0.09,3.5,0.09);
  [[-0.44,1.8,1.05],[0.44,1.8,1.05]].forEach(([x,y,z])=>{
    const m=new THREE.Mesh(mastGeo,mastMat); m.position.set(x,y,z); g.add(m);
  });
  // Fork group (animated y)
  const forkGroup=new THREE.Group();
  forkGroup.position.set(0,0.12,1.05);
  // Carriage plate
  const cpMesh=new THREE.Mesh(new THREE.BoxGeometry(1.05,0.45,0.12),
    new THREE.MeshPhongMaterial({color:0x888888}));
  cpMesh.position.set(0,0,0); forkGroup.add(cpMesh);
  // Fork arms
  const fkGeo=new THREE.BoxGeometry(0.16,0.09,1.5);
  const fkMat=new THREE.MeshPhongMaterial({color:0xbbbbbb,specular:0x888888});
  [[-0.32,0.32]].flat().forEach(xo=>{
    const fk=new THREE.Mesh(fkGeo,fkMat); fk.position.set(xo,-0.22,0.75); forkGroup.add(fk);
  });
  [-0.32,0.32].forEach(xo=>{
    const fk=new THREE.Mesh(fkGeo,fkMat); fk.position.set(xo,-0.22,0.75); forkGroup.add(fk);
  });
  g.add(forkGroup);
  const beaconMat=beacon.material;
  return {group:g, forkGroup, beaconMat};
}

// ─── Robotic Arm Model ───────────────────────────────────────────
const armScene=new THREE.Group();
armScene.position.set(25.5,0,0);   // centred between drop zone (x=22) and belt (x=28.5)
scene.add(armScene);
// Base plate
const armBase=new THREE.Mesh(new THREE.CylinderGeometry(0.85,1.0,0.18,16),
  new THREE.MeshPhongMaterial({color:0x223344,specular:0x445566}));
armBase.position.y=0.09; armScene.add(armBase);
// Turret
const turret=new THREE.Group(); turret.position.y=0.18; armScene.add(turret);
const turretBody=new THREE.Mesh(new THREE.CylinderGeometry(0.38,0.42,0.55,12),
  new THREE.MeshPhongMaterial({color:0x1a2e42,specular:0x334466}));
turretBody.position.y=0.28; turret.add(turretBody);
// Shoulder group
const shoulderGrp=new THREE.Group(); shoulderGrp.position.y=0.55; turret.add(shoulderGrp);
// Upper arm
const upperArm=new THREE.Mesh(new THREE.BoxGeometry(0.2,1.5,0.2),
  new THREE.MeshPhongMaterial({color:0xff6600,specular:0x664400}));
upperArm.position.y=0.75; shoulderGrp.add(upperArm);
// Shoulder sphere joint
const shoulderJoint=new THREE.Mesh(new THREE.SphereGeometry(0.18,10,8),
  new THREE.MeshPhongMaterial({color:0x334455}));
shoulderGrp.add(shoulderJoint);
// Elbow group
const elbowGrp=new THREE.Group(); elbowGrp.position.y=1.5; shoulderGrp.add(elbowGrp);
const elbowJoint=new THREE.Mesh(new THREE.SphereGeometry(0.15,10,8),
  new THREE.MeshPhongMaterial({color:0x334455}));
elbowGrp.add(elbowJoint);
const lowerArm=new THREE.Mesh(new THREE.BoxGeometry(0.17,1.1,0.17),
  new THREE.MeshPhongMaterial({color:0xff6600,specular:0x664400}));
lowerArm.position.y=0.55; elbowGrp.add(lowerArm);
// Wrist group
const wristGrp=new THREE.Group(); wristGrp.position.y=1.1; elbowGrp.add(wristGrp);
const wristHousing=new THREE.Mesh(new THREE.CylinderGeometry(0.11,0.11,0.28,10),
  new THREE.MeshPhongMaterial({color:0x1a1a2e}));
wristGrp.add(wristHousing);
// Gripper fingers
const gripMat=new THREE.MeshPhongMaterial({color:0xcccccc,specular:0x888888});
const fGeo=new THREE.BoxGeometry(0.055,0.04,0.28);
const finger1=new THREE.Mesh(fGeo,gripMat.clone()); finger1.position.set(-0.13,-0.18,0.14); wristGrp.add(finger1);
const finger2=new THREE.Mesh(fGeo,gripMat.clone()); finger2.position.set(0.13,-0.18,0.14);  wristGrp.add(finger2);
// Arm item mesh (follows gripper)
const armItemMesh=new THREE.Mesh(new THREE.BoxGeometry(0.7,0.55,0.6),
  new THREE.MeshPhongMaterial({color:0x8B6914}));
armItemMesh.visible=false; armItemMesh.castShadow=true; scene.add(armItemMesh);

// ─── Pallet + Wooden Crate (forklift carry model) ────────────────
function mkPalletCrate(){
  const g=new THREE.Group();

  // ── Pallet ────────────────────────────────────────────────────
  const palMat =new THREE.MeshPhongMaterial({color:0x8B6205,specular:0x221100});
  const darkMat=new THREE.MeshPhongMaterial({color:0x5c3e00});

  // 3 bottom runners (legs of the pallet)
  [-0.42,0,0.42].forEach(x=>{
    const run=new THREE.Mesh(new THREE.BoxGeometry(0.13,0.14,1.05),darkMat);
    run.position.set(x,-0.07,0); g.add(run);
  });
  // Top deck boards (visible planks with gaps)
  for(let z=-0.44;z<=0.45;z+=0.15){
    const bd=new THREE.Mesh(new THREE.BoxGeometry(1.32,0.06,0.115),palMat);
    bd.position.set(0,0.03,z); g.add(bd);
  }
  // Bottom cross-boards (between runners, for that classic pallet look)
  [-0.44,0.44].forEach(z=>{
    const cb=new THREE.Mesh(new THREE.BoxGeometry(1.32,0.04,0.10),darkMat);
    cb.position.set(0,-0.125,z); g.add(cb);
  });

  // ── Wooden Crate ──────────────────────────────────────────────
  const crateCol =0x7a5018;
  const slatCol  =0x3d2800;
  const bandCol  =0x1a0a00;
  const crateMat =new THREE.MeshPhongMaterial({color:crateCol,specular:0x221100});
  const slatMat  =new THREE.MeshPhongMaterial({color:slatCol});
  const bandMat  =new THREE.MeshBasicMaterial({color:bandCol});

  const CW=1.06, CH=0.96, CD=0.88;
  const CY=0.06+CH/2;  // crate centre y (sits on top of deck boards)

  // Crate body
  const cBody=new THREE.Mesh(new THREE.BoxGeometry(CW,CH,CD),crateMat);
  cBody.position.set(0,CY,0); cBody.castShadow=true; g.add(cBody);

  // Vertical board slats – front & back faces (z±)
  for(let xi=-0.37;xi<=0.38;xi+=0.185){
    const sl=new THREE.Mesh(new THREE.BoxGeometry(0.05,CH+0.02,0.03),slatMat);
    sl.position.set(xi,CY, CD/2+0.012); g.add(sl);
    const sl2=sl.clone(); sl2.position.z=-CD/2-0.012; g.add(sl2);
  }
  // Vertical board slats – side faces (x±)
  for(let zi=-0.3;zi<=0.31;zi+=0.3){
    const sl=new THREE.Mesh(new THREE.BoxGeometry(0.03,CH+0.02,0.05),slatMat);
    sl.position.set( CW/2+0.012,CY,zi); g.add(sl);
    const sl2=sl.clone(); sl2.position.x=-CW/2-0.012; g.add(sl2);
  }
  // Horizontal corner braces (top & bottom)
  [CY-CH/2+0.08, CY+CH/2-0.08].forEach(y=>{
    const br=new THREE.Mesh(new THREE.BoxGeometry(CW+0.03,0.055,CD+0.03),bandMat);
    br.position.set(0,y,0); g.add(br);
  });
  // Middle horizontal band
  const mid=new THREE.Mesh(new THREE.BoxGeometry(CW+0.03,0.04,CD+0.03),bandMat);
  mid.position.set(0,CY,0); g.add(mid);

  return g;
}

// ─── Simple box carry mesh (tugbot) ──────────────────────────────
function mkBoxCarry(){
  const m=new THREE.Mesh(
    new THREE.BoxGeometry(0.72,0.48,0.72),
    new THREE.MeshPhongMaterial({color:0x8B6914,specular:0x442200}));
  m.castShadow=true;
  const g=new THREE.Group(); g.add(m);
  return g;
}

// ─── Robot Meshes ────────────────────────────────────────────────
const robotMeshes=[];
const carryMeshes=[];
const trailLines=[];
const pathLines=[];
const trails=Array(6).fill(null).map(()=>[]);

function initRobots(){
  const defs=[
    {i:0,c:"#00c8ff",t:"tugbot"},{i:1,c:"#7fff00",t:"tugbot"},{i:2,c:"#ff6b35",t:"tugbot"},
    {i:3,c:"#ffd700",t:"forklift"},{i:4,c:"#ff9500",t:"forklift"},{i:5,c:"#ff3af0",t:"tugbot"},
  ];
  defs.forEach(d=>{
    const rm=d.t==="forklift"?mkForklift(d.c):mkTugbot(d.c);
    scene.add(rm.group);
    robotMeshes.push(rm);
    // Carry group – forklift gets pallet+crate, tugbot gets simple box
    const cg=d.t==="forklift"?mkPalletCrate():mkBoxCarry();
    cg.visible=false; scene.add(cg);
    carryMeshes.push(cg);
    // Trail line
    const tg=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3()]);
    const tl=new THREE.Line(tg,new THREE.LineBasicMaterial({color:new THREE.Color(d.c),opacity:0.5,transparent:true}));
    scene.add(tl); trailLines.push(tl);
    // Path line (dashed)
    const pg=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3()]);
    const pl2=new THREE.Line(pg,new THREE.LineDashedMaterial({color:new THREE.Color(d.c),dashSize:0.6,gapSize:0.4,opacity:0.7,transparent:true}));
    scene.add(pl2); pathLines.push(pl2);
  });
}
initRobots();

// ─── Label sprites (rack names) ──────────────────────────────────
function makeLabel(text,x,y,z,color){
  const cv=document.createElement('canvas'); cv.width=128; cv.height=48;
  const ctx=cv.getContext('2d');
  ctx.fillStyle='rgba(0,0,0,0.6)'; ctx.fillRect(0,0,128,48);
  ctx.font='bold 26px monospace'; ctx.fillStyle=color||'#00c8ff';
  ctx.textAlign='center'; ctx.fillText(text,64,34);
  const tex=new THREE.CanvasTexture(cv);
  const sp=new THREE.Sprite(new THREE.SpriteMaterial({map:tex,transparent:true}));
  sp.scale.set(2,0.75,1); sp.position.set(x,y,z); scene.add(sp);
}
RACK_DEFS_3D.forEach(r=>makeLabel(r.label,r.x,5.0,r.z+(r.row==='A'?-1.8:r.row==='C'?1.8:0),'#00c8ff'));
makeLabel('DISPATCH',26,1.5,0,'#00ff88');
makeLabel('STAGING',26,1.5,5.5,'#ffaa00');
makeLabel('INBOUND',26,1.5,-5.5,'#ff6644');
makeLabel('CONVEYOR',28.5,2.0,0,'#ffffff');
[[-26,-16],[-22,-16],[-18,-16]].forEach(([x,z],i)=>makeLabel(`CHG${i+1}`,x,3,z,'#4488ff'));

// ─── Camera State ────────────────────────────────────────────────
let camMode=0;
let followRid=0;
let dashOff=0;
const CAM_LABELS=['🎥 Free Orbit','🗺 Top-Down','👁 Follow Robot','📐 Isometric','🏗 Dock View',
                  '🚜 FL1 Driver POV','🚜 FL2 Driver POV','🦾 Arm / Conveyor'];

function setCamera(mode,rid){
  camMode=mode;
  if(rid!==undefined) followRid=rid;
  document.querySelectorAll('.cam-btn').forEach((b,i)=>b.classList.toggle('active',i===mode));
  document.getElementById('cam-label').textContent=CAM_LABELS[mode]||'';
  if(mode===0) updCam();
}

function applyCamera(){
  const r=FLEET_DATA[followRid]||FLEET_DATA[0];
  const tgt=r?new THREE.Vector3(r.x,0,r.z):new THREE.Vector3(0,0,0);
  if(camMode===0){ updCam(); return; }
  const cx=r?r.x:0, cz=r?r.z:0;
  if(camMode===1){
    camera.position.set(0,80,0.001);
    camera.lookAt(0,0,0);
  } else if(camMode===2){
    const yaw=(r&&r.yaw)||0;
    camera.position.set(cx-Math.cos(yaw)*12,9,cz-Math.sin(yaw)*12);
    camera.lookAt(cx,1,cz);
  } else if(camMode===3){
    camera.position.set(55,55,55);
    camera.lookAt(0,0,0);
  } else if(camMode===4){
    camera.position.set(42,10,0);
    camera.lookAt(0,0,0);
  } else if(camMode===5||camMode===6){
    const fIdx=camMode===5?3:4;
    const fl=FLEET_DATA[fIdx];
    if(fl){
      // Forklift faces (cos(yaw), 0, sin(yaw)) in world with corrected π/2-yaw rotation
      const fwdX=Math.cos(fl.yaw), fwdZ=Math.sin(fl.yaw);
      // Driver sits at roughly x=0.0, y=1.8, z=0.45 in forklift local space
      // In world: base + fwd*0.5 (slightly ahead)
      camera.position.set(fl.x+fwdX*0.5, 2.1, fl.z+fwdZ*0.5);
      camera.lookAt(fl.x+fwdX*20, 1.8, fl.z+fwdZ*20);
    }
  } else if(camMode===7){
    camera.position.set(26.8,10,-6);
    camera.lookAt(26.8,1,2);
  }
}

// ─── Floor click → robot goto ────────────────────────────────────
const raycaster=new THREE.Raycaster();
const mouse2=new THREE.Vector2();
let clickTimer=null;
canvas.addEventListener('click',e=>{
  if(camMode!==0&&camMode!==1&&camMode!==3)return;
  const rect=canvas.getBoundingClientRect();
  mouse2.x=((e.clientX-rect.left)/rect.width)*2-1;
  mouse2.y=-((e.clientY-rect.top)/rect.height)*2+1;
  raycaster.setFromCamera(mouse2,camera);
  const plane=new THREE.Plane(new THREE.Vector3(0,1,0),0);
  const hit=new THREE.Vector3();
  raycaster.ray.intersectPlane(plane,hit);
  if(hit){
    sio.emit('cmd',{action:'goto',rid:selRid,x:Math.round(hit.x*2)/2,z:Math.round(hit.z*2)/2});
    addLog(`🖱 Robot ${selRid+1} → (${hit.x.toFixed(1)},${hit.z.toFixed(1)})`,'cmd');
    // Click marker
    showClickMarker(hit.x,hit.z);
  }
});

let clickMarker=null;
function showClickMarker(x,z){
  if(!clickMarker){
    clickMarker=new THREE.Mesh(new THREE.RingGeometry(0.3,0.5,16),
      new THREE.MeshBasicMaterial({color:0x00ff88,side:THREE.DoubleSide,transparent:true,opacity:0.9}));
    clickMarker.rotation.x=-Math.PI/2; clickMarker.position.y=0.05; scene.add(clickMarker);
  }
  clickMarker.position.set(x,0.05,z); clickMarker.visible=true; clickMarker.scale.set(1,1,1);
  clearTimeout(clickTimer);
  clickTimer=setTimeout(()=>{if(clickMarker)clickMarker.visible=false;},2500);
}

// ─── Panel State ─────────────────────────────────────────────────
let selRid=0;
let showPaths=true;
let simTime=0; let startTime=Date.now();
let paused=false;

function toggle(id){
  const el=document.getElementById(id);
  el.style.display=el.style.display==='none'?'':'none';
}

function updatePanel(){
  if(!FLEET_DATA.length)return;
  // Entity tree
  const et=document.getElementById('entity-tree');
  et.innerHTML='';
  FLEET_DATA.forEach(r=>{
    const row=document.createElement('div');
    row.className='e-row'+(r.id===selRid?' sel':'');
    row.onclick=()=>{selRid=r.id;document.getElementById('rsel').value=r.id;updatePanel();};
    row.innerHTML=`<div class="e-dot" style="background:${r.color}"></div>
      <span>${r.name}</span>
      <span class="e-type">${r.vtype}</span>`;
    et.appendChild(row);
  });
  // Arm row
  const armRow=document.createElement('div'); armRow.className='e-row';
  armRow.innerHTML=`<div class="e-dot" style="background:#ff6600"></div><span>RoboticArm</span><span class="e-type">arm/${ARM_DATA.state||'idle'}</span>`;
  et.appendChild(armRow);
  // Fleet cards
  const fc=document.getElementById('fleet-cards');
  fc.innerHTML='';
  FLEET_DATA.forEach(r=>{
    const pct=r.battery;
    const batCol=pct>50?'#7fff00':pct>20?'#ffd700':'#ff4444';
    const div=document.createElement('div');
    div.className='rcard'+(r.id===selRid?' sel':'');
    div.onclick=()=>{selRid=r.id;document.getElementById('rsel').value=r.id;};
    div.innerHTML=`<div class="rcard-top">
      <span class="rname" style="color:${r.color}">${r.name}</span>
      <span class="rstate st-${r.state}">${r.state.toUpperCase()}</span>
    </div>
    <div class="rinfo">
      <span>⚡ ${pct.toFixed(0)}%</span>
      ${r.carrying?`<span>📦 ${r.carrying}</span>`:''}
      ${r.sensor&&r.sensor.impact?`<span style="color:#f44">⚠ IMPACT</span>`:''}
    </div>
    <div class="bat-bar"><div class="bat-fill" style="width:${pct}%;background:${batCol}"></div></div>
    ${r.carrying?`<div class="carrying-tag">Carrying: ${r.carrying}</div>`:''}`;
    fc.appendChild(div);
  });
  // Update teleop selector
  const rsel=document.getElementById('rsel');
  if(!rsel.options.length){
    FLEET_DATA.forEach(r=>{
      const o=document.createElement('option'); o.value=r.id;
      o.textContent=`${r.id+1}. ${r.name}`; rsel.appendChild(o);
    });
  }
  // Update sensor HUD for selected robot
  const sr=FLEET_DATA[selRid];
  if(sr&&sr.sensor){
    const s=sr.sensor;
    document.getElementById('sh-name').textContent=sr.name;
    const dirs=[['f','front'],['r','rear'],['l','left'],['rr','right']];
    const svIds=['sv-f','sv-r','sv-l','sv-rr'];
    const sbIds=['sb-f','sb-r','sb-l','sb-rr'];
    dirs.forEach(([k,dn],i)=>{
      const v=s.ultrasonic[dn];
      document.getElementById(svIds[i]).textContent=v.toFixed(1)+'m';
      const pct=Math.min(100,(v/8)*100);
      const el=document.getElementById(sbIds[i]);
      el.style.width=pct+'%';
      el.style.background=v<1.5?'#ff4444':v<3?'#ffd700':'#00c8ff';
    });
    document.getElementById('sh-ir').textContent=s.ir.floor;
    const imp=document.getElementById('sh-impact');
    imp.textContent=s.impact?'⚠ WARNING':'○ OK';
    imp.className=s.impact?'sh-impact-warn':'sh-impact-ok';
  }
}

// ─── Main Animation Loop ─────────────────────────────────────────
function animate(){
  requestAnimationFrame(animate);
  const now=Date.now();
  simTime=(now-startTime)/1000;
  dashOff-=0.03;
  // Beacon flicker
  const beaconOn=Math.sin(now*0.008)>0;

  // Apply camera
  applyCamera();

  // Update robot meshes
  FLEET_DATA.forEach((r,i)=>{
    const rm=robotMeshes[i]; if(!rm)return;
    rm.group.position.set(r.x,0,r.z);
    // Models face +Z local → correct world facing = π/2 - yaw
    rm.group.rotation.y=Math.PI/2-r.yaw;
    // Forklift fork animation
    if(r.vtype==='forklift'&&rm.forkGroup){
      rm.forkGroup.position.y=r.fork_height+0.12;
    }
    // LIDAR spin
    if(rm.disc) rm.disc.rotation.y=now*0.005;
    // Impact sensor flash
    if(rm.impMats){
      const col=r.sensor&&r.sensor.impact?0xff2200:0x00ff44;
      rm.impMats.forEach(m=>m.color.set(col));
    }
    // Beacon blink (forklifts)
    if(rm.beaconMat){
      rm.beaconMat.color.set(beaconOn&&r.state==='moving'?0xff6600:0x663300);
    }
    // LED ring color based on state
    if(rm.ledMat){
      const stateColors={'idle':0x00c8ff,'moving':0x00ff44,'picking':0xffd700,
        'dropping':0xffaa00,'charging':0x0044ff,'manual':0xff3af0};
      rm.ledMat.color.set(stateColors[r.state]||0x00c8ff);
    }
    // Carry group (pallet+crate for forklifts, box for tugbots)
    const cm=carryMeshes[i];
    if(r.carrying&&r.state!=='idle'){
      cm.visible=true;
      if(r.vtype==='forklift'){
        // Place pallet on fork surface – pallet base y=0 in group, forks at fork_height+0.12
        const fwdX=Math.cos(r.yaw), fwdZ=Math.sin(r.yaw);
        cm.position.set(
          r.x + fwdX*1.35,        // 1.35m ahead along fork direction
          r.fork_height + 0.14,   // sit flush on the fork surface
          r.z + fwdZ*1.35);
        cm.rotation.y=Math.PI/2-r.yaw;
      } else {
        // Tugbot: box rides on top of the robot body
        cm.position.set(r.x, 0.9, r.z);
        cm.rotation.y=Math.PI/2-r.yaw;
      }
    } else {
      cm.visible=false;
    }
    // Trail
    const tr=trails[i];
    tr.push(new THREE.Vector3(r.x,0.15,r.z));
    if(tr.length>180) tr.shift();
    if(tr.length>1){
      trailLines[i].geometry.setFromPoints(tr);
      trailLines[i].visible=showPaths;
    }
    // Planned path
    if(showPaths&&r.path&&r.path.length>1){
      const pts=r.path.map(p=>new THREE.Vector3(p[0],0.18,p[1]));
      pathLines[i].geometry.setFromPoints(pts);
      pathLines[i].geometry.computeBoundingBox();
      pathLines[i].computeLineDistances();
      pathLines[i].material.dashOffset=dashOff;
      pathLines[i].visible=true;
    } else {
      pathLines[i].visible=false;
    }
  });

  // Update arm model
  if(ARM_DATA&&ARM_DATA.base!==undefined){
    // Turret: rotate around Y axis (base angle from server)
    turret.rotation.y = ARM_DATA.base;
    // Shoulder: bend in the arm's local ZX plane (tilt arm outward)
    // Negative z-rotation leans arm in the direction turret is facing
    shoulderGrp.rotation.z = -(Math.PI/2 - ARM_DATA.shoulder);
    // Elbow: bend relative to upper arm (positive = further bend)
    elbowGrp.rotation.z = ARM_DATA.elbow * 0.9;
    // Wrist: small rotation for realism
    wristGrp.rotation.z = ARM_DATA.wrist || 0;
    // Gripper fingers open/close — 0=open, 1=closed
    const openAmt = 1.0 - ARM_DATA.gripper;  // 1=open, 0=closed
    finger1.position.x = -0.07 - openAmt*0.11;
    finger2.position.x =  0.07 + openAmt*0.11;
    // Arm item mesh: use server-computed grip world position
    if(ARM_DATA.has_item && ARM_DATA.state!=='idle'){
      armItemMesh.visible = true;
      // Use server FK result — direct world coordinates
      armItemMesh.position.set(
        ARM_DATA.grip_x !== undefined ? ARM_DATA.grip_x : 25.5,
        ARM_DATA.grip_y !== undefined ? ARM_DATA.grip_y - 0.2 : 1.5,
        ARM_DATA.grip_z !== undefined ? ARM_DATA.grip_z : 0
      );
      armItemMesh.rotation.y = ARM_DATA.base;
    } else {
      armItemMesh.visible = false;
    }
    // Arm state label colour
    const armStateColors={idle:0xff6600,open_grip:0xffaa00,swing_drop:0x00c8ff,
      descend:0xffd700,close_grip:0xff4400,ascend:0x00ff88,
      swing_belt:0x00c8ff,lower_belt:0xffd700,release:0xff8800,return:0x888888};
    if(armScene.children[0])
      armScene.children[0].material&&(armScene.children[0].material.emissive&&
        armScene.children[0].material.emissive.set(armStateColors[ARM_DATA.state]||0x000000));
  }

  // Update conveyor items
  convItemMeshes.forEach(m=>m.visible=false);
  if(CONV_DATA.items){
    CONV_DATA.items.forEach((it,i)=>{
      if(i<convItemMeshes.length){
        convItemMeshes[i].visible=true;
        convItemMeshes[i].position.set(28.5,1.1,it.z);
        convItemMeshes[i].rotation.y=it.z*0.3;
      }
    });
  }
  // Belt UV scroll (just subtle color pulse for now)
  beltMesh.material.color.setHSL(0.6,0.2,0.08+0.02*Math.sin(now*0.003+CONV_DATA.belt_offset*3));

  // Timer
  const elapsed=Math.floor((Date.now()-startTime)/1000);
  document.getElementById('timer').textContent=
    `${String(Math.floor(elapsed/3600)).padStart(2,'0')}:${String(Math.floor(elapsed/60)%60).padStart(2,'0')}:${String(elapsed%60).padStart(2,'0')}`;

  renderer.render(scene,camera);
}

// ─── Controls ────────────────────────────────────────────────────
function estop(){ sio.emit('cmd',{action:'estop'}); }
function togglePause(){ sio.emit('cmd',{action:'pause'}); }
function togglePaths(){ showPaths=!showPaths; }
function mv(vx,vz){ sio.emit('cmd',{action:'manual',rid:selRid,vx,vz}); }
function stopRobot(){
  sio.emit('cmd',{action:'manual',rid:selRid,vx:0,vz:0});
  sio.emit('command',{text:`robot ${selRid+1} all stop`});
}
function forkCmd(dir){
  sio.emit('command',{text:`forklift ${selRid-1} forks ${dir}`});
}
function sendCmd(){
  const t=document.getElementById('cmd-in').value.trim();
  if(!t)return;
  sio.emit('command',{text:t});
  document.getElementById('cmd-in').value='';
}
function addLog(msg,type){
  const el=document.getElementById('log-scroll');
  const d=document.createElement('div');
  d.className='log-entry log-'+(type||'');
  d.textContent=`[${new Date().toLocaleTimeString()}] ${msg}`;
  el.appendChild(d);
  if(el.children.length>80) el.removeChild(el.firstChild);
  el.scrollTop=el.scrollHeight;
}

// Keyboard shortcuts
document.addEventListener('keydown',e=>{
  const tgt=document.activeElement.tagName;
  if(tgt==='INPUT'||tgt==='TEXTAREA')return;
  if(e.key>='1'&&e.key<='8') setCamera(+e.key-1);
  if(e.key==='Escape') estop();
  if(e.key===' ') { e.preventDefault(); sio.emit('cmd',{action:'pause'}); }
  if(e.key==='ArrowUp')    mv(0,-1);
  if(e.key==='ArrowDown')  mv(0,1);
  if(e.key==='ArrowLeft')  mv(-1,0);
  if(e.key==='ArrowRight') mv(1,0);
  if(e.key==='p'||e.key==='P') showPaths=!showPaths;
});
document.addEventListener('keyup',e=>{
  if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)) mv(0,0);
});

// Resize handler
function onResize(){
  const w=wrap.clientWidth,h=wrap.clientHeight;
  renderer.setSize(w,h);
  camera.aspect=w/h; camera.updateProjectionMatrix();
}
window.addEventListener('resize',onResize);
onResize();

// Start
animate();
addLog('🏭 Warehouse Sim v4 started — Sensor Edition','cmd');
addLog('📡 Ultrasonic/IR/Impact sensors active','');
addLog('🚀 Fleet auto-assigned — watch the workflow!','');
addLog('💡 Press keys 1-8 for camera modes, P to toggle paths','');
addLog('🚜 Forklifts (yellow) = C-row CRATES only','');
addLog('📦 TugBots = A & B row BOXES','');
</script>
</body>
</html>
"""

if __name__=="__main__":
    print("="*55)
    print("  Warehouse Automation Sim v4 — Sensor Edition")
    print("="*55)
    print("  Open: http://localhost:5001")
    print("  Keys: 1-8 = cameras, P = paths, ESC = E-STOP")
    print("  Arrows = drive selected robot")
    print("  Commands: 'robot 1 go to A3', 'forklift 1 pick from C4'")
    print("  Forklifts handle CRATES (C-row) only")
    print("  TugBots handle BOXES (A & B rows)")
    print("="*55)
    sio.run(app,host="0.0.0.0",port=5001,debug=False)
