import math
import json
from collections import deque

N_CELLS = 5                 # racs meret
CELL = 0.20                 # cella [m]
ORIGIN = -0.5               # bal also sarok
WALL_HALF = 0.005           # fal fele
START_CELL = (0, 0)
GOAL_CELLS = [(4, 4)]

DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]     # E, N, W, S
DIR_NAMES = "ENWS"

UNKNOWN, OPEN, WALL = 0, 1, 2


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def cell_center(c):
    return (ORIGIN + CELL * (c[0] + 0.5), ORIGIN + CELL * (c[1] + 0.5))


def cell_of(x, y):
    i = int(math.floor((x - ORIGIN) / CELL))
    j = int(math.floor((y - ORIGIN) / CELL))
    return (min(max(i, 0), N_CELLS - 1), min(max(j, 0), N_CELLS - 1))


def heading_to_dir(theta):
    return int(round(wrap(theta) / (math.pi / 2.0))) % 4


# terkep
class MazeMap:

    def __init__(self):
        self.walls = [[[UNKNOWN] * 4 for _ in range(N_CELLS)] for _ in range(N_CELLS)]
        self.visited = [[False] * N_CELLS for _ in range(N_CELLS)]
        for i in range(N_CELLS):
            for j in range(N_CELLS):
                for d, (di, dj) in enumerate(DIRS):
                    if not self.inside((i + di, j + dj)):
                        self.walls[i][j][d] = WALL

    @staticmethod
    def inside(c):
        return 0 <= c[0] < N_CELLS and 0 <= c[1] < N_CELLS

    def get(self, c, d):
        return self.walls[c[0]][c[1]][d]

    def set(self, c, d, state):
        n = (c[0] + DIRS[d][0], c[1] + DIRS[d][1])
        if not self.inside(n):
            return False           # perem fal
        changed = self.walls[c[0]][c[1]][d] != state
        self.walls[c[0]][c[1]][d] = state
        self.walls[n[0]][n[1]][(d + 2) % 4] = state
        return changed

    def flood_fill(self, goals=GOAL_CELLS, optimistic=True):
        INF = 10 ** 6
        dist = [[INF] * N_CELLS for _ in range(N_CELLS)]
        q = deque()
        for g in goals:
            dist[g[0]][g[1]] = 0
            q.append(g)
        while q:
            c = q.popleft()
            for d, (di, dj) in enumerate(DIRS):
                s = self.get(c, d)
                if s == WALL or (not optimistic and s != OPEN):
                    continue
                n = (c[0] + di, c[1] + dj)
                if self.inside(n) and dist[n[0]][n[1]] > dist[c[0]][c[1]] + 1:
                    dist[n[0]][n[1]] = dist[c[0]][c[1]] + 1
                    q.append(n)
        return dist

    def best_dir(self, c, heading_dir):
        dist = self.flood_fill()
        best, best_key = None, None
        for d, (di, dj) in enumerate(DIRS):
            if self.get(c, d) == WALL:
                continue
            n = (c[0] + di, c[1] + dj)
            turn = min((d - heading_dir) % 4, (heading_dir - d) % 4)
            key = (dist[n[0]][n[1]], turn)
            if best_key is None or key < best_key:
                best, best_key = d, key
        if best is None or best_key[0] >= 10 ** 6:
            return None
        return best

    def path_length(self, start=START_CELL, optimistic=True):
        return self.flood_fill(optimistic=optimistic)[start[0]][start[1]]

    # mentes
    def to_json(self):
        return json.dumps({"walls": self.walls, "visited": self.visited})

    @classmethod
    def from_json(cls, s):
        m = cls()
        data = json.loads(s)
        m.walls = data["walls"]
        m.visited = data.get("visited", m.visited)
        return m

    def ascii(self, robot=None):
        ch = {UNKNOWN: ".", OPEN: " ", WALL: "#"}
        lines = []
        for j in range(N_CELLS - 1, -1, -1):
            top = "+"
            mid = ""
            for i in range(N_CELLS):
                top += {UNKNOWN: " . ", OPEN: "   ", WALL: "---"}[self.get((i, j), 1)] + "+"
                w = self.get((i, j), 2)
                mid += {UNKNOWN: ".", OPEN: " ", WALL: "|"}[w]
                mark = " "
                if robot == (i, j):
                    mark = "R"
                elif (i, j) in GOAL_CELLS:
                    mark = "G"
                elif self.visited[i][j]:
                    mark = "o"
                mid += " " + mark + " "
            mid += {UNKNOWN: ".", OPEN: " ", WALL: "|"}[self.get((N_CELLS - 1, j), 0)]
            lines.append(top)
            lines.append(mid)
        bot = "+"
        for i in range(N_CELLS):
            bot += {UNKNOWN: " . ", OPEN: "   ", WALL: "---"}[self.get((i, 0), 3)] + "+"
        lines.append(bot)
        _ = ch
        return "\n".join(lines)


# parameterek
class RobotParams:
    WHEEL_RADIUS = 0.022
    WHEEL_BASE = 0.090
    PAIR_BASE = 0.050
    SENSORS = {
        "F":  (0.040, 0.000, 0.0),
        "FL": (0.028, 0.028, math.pi / 4),
        "FR": (0.028, -0.028, -math.pi / 4),
        "LF": (0.025, 0.035, math.pi / 2),
        "LB": (-0.025, 0.035, math.pi / 2),
        "RF": (0.025, -0.035, -math.pi / 2),
        "RB": (-0.025, -0.035, -math.pi / 2),
    }
    SENSOR_RANGE = 0.30

# odometria
class Pose:
    def __init__(self, x, y, th):
        self.x, self.y, self.th = x, y, th
        self.prev = None

    def update(self, phi_l, phi_r, p=RobotParams):
        if self.prev is None:
            self.prev = (phi_l, phi_r)
            return 0.0
        dl = wrap(phi_l - self.prev[0]) * p.WHEEL_RADIUS
        dr = wrap(phi_r - self.prev[1]) * p.WHEEL_RADIUS
        self.prev = (phi_l, phi_r)
        ds = 0.5 * (dl + dr)
        dth = (dr - dl) / p.WHEEL_BASE
        mid = self.th + 0.5 * dth
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.th = wrap(self.th + dth)
        return ds


# vezerlo allapotgep
class MazeController:
    # parameterek
    V_EXPLORE = 0.12            # [m/s]
    V_FAST = 0.16               # masodik menet
    V_MIN = 0.02
    ACC = 0.25                  # [m/s^2]
    W_MAX = 3.0                 # [rad/s]
    W_ACC = 8.0                 # [rad/s^2]
    K_LAT = 25.0                # oldalhiba
    K_HEAD = 5.0                # iranyhiba
    WALL_TH = 0.12              # fal kuszob
    SIDE_VALID = 0.11           # oldal max
    CORR_GAIN = 0.25            # pozicio suly
    HEAD_CORR_GAIN = 0.3        # irany suly
    SNAP_TOL = 0.06             # racs tures
    EDGE_GAIN = 0.6             # falveg suly
    SETTLE_STEPS = 3
    SENSE_STEPS = 4

    def __init__(self, x0, y0, th0, prior_map_json=None, log=print):
        self.log = log
        self.pose = Pose(x0, y0, wrap(round(th0 / (math.pi / 2)) * math.pi / 2))
        self.fast = False
        self.map = MazeMap()
        if prior_map_json:
            try:
                self.map = MazeMap.from_json(prior_map_json)
                self.fast = True
                proven = self.map.path_length(optimistic=False)
                opt = self.map.path_length(optimistic=True)
                self.log("[MAP] Elozo menet terkepe betoltve. Legrovidebb ut: %d lepes%s"
                         % (opt, " (bizonyitottan optimalis)" if proven == opt else
                            " (meg nem bizonyitott, ismeretlen reszeken at)"))
            except Exception as e:
                self.log("[MAP] Elozo terkep nem olvashato (%s), uj felfedezes." % e)
                self.map = MazeMap()
        self.cell = cell_of(self.pose.x, self.pose.y)
        self.state = "SETTLE"
        self.t_state = 0
        self.v = 0.0
        self.w = 0.0
        self.target_cell = None
        self.target_dir = heading_to_dir(self.pose.th)
        self.done = False
        self.edge_prev = {}
        self.steps = 0
        self.cells_moved = 0
        self.path = [self.cell]
        self.map_changed = False

    # helpers
    def _ramp(self, cur, target, rate, dt):
        step = rate * dt
        if target > cur + step:
            return cur + step
        if target < cur - step:
            return cur - step
        return target

    def _wheels(self, v, w, p=RobotParams):
        wl = (v - w * p.WHEEL_BASE / 2.0) / p.WHEEL_RADIUS
        wr = (v + w * p.WHEEL_BASE / 2.0) / p.WHEEL_RADIUS
        return wl, wr

    def _sense_walls(self, dists):
        h = heading_to_dir(self.pose.th)
        out = {}
        for names, rel in ((("F",), 0), (("LF", "LB"), 1), (("RF", "RB"), 3)):
            vals = [dists.get(n) for n in names if dists.get(n) is not None]
            out[(h + rel) % 4] = bool(vals) and min(vals) < self.WALL_TH
        return out

    def _wall_correction(self, dists):
        p = self.pose
        h = heading_to_dir(p.th)
        axis_th = h * math.pi / 2
        if abs(wrap(p.th - axis_th)) > math.radians(15):
            return
        for name in ("F", "LF", "LB", "RF", "RB"):
            d = dists.get(name)
            if d is None:
                continue
            if name == "F" and d > 0.29:
                continue
            if name != "F" and d > self.SIDE_VALID:
                continue
            sx, sy, sa = RobotParams.SENSORS[name]
            ray_th = wrap(p.th + sa)
            ray_dir = heading_to_dir(ray_th)
            ox = p.x + sx * math.cos(p.th) - sy * math.sin(p.th)
            oy = p.y + sx * math.sin(p.th) + sy * math.cos(p.th)
            hx = ox + d * math.cos(ray_th)
            hy = oy + d * math.sin(ray_th)
            if ray_dir in (0, 2):  # x
                sign = 1 if ray_dir == 0 else -1
                line = ORIGIN + CELL * round((hx + sign * WALL_HALF - ORIGIN) / CELL)
                face = line - sign * WALL_HALF
                err = face - hx
                if abs(err) < (0.08 if name == "F" else self.SNAP_TOL):
                    p.x += self.CORR_GAIN * err
            else:  # y
                sign = 1 if ray_dir == 1 else -1
                line = ORIGIN + CELL * round((hy + sign * WALL_HALF - ORIGIN) / CELL)
                face = line - sign * WALL_HALF
                err = face - hy
                if abs(err) < (0.08 if name == "F" else self.SNAP_TOL):
                    p.y += self.CORR_GAIN * err

    def _heading_correction(self, dists, gain):
        axis_th = heading_to_dir(self.pose.th) * math.pi / 2
        meas = []
        for f, b, sgn in (("LF", "LB", -1.0), ("RF", "RB", 1.0)):
            df, db = dists.get(f), dists.get(b)
            if df is None or db is None or max(df, db) > self.SIDE_VALID:
                continue
            diff = sgn * (df - db)
            if abs(diff) > RobotParams.PAIR_BASE * math.tan(math.radians(20)):
                continue
            meas.append(math.atan2(diff, RobotParams.PAIR_BASE))
        if meas:
            e = sum(meas) / len(meas)
            th_meas = wrap(axis_th + e)
            self.pose.th = wrap(self.pose.th + gain * wrap(th_meas - self.pose.th))
            return True
        return False

    def _edge_correction(self, dists):
        p = self.pose
        d = heading_to_dir(p.th)
        if abs(wrap(p.th - d * math.pi / 2)) > math.radians(10):
            self.edge_prev = {}
            return
        ux, uy = DIRS[d]
        a_now = p.x * ux + p.y * uy
        for name in ("LF", "LB", "RF", "RB"):
            v = dists.get(name)
            near = v is not None and v < self.SIDE_VALID
            prev = self.edge_prev.get(name)
            self.edge_prev[name] = (near, a_now)
            if prev is None or prev[0] == near:
                continue
            sx = RobotParams.SENSORS[name][0]
            s_est = 0.5 * (prev[1] + a_now) + sx
            sgn = ux + uy
            line = sgn * (ORIGIN + CELL * round((sgn * s_est - ORIGIN) / CELL))
            expected = line - WALL_HALF if near else line + WALL_HALF
            err = expected - s_est
            if abs(err) < 0.04:
                k = self.EDGE_GAIN * err
                p.x += k * ux
                p.y += k * uy
                for n2, (nr, a2) in list(self.edge_prev.items()):
                    self.edge_prev[n2] = (nr, a2 + k)

    # main step
    def step(self, dt, phi_l, phi_r, dists):
        self.steps += 1
        ds = self.pose.update(phi_l, phi_r)
        self.t_state += 1

        if self.state == "SETTLE":
            self.v = self.w = 0.0
            if self.t_state > self.SETTLE_STEPS:
                self._enter("SENSE")
            return 0.0, 0.0

        if self.state == "DONE":
            self.v = self._ramp(self.v, 0.0, self.ACC, dt)
            self.w = 0.0
            return self._wheels(self.v, 0.0)

        if self.state == "SENSE":
            self.v = self.w = 0.0
            self._heading_correction(dists, 0.5)
            self._wall_correction(dists)
            if self.t_state >= self.SENSE_STEPS:
                return self._decide(dists)
            return 0.0, 0.0

        if self.state == "TURN":
            return self._turn(dt, dists)

        if self.state == "POSTTURN":
            self._heading_correction(dists, 0.5)
            self._wall_correction(dists)
            if self.t_state >= self.SENSE_STEPS:
                err = wrap(self.target_dir * math.pi / 2 - self.pose.th)
                if abs(err) > math.radians(2.5):
                    self._enter("TURN")
                else:
                    self._start_drive()
            return 0.0, 0.0

        if self.state == "DRIVE":
            return self._drive(dt, dists, ds)

        return 0.0, 0.0

    def _enter(self, st):
        self.state = st
        self.t_state = 0

    def _decide(self, dists):
        p = self.pose
        self.cell = cell_of(p.x, p.y)
        c = self.cell
        self.map.visited[c[0]][c[1]] = True
        # 1. falak
        for d, is_wall in self._sense_walls(dists).items():
            if self.map.get(c, d) != WALL or not is_wall:
                if self.map.set(c, d, WALL if is_wall else OPEN):
                    self.map_changed = True
        # 2. cel?
        if c in GOAL_CELLS:
            self.done = True
            self.log("[CEL] Megerkezett a %s cellaba %d cellalepes utan. Pozicio: (%.3f, %.3f)"
                     % (str(c), self.cells_moved, p.x, p.y))
            self.log(self.map.ascii(robot=c))
            self._enter("DONE")
            return 0.0, 0.0
        # 3. flood fill
        h = heading_to_dir(p.th)
        d = self.map.best_dir(c, h)
        if d is None:
            self.log("[MAP] A cel nem erheto el a terkep szerint, terkep torlese.")
            fresh = MazeMap()
            for dd, is_wall in self._sense_walls(dists).items():
                fresh.set(c, dd, WALL if is_wall else OPEN)
            self.map = fresh
            d = self.map.best_dir(c, h)
            if d is None:
                self._enter("DONE")
                return 0.0, 0.0
        self.target_dir = d
        self.target_cell = (c[0] + DIRS[d][0], c[1] + DIRS[d][1])
        if d == h:
            self._start_drive()
        else:
            self._enter("TURN")
        return 0.0, 0.0

    def _start_drive(self):
        self.edge_prev = {}
        self._enter("DRIVE")

    def _turn(self, dt, dists):
        target = self.target_dir * math.pi / 2
        err = wrap(target - self.pose.th)
        w_cmd = math.copysign(max(0.25, min(self.W_MAX, 0.8 * math.sqrt(2 * self.W_ACC * abs(err)))), err)
        if abs(w_cmd) > abs(self.w) or w_cmd * self.w < 0:
            self.w = self._ramp(self.w, w_cmd, self.W_ACC, dt)
        else:
            self.w = w_cmd
        self.v = 0.0
        if abs(err) < math.radians(1.5):
            self.w = 0.0
            self._enter("POSTTURN")
        return self._wheels(0.0, self.w)

    def _drive(self, dt, dists, ds):
        p = self.pose
        d = self.target_dir
        axis_th = d * math.pi / 2
        ux, uy = DIRS[d]
        tx, ty = cell_center(self.target_cell)
        rem = (tx - p.x) * ux + (ty - p.y) * uy
        lat = -(p.x - tx) * uy + (p.y - ty) * ux

        # korrekcio
        self._heading_correction(dists, self.HEAD_CORR_GAIN)
        self._wall_correction(dists)
        self._edge_correction(dists)

        # biztonsagi megallas
        f = dists.get("F")
        if f is not None and f < 0.035:
            rem = min(rem, 0.0)

        vmax = self.V_FAST if self.fast else self.V_EXPLORE
        v_cmd = min(vmax, math.sqrt(max(0.0, 2 * self.ACC * max(rem, 0.0))) + self.V_MIN)
        if rem <= 0.002:
            v_cmd = 0.0
        self.v = self._ramp(self.v, v_cmd, self.ACC, dt)
        if self.v > v_cmd:
            self.v = v_cmd

        e_head = wrap(p.th - axis_th)
        w_cmd = -self.K_HEAD * e_head - self.K_LAT * lat * (0.4 + self.v / vmax)
        # sarok elkerules
        fl, fr = dists.get("FL"), dists.get("FR")
        if fl is not None and fl < 0.045:
            w_cmd -= 1.5
        if fr is not None and fr < 0.045:
            w_cmd += 1.5
        self.w = max(-self.W_MAX, min(self.W_MAX, w_cmd))

        if rem <= 0.002 and self.v < 1e-3:
            self.v = self.w = 0.0
            self.cells_moved += 1
            self.path.append(self.target_cell)
            self._enter("SENSE")
            return 0.0, 0.0
        return self._wheels(self.v, self.w)
