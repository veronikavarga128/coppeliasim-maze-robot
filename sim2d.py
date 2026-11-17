"""
2D-s kinematikai tesztszimulator a labirintus-vezerlohoz
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import maze_controller as mc

ROBOT_RADIUS = 0.0555


# labirintusok
def maze_from_figure():
    """A feladatkiírás 1. ábrájának labirintusa (szakaszok cm-ben).
    A külső peremen két rés van: a Start alatt (x∈[-50,-30], y=-50) és a Cél
    fölött (x∈[30,50], y=50)."""
    segs_cm = [
        # külső perem (résekkel)
        (-50, -50, -50, 50), (-50, 50, 30, 50), (50, -50, 50, 50), (-30, -50, 50, -50),
        # belső falak
        (-30, -50, -30, -30), (-30, -30, -10, -30), (-10, -30, -10, 10),
        (-50, -10, -30, -10), (-10, -10, 10, -10), (30, -10, 50, -10),
        (10, -30, 10, -10), (30, -30, 30, -10),
        (-50, 10, -30, 10), (-30, 10, -30, 30), (-30, 30, -10, 30),
        (10, 10, 10, 50), (10, 10, 30, 10), (10, 30, 30, 30),
    ]
    return [(a / 100, b / 100, c / 100, d / 100) for a, b, c, d in segs_cm]


def random_maze(seed, loops=0, gaps=False):
    """Véletlen 5x5-ös labirintus (DFS), opcionálisan hurkokkal
    (néhány extra fal eltávolítva) és peremrésekkel."""
    rnd = random.Random(seed)
    n = mc.N_CELLS
    # walls[(i,j,'E')] , walls[(i,j,'N')]
    east = [[True] * n for _ in range(n)]
    north = [[True] * n for _ in range(n)]
    seen = [[False] * n for _ in range(n)]
    stack = [(0, 0)]
    seen[0][0] = True
    while stack:
        i, j = stack[-1]
        nb = [(i + di, j + dj) for di, dj in mc.DIRS
              if 0 <= i + di < n and 0 <= j + dj < n and not seen[i + di][j + dj]]
        if not nb:
            stack.pop()
            continue
        a, b = rnd.choice(nb)
        if a == i + 1:
            east[i][j] = False
        elif a == i - 1:
            east[a][b] = False
        elif b == j + 1:
            north[i][j] = False
        else:
            north[a][b] = False
        seen[a][b] = True
        stack.append((a, b))
    for _ in range(loops):
        i, j = rnd.randrange(n - 1), rnd.randrange(n - 1)
        if rnd.random() < 0.5:
            east[i][j] = False
        else:
            north[i][j] = False
    o, c = mc.ORIGIN, mc.CELL
    segs = []
    for i in range(n):
        for j in range(n):
            if i < n - 1 and east[i][j]:
                x = o + c * (i + 1)
                segs.append((x, o + c * j, x, o + c * (j + 1)))
            if j < n - 1 and north[i][j]:
                y = o + c * (j + 1)
                segs.append((o + c * i, y, o + c * (i + 1), y))
    if gaps:
        segs += [(-0.5, -0.5, -0.5, 0.5), (-0.5, 0.5, 0.3, 0.5), (0.5, -0.5, 0.5, 0.5), (-0.3, -0.5, 0.5, -0.5)]
    else:
        segs += [(-0.5, -0.5, -0.5, 0.5), (-0.5, 0.5, 0.5, 0.5), (0.5, -0.5, 0.5, 0.5), (-0.5, -0.5, 0.5, -0.5)]
    return segs


def segs_to_rects(segs, half=mc.WALL_HALF):
    """Falszakasz -> 1 cm vastag téglalap (sarkoknál átfedéssel)."""
    rects = []
    for x1, y1, x2, y2 in segs:
        rects.append((min(x1, x2) - half, min(y1, y2) - half, max(x1, x2) + half, max(y1, y2) + half))
    return rects


# geometria
def ray_rect(ox, oy, dx, dy, r):
    """Sugár–téglalap metszés (slab módszer). Visszaadja t-t vagy None-t."""
    tmin, tmax = -1e9, 1e9
    for o, d, lo, hi in ((ox, dx, r[0], r[2]), (oy, dy, r[1], r[3])):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
            if tmin > tmax:
                return None
    if tmax < 0:
        return None
    return max(tmin, 0.0)


def circle_rect_pen(x, y, rad, r):
    cx = min(max(x, r[0]), r[2])
    cy = min(max(y, r[1]), r[3])
    return rad - math.hypot(x - cx, y - cy)


# szimuláció
class Sim:
    def __init__(self, segs, seed=0, slip=0.03, radius_err=0.02, sensor_noise=0.001,
                 start=(-0.4, -0.4, math.pi / 2), dt=0.05, tau=0.06):
        self.rects = segs_to_rects(segs)
        self.rnd = random.Random(seed)
        self.x, self.y, self.th = start
        self.dt, self.tau = dt, tau
        self.slip, self.sensor_noise = slip, sensor_noise
        # valódi kerékátmérő eltér a névlegestől (szisztematikus odometria-hiba)
        self.rl = mc.RobotParams.WHEEL_RADIUS * (1 + self.rnd.uniform(-radius_err, radius_err))
        self.rr = mc.RobotParams.WHEEL_RADIUS * (1 + self.rnd.uniform(-radius_err, radius_err))
        self.base = mc.RobotParams.WHEEL_BASE * (1 + self.rnd.uniform(-0.03, 0.03))
        self.phi_l = self.phi_r = 0.0
        self.wl = self.wr = 0.0
        self.max_pen = 0.0
        self.collisions = 0
        self.traj = [(self.x, self.y)]

    def sensors(self):
        out = {}
        for name, (sx, sy, sa) in mc.RobotParams.SENSORS.items():
            ox = self.x + sx * math.cos(self.th) - sy * math.sin(self.th)
            oy = self.y + sx * math.sin(self.th) + sy * math.cos(self.th)
            a = self.th + sa
            dx, dy = math.cos(a), math.sin(a)
            best = None
            for r in self.rects:
                t = ray_rect(ox, oy, dx, dy, r)
                if t is not None and (best is None or t < best):
                    best = t
            if best is not None and best <= mc.RobotParams.SENSOR_RANGE:
                out[name] = best + self.rnd.gauss(0, self.sensor_noise)
            else:
                out[name] = None
        return out

    def step(self, wl_cmd, wr_cmd):
        a = self.dt / (self.tau + self.dt)
        self.wl += a * (wl_cmd - self.wl)
        self.wr += a * (wr_cmd - self.wr)
        # encoder
        self.phi_l = mc.wrap(self.phi_l + self.wl * self.dt)
        self.phi_r = mc.wrap(self.phi_r + self.wr * self.dt)
        # tényleges mozgás csúszással
        sl = 1 - abs(self.rnd.gauss(0, self.slip))
        sr = 1 - abs(self.rnd.gauss(0, self.slip))
        vl = self.wl * self.rl * sl
        vr = self.wr * self.rr * sr
        v = 0.5 * (vl + vr)
        w = (vr - vl) / self.base
        nx = self.x + v * math.cos(self.th + w * self.dt / 2) * self.dt
        ny = self.y + v * math.sin(self.th + w * self.dt / 2) * self.dt
        self.th = mc.wrap(self.th + w * self.dt)
        pen = max(circle_rect_pen(nx, ny, ROBOT_RADIUS, r) for r in self.rects)
        if pen > 0:
            self.collisions += 1
            self.max_pen = max(self.max_pen, pen)
        self.x, self.y = nx, ny
        self.traj.append((self.x, self.y))


def run(segs, seed=0, prior=None, max_time=240.0, quiet=True, **kw):
    sim = Sim(segs, seed=seed, **kw)
    logs = []
    ctrl = mc.MazeController(sim.x, sim.y, sim.th, prior_map_json=prior,
                             log=(logs.append if quiet else print))
    t = 0.0
    while t < max_time:
        wl, wr = ctrl.step(sim.dt, sim.phi_l, sim.phi_r, sim.sensors())
        sim.step(wl, wr)
        t += sim.dt
        if ctrl.done and abs(sim.wl) < 1e-3 and abs(sim.wr) < 1e-3:
            break
        if abs(sim.x) > 0.55 or abs(sim.y) > 0.55:
            break
    in_goal = sim.x > 0.30 and sim.y > 0.30
    return dict(sim=sim, ctrl=ctrl, t=t, ok=ctrl.done and in_goal, in_goal=in_goal,
                cells=ctrl.cells_moved, logs=logs,
                pose_err=math.hypot(ctrl.pose.x - sim.x, ctrl.pose.y - sim.y),
                head_err=math.degrees(abs(mc.wrap(ctrl.pose.th - sim.th))))


def shortest(segs):
    """Referencia: valódi legrövidebb út (cellalépés) a teljes ismert pályán."""
    m = mc.MazeMap()
    rects = segs_to_rects(segs)
    for i in range(mc.N_CELLS):
        for j in range(mc.N_CELLS):
            for d in range(4):
                cx, cy = mc.cell_center((i, j))
                dx, dy = mc.DIRS[d]
                best = None
                for r in rects:
                    t = ray_rect(cx, cy, dx, dy, r)
                    if t is not None and (best is None or t < best):
                        best = t
                m.set((i, j), d, mc.WALL if best is not None and best < 0.1 else mc.OPEN)
    return m.path_length(optimistic=False)


def plot(results, title, fname):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(results), figsize=(5 * len(results), 5))
    if len(results) == 1:
        axes = [axes]
    for ax, (label, segs, res) in zip(axes, results):
        for r in segs_to_rects(segs):
            ax.add_patch(plt.Rectangle((r[0], r[1]), r[2] - r[0], r[3] - r[1], color="#1f2f4f"))
        ax.add_patch(plt.Rectangle((-0.5, -0.5), 0.2, 0.2, color="#a6e3a1", alpha=0.6, zorder=0))
        ax.add_patch(plt.Rectangle((0.3, 0.3), 0.2, 0.2, color="#f5a3a3", alpha=0.6, zorder=0))
        xs, ys = zip(*res["sim"].traj)
        ax.plot(xs, ys, color="#d9480f", lw=1.6)
        ax.plot(xs[-1], ys[-1], "o", color="#d9480f")
        ax.set_xlim(-0.55, 0.55)
        ax.set_ylim(-0.55, 0.55)
        ax.set_aspect("equal")
        ax.set_title("%s\n%d cella, %.1f s, %s" % (label, res["cells"], res["t"],
                                                   "CÉLBA ÉRT" if res["ok"] else "SIKERTELEN"), fontsize=10)
        ax.grid(True, ls=":", lw=0.5)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(fname, dpi=130)


def main():
    plot_file = None
    if "--plot" in sys.argv:
        plot_file = sys.argv[sys.argv.index("--plot") + 1]
    print("=== 1) A kiírás ábrájának labirintusa ===")
    fig = maze_from_figure()
    r1 = run(fig, seed=1, quiet=False)
    print("1. menet: %s, %d cella, %.1f s, ütközés: %d, odometria-hiba a végén: %.1f mm / %.1f°"
          % ("OK" if r1["ok"] else "HIBA", r1["cells"], r1["t"], r1["sim"].collisions,
             r1["pose_err"] * 1000, r1["head_err"]))
    prior = r1["ctrl"].map.to_json()
    r2 = run(fig, seed=2, prior=prior, quiet=False)
    print("2. menet: %s, %d cella, %.1f s (legrövidebb lehetséges: %d)"
          % ("OK" if r2["ok"] else "HIBA", r2["cells"], r2["t"], shortest(fig)))

    print("\n=== 2) 200 véletlen labirintus, erős zajjal (8% csúszás, 5% kerékátmérő-hiba) ===")
    stats = dict(n=0, ok=0, coll=0, ok2=0, opt2=0)
    worst = 0.0
    for seed in range(200):
        segs = random_maze(seed, loops=seed % 4, gaps=(seed % 2 == 0))
        a = run(segs, seed=seed, slip=0.08, radius_err=0.05)
        b = run(segs, seed=seed + 1000, prior=a["ctrl"].map.to_json(), slip=0.08, radius_err=0.05)
        stats["n"] += 1
        stats["ok"] += a["ok"]
        stats["ok2"] += b["ok"]
        stats["coll"] += (a["sim"].collisions > 0) + (b["sim"].collisions > 0)
        stats["opt2"] += b["ok"] and b["cells"] == shortest(segs)
        worst = max(worst, a["pose_err"], b["pose_err"])
        if not (a["ok"] and b["ok"]) or a["sim"].collisions or b["sim"].collisions:
            print("  seed %d: 1.menet %s (%d ütk.), 2.menet %s (%d ütk.)" %
                  (seed, a["ok"], a["sim"].collisions, b["ok"], b["sim"].collisions))
    print("1. menet sikeres: %d/%d, 2. menet sikeres: %d/%d, 2. menet optimális: %d/%d,"
          " ütközéses futás: %d, legnagyobb odometria-hiba: %.1f mm"
          % (stats["ok"], stats["n"], stats["ok2"], stats["n"], stats["opt2"], stats["n"],
             stats["coll"], worst * 1000))
    if plot_file:
        plot([("1. menet (felfedezés)", fig, r1), ("2. menet (ismert térkép)", fig, r2)],
             "2D teszt – a kiírás labirintusa", plot_file)
        print("Ábra mentve:", plot_file)


if __name__ == "__main__":
    main()
