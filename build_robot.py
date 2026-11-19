import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from maze_controller import RobotParams  # noqa: E402

# parameterek
START = (-0.40, -0.40)          # Start cella
START_YAW = math.pi / 2
CHASSIS_D, CHASSIS_H, CLEARANCE = 0.080, 0.035, 0.010
CASTER_R = 0.008
CASTER_X = 0.030
WHEEL_R = RobotParams.WHEEL_RADIUS      # 0.022
WHEEL_W = 0.010
WHEEL_Y = RobotParams.WHEEL_BASE / 2    # 0.045
SENSOR_Z = 0.050
SENSOR_RANGE = RobotParams.SENSOR_RANGE
CHASSIS_MASS, WHEEL_MASS = 0.25, 0.03
WHEEL_FRICTION, BODY_FRICTION = 2.0, 0.0
HANDLE_WORLD = -1


def build_script_text():
    with open(os.path.join(HERE, "maze_controller.py"), encoding="utf-8") as f:
        ctrl = f.read()
    with open(os.path.join(HERE, "coppelia_adapter.py"), encoding="utf-8") as f:
        adapter = f.read()
    return "#python\n\n" + ctrl + "\n\n" + adapter


# geometria
def local_to_world(lx, ly, lz):
    c, s = math.cos(START_YAW), math.sin(START_YAW)
    return (START[0] + c * lx - s * ly, START[1] + s * lx + c * ly, lz)


def matrix(xaxis, yaxis, zaxis, pos):
    return [xaxis[0], yaxis[0], zaxis[0], pos[0],
            xaxis[1], yaxis[1], zaxis[1], pos[1],
            xaxis[2], yaxis[2], zaxis[2], pos[2]]


def axis_matrix(z_angle, pos):
    c, s = math.cos(z_angle), math.sin(z_angle)
    return matrix((0, 0, 1), (s, -c, 0), (c, s, 0), pos)


class Builder:
    def __init__(self, sim):
        self.sim = sim

    # helpers
    def set_matrix(self, h, m):
        try:
            self.sim.setObjectMatrix(h, m, HANDLE_WORLD)      # uj api
        except Exception:
            self.sim.setObjectMatrix(h, HANDLE_WORLD, m)      # regi api

    def set_position(self, h, p):
        try:
            self.sim.setObjectPosition(h, list(p), HANDLE_WORLD)
        except Exception:
            self.sim.setObjectPosition(h, HANDLE_WORLD, list(p))

    def try_(self, label, fn, *a):
        try:
            return fn(*a)
        except Exception as e:
            print("  (figyelmeztetes) %s: %s" % (label, e))
            return None

    def dynamic(self, h, mass, friction, density=1000.0):
        sim = self.sim
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 0)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 1)
        self.try_("utkozesi maszk", lambda: sim.setObjectInt32Param(h, sim.shapeintparam_respondable_mask, 0xFF00))
        self.try_("computeMassAndInertia", sim.computeMassAndInertia, h, density)
        self.try_("setShapeMass", sim.setShapeMass, h, mass)
        # surlodas
        for name in ("bullet_body_friction", "ode_body_friction",
                     "newton_body_staticfriction", "newton_body_kineticfriction",
                     "mujoco_body_friction1"):
            pid = getattr(sim, name, None)
            if pid is not None:
                try:
                    sim.setEngineFloatParam(pid, h, friction)
                except Exception:
                    pass

    def cylinder(self, d, hgt):
        sim = self.sim
        return sim.createPrimitiveShape(sim.primitiveshape_cylinder, [d, d, hgt], 0)

    def sphere(self, d):
        sim = self.sim
        return sim.createPrimitiveShape(sim.primitiveshape_spheroid, [d, d, d], 0)

    # robot
    def build(self):
        sim = self.sim
        try:
            old = sim.getObject("/MazeBot")
            print("Regi MazeBot eltavolitasa")
            sim.removeModel(old)
        except Exception:
            pass

        chassis = self.cylinder(CHASSIS_D, CHASSIS_H)
        self.set_position(chassis, local_to_world(0, 0, CLEARANCE + CHASSIS_H / 2))
        casters = []
        for cx in (CASTER_X, -CASTER_X):
            b = self.sphere(2 * CASTER_R)
            self.set_position(b, local_to_world(cx, 0, CASTER_R + 0.0005))
            casters.append(b)
        body = sim.groupShapes([chassis] + casters, False)
        sim.setObjectAlias(body, "MazeBot")
        self.dynamic(body, CHASSIS_MASS, BODY_FRICTION)
        self.try_("modell tulajdonsag", sim.setModelProperty, body, 0)
        self.try_("szin", lambda: sim.setShapeColor(body, None, sim.colorcomponent_ambient_diffuse, [0.15, 0.35, 0.75]))

        for side, sy in (("left", WHEEL_Y), ("right", -WHEEL_Y)):
            j = sim.createJoint(sim.joint_revolute_subtype, sim.jointmode_dynamic, 0, [0.03, 0.006])
            sim.setObjectAlias(j, side + "Motor")
            p = local_to_world(0, sy, WHEEL_R)
            yl = START_YAW + math.pi / 2
            self.set_matrix(j, matrix((math.cos(START_YAW), math.sin(START_YAW), 0), (0, 0, -1),
                                      (math.cos(yl), math.sin(yl), 0), p))

            self.try_("motor engedelyezes",
                      lambda: sim.setObjectInt32Param(j, sim.jointintparam_motor_enabled, 1))
            self.try_("sebessegszabalyozott motor",
                      lambda: sim.setObjectInt32Param(j, sim.jointintparam_dynctrlmode, sim.jointdynctrl_velocity))
            self.try_("nyomatek", lambda: sim.setJointTargetForce(j, 0.1))
            sim.setObjectParent(j, body, True)
            w = self.cylinder(2 * WHEEL_R, WHEEL_W)
            sim.setObjectAlias(w, side + "Wheel")
            self.set_matrix(w, matrix((math.cos(START_YAW), math.sin(START_YAW), 0), (0, 0, -1),
                                      (math.cos(yl), math.sin(yl), 0), p))
            self.dynamic(w, WHEEL_MASS, WHEEL_FRICTION, density=2000.0)
            self.try_("szin", lambda: sim.setShapeColor(w, None, sim.colorcomponent_ambient_diffuse, [0.1, 0.1, 0.1]))
            sim.setObjectParent(w, j, True)

        for name, (sx, sy, sa) in RobotParams.SENSORS.items():
            h = sim.createProximitySensor(
                sim.proximitysensor_ray_subtype, 16, 0,
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0.0, SENSOR_RANGE, 0.01, 0.01, 0.01, 0.01, 0.0, 0.01, 0.01, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0])
            sim.setObjectAlias(h, "prox_" + name)
            self.set_matrix(h, axis_matrix(START_YAW + sa, local_to_world(sx, sy, SENSOR_Z)))
            sim.setObjectParent(h, body, True)

        # vezerlo script
        code = build_script_text()
        script = None
        try:
            script = sim.createScript(sim.scripttype_simulation, code, 0, "python")   # 4.6+
            sim.setObjectParent(script, body, True)
            sim.setObjectAlias(script, "MazeBotController")
        except Exception as e:
            print("  createScript nem elerheto (%s), regi API-val probalom" % e)
            script = sim.addScript(sim.scripttype_childscript)
            sim.setScriptText(script, code)
            sim.associateScriptWithObject(script, body)
        print("MazeBot kesz: alvaz handle=%s, script handle=%s" % (body, script))
        return body


def self_test(sim, seconds=0.3):
    print("Onteszt: %.1f s szimulacio" % seconds)
    sim.setStepping(True)
    sim.startSimulation()
    t0 = sim.getSimulationTime()
    while sim.getSimulationTime() - t0 < seconds:
        sim.step()
    for name in RobotParams.SENSORS:
        h = sim.getObject("/MazeBot/prox_" + name)
        r = sim.readProximitySensor(h)
        print("  prox_%-2s : %s" % (name, ("%.3f m" % r[1]) if r[0] > 0 else "nincs fal (> %.2f m)" % SENSOR_RANGE))
    p = sim.getObjectPosition(sim.getObject("/MazeBot"), HANDLE_WORLD)
    print("  robot helyzete: x=%.3f y=%.3f z=%.3f" % tuple(p[:3]))
    sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.05)
    sim.setStepping(False)
    print("Falnal 0.05-0.07 m az elvart ertek.")


def main():
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    except ImportError:
        print("Hianyzik: pip install coppeliasim-zmqremoteapi-client")
        sys.exit(1)
    client = RemoteAPIClient()
    sim = client.require("sim") if hasattr(client, "require") else client.getObject("sim")
    if sim.getSimulationState() != sim.simulation_stopped:
        print("Elobb allitsd le a szimulaciot!")
        sys.exit(1)
    Builder(sim).build()
    if "--no-test" not in sys.argv:
        self_test(sim)
    print("Most mentsd el a jelenetet: File > Save scene as")


if __name__ == "__main__":
    if "--print-script" in sys.argv:
        print(build_script_text())
    else:
        main()
