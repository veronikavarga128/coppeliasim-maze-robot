ROBOT_PATH = "/MazeBot"
MAP_TAG = "MazeBot.mazeMap"
USE_SAVED_MAP = True
RESET_MAP = False
MOTOR_TORQUE = 0.1
DETECT_STEPS = 10             # fal flag lepesek

_s = {}


def _log(msg):
    try:
        sim.addLog(sim.verbosity_scriptinfos, msg)
    except Exception:
        print(msg)


def _make_detectable():
    # falak detektalhatok
    own = set(sim.getObjectsInTree(_s["h"]["robot"], sim.handle_all, 0))
    n = 0
    for sh in sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0):
        if sh in own or sh in _s["detect"]:
            continue
        _s["detect"].add(sh)
        try:
            if sim.getObjectInt32Param(sh, sim.shapeintparam_static) != 1:
                continue
            p = sim.getObjectSpecialProperty(sh)
            if not p & sim.objectspecialproperty_detectable:
                sim.setObjectSpecialProperty(sh, p | sim.objectspecialproperty_detectable)
                n += 1
        except Exception:
            pass
    if n:
        _log("[MazeBot] %d statikus shape detektalhato lett." % n)


def sysCall_init():
    global sim
    try:
        sim = require("sim")          # CoppeliaSim 4.6+
    except NameError:
        pass
    h = {}
    h["robot"] = sim.getObject(ROBOT_PATH)
    h["L"] = sim.getObject(ROBOT_PATH + "/leftMotor")
    h["R"] = sim.getObject(ROBOT_PATH + "/rightMotor")
    h["sens"] = {n: sim.getObject(ROBOT_PATH + "/prox_" + n) for n in RobotParams.SENSORS}
    for j in (h["L"], h["R"]):
        try:
            sim.setJointTargetForce(j, MOTOR_TORQUE)
        except Exception:
            pass

    pl = sim.getObjectPosition(h["L"], sim.handle_world)
    pr = sim.getObjectPosition(h["R"], sim.handle_world)
    x0, y0 = 0.5 * (pl[0] + pr[0]), 0.5 * (pl[1] + pr[1])
    yaw0 = math.atan2(pl[1] - pr[1], pl[0] - pr[0]) - math.pi / 2
    prior = None
    if RESET_MAP:
        try:
            sim.persistentDataWrite(MAP_TAG, "")
        except Exception:
            pass
    elif USE_SAVED_MAP:
        try:
            prior = sim.persistentDataRead(MAP_TAG)
            if isinstance(prior, (bytes, bytearray)):
                prior = prior.decode("utf-8")
            if not prior:
                prior = None
        except Exception:
            prior = None
    _s["h"] = h
    _s["saved"] = False
    _s["detect"] = set()
    _s["n"] = 0
    _make_detectable()
    _s["ctrl"] = MazeController(x0, y0, yaw0, prior_map_json=prior, log=_log)
    _log("[MazeBot] Indulas: x=%.3f y=%.3f irany=%.1f fok, mod: %s"
         % (x0, y0, math.degrees(yaw0), "2. menet (ismert terkep)" if prior else "felfedezes"))


def _read_dists():
    out = {}
    for n, hs in _s["h"]["sens"].items():
        r = sim.readProximitySensor(hs)
        out[n] = r[1] if (r and r[0] > 0) else None
    return out


def sysCall_actuation():
    h = _s["h"]
    ctrl = _s["ctrl"]
    dt = sim.getSimulationTimeStep()
    _s["n"] += 1
    if _s["n"] <= DETECT_STEPS:
        _make_detectable()   # kesoi falak
    wl, wr = ctrl.step(dt, sim.getJointPosition(h["L"]), sim.getJointPosition(h["R"]), _read_dists())
    sim.setJointTargetVelocity(h["L"], wl)
    sim.setJointTargetVelocity(h["R"], wr)
    if ctrl.done and not _s["saved"]:
        _save_map()


def _save_map():
    try:
        sim.persistentDataWrite(MAP_TAG, _s["ctrl"].map.to_json())
        _log("[MazeBot] Terkep elmentve, a kovetkezo inditas a legrovidebb uton megy.")
    except Exception as e:
        _log("[MazeBot] Terkep mentese sikertelen: %s" % e)
    _s["saved"] = True


def sysCall_cleanup():
    if "ctrl" in _s and not _s["saved"] and _s["ctrl"].cells_moved > 0:
        _save_map()
