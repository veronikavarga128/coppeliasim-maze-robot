"""
CoppeliaSim illesztes a MazeBot vezerlohoz.
A robot scriptjeben fut: inditaskor megkeresi a motorokat es szenzorokat,
betolti az elozo menet terkepet, minden lepesben kiolvassa a szenzorokat,
a MazeController-tol kapott kerekszogsebessegeket kiadja a motoroknak,
a celban elmenti a terkepet. A falakat erzekelhetove teszi (detectable).
"""
ROBOT_PATH = "/MazeBot"
MAP_TAG = "MazeBot.mazeMap"   # terkep kulcsa a CoppeliaSim memoriajaban
USE_SAVED_MAP = True          # 2. menet: elozo terkep betoltese
RESET_MAP = False             # True: terkep torlese, uj felfedezes
MOTOR_TORQUE = 0.1            # max motor nyomatek [Nm]
DETECT_STEPS = 10             # ennyi lepesig keressuk a kesobb letrejott falakat

# handle-ok es a script allapota
robotHandle = None
leftMotor = None
rightMotor = None
sensors = {}                  # nev -> proximity szenzor handle
controller = None             # MazeController (allapotgep)
mapSaved = False
stepCount = 0
detectChecked = set()         # mar megvizsgalt shape-ek


# naplo a CoppeliaSim Statusbar ablakaba
def _log(msg):
    try:
        sim.addLog(sim.verbosity_scriptinfos, msg)
    except Exception:
        print(msg)


def _make_detectable():
    # a tanari palya falai nem "detectable"-ek, ezert a szenzorok nem latnak oket;
    # minden statikus shape-et (a robot sajat reszeit kiveve) detektalhatova teszek
    own = set(sim.getObjectsInTree(robotHandle, sim.handle_all, 0))
    n = 0
    for sh in sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0):
        if sh in own or sh in detectChecked:
            continue
        detectChecked.add(sh)
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


# szimulacio indulasa
def sysCall_init():
    global sim, robotHandle, leftMotor, rightMotor, sensors, controller
    global mapSaved, stepCount, detectChecked

    # 1. sim modul (beagyazott scriptben nem kell ZeroMQ kapcsolat)
    try:
        sim = require("sim")          # CoppeliaSim 4.6+
    except NameError:
        pass

    # 2. Objektumok handle-jeinek lekerese
    robotHandle = sim.getObject(ROBOT_PATH)
    leftMotor = sim.getObject(ROBOT_PATH + "/leftMotor")
    rightMotor = sim.getObject(ROBOT_PATH + "/rightMotor")
    sensors = {n: sim.getObject(ROBOT_PATH + "/prox_" + n) for n in RobotParams.SENSORS}
    for motor in (leftMotor, rightMotor):
        try:
            sim.setJointTargetForce(motor, MOTOR_TORQUE)
        except Exception:
            pass

    # kezdo pozicio es irany a ket kerek helyzetebol
    pl = sim.getObjectPosition(leftMotor, sim.handle_world)
    pr = sim.getObjectPosition(rightMotor, sim.handle_world)
    x0, y0 = 0.5 * (pl[0] + pr[0]), 0.5 * (pl[1] + pr[1])
    yaw0 = math.atan2(pl[1] - pr[1], pl[0] - pr[0]) - math.pi / 2

    # terkep: torles (RESET_MAP) vagy az elozo menet betoltese
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

    mapSaved = False
    detectChecked = set()
    stepCount = 0
    _make_detectable()

    # 3-4. Parameterek es allapotgep inicializalasa (kezdo allapot: SETTLE)
    controller = MazeController(x0, y0, yaw0, prior_map_json=prior, log=_log)
    _log("[MazeBot] Indulas: x=%.3f y=%.3f irany=%.1f fok, mod: %s"
         % (x0, y0, math.degrees(yaw0), "2. menet (ismert terkep)" if prior else "felfedezes"))


# szenzortavolsagok [m]; None, ha nincs fal a hatotavon belul
def _read_sensors():
    out = {}
    for name, sensor in sensors.items():
        r = sim.readProximitySensor(sensor)
        result, distance = (r[0], r[1]) if r else (0, None)
        out[name] = distance if result > 0 else None
    return out


# 5. Fo vezerlesi hurok: minden szimulacios lepesben lefut
def sysCall_actuation():
    global stepCount
    dt = sim.getSimulationTimeStep()
    stepCount += 1
    if stepCount <= DETECT_STEPS:
        _make_detectable()   # kesoi falak

    # szenzorok -> allapotgep -> inverz kinematika (a MazeController-ben)
    w_left, w_right = controller.step(dt, sim.getJointPosition(leftMotor),
                                      sim.getJointPosition(rightMotor), _read_sensors())

    # Parancsok kikuldese
    sim.setJointTargetVelocity(leftMotor, w_left)
    sim.setJointTargetVelocity(rightMotor, w_right)

    if controller.done and not mapSaved:
        _save_map()


# terkep mentese a kovetkezo (legrovidebb utas) menethez
def _save_map():
    global mapSaved
    try:
        sim.persistentDataWrite(MAP_TAG, controller.map.to_json())
        _log("[MazeBot] Terkep elmentve, a kovetkezo inditas a legrovidebb uton megy.")
    except Exception as e:
        _log("[MazeBot] Terkep mentese sikertelen: %s" % e)
    mapSaved = True


# leallitaskor: motorok lefekezese
# ha a cel elott allitottak le, a reszleges terkepet is mentjuk
def sysCall_cleanup():
    for motor in (leftMotor, rightMotor):
        if motor is not None:
            try:
                sim.setJointTargetVelocity(motor, 0.0)
            except Exception:
                pass
    if controller is not None and not mapSaved and controller.cells_moved > 0:
        _save_map()
