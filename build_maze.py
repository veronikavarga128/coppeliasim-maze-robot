# Labirintus falainak letrehozasa a megnyitott jelenetbe (ZeroMQ Remote API).
# Futtatas: a CoppeliaSim nyitva, szimulacio leallitva -> python build_maze.py
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sim = RemoteAPIClient().require('sim')
if sim.getSimulationState() != sim.simulation_stopped:
    raise SystemExit('Allitsd le a szimulaciot, majd futtasd ujra.')
try:
    sim.getObject('/Labirintus_1x1m')
    raise SystemExit('A labirintus mar letezik a jelenetben.')
except Exception as e:
    if isinstance(e, SystemExit):
        raise

cell, h, t, half = 0.20, 0.10, 0.01, 0.5
parent = sim.createDummy(0.05)
sim.setObjectAlias(parent, 'Labirintus_1x1m')

def wall(name, sx, sy, x, y):
    s = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [sx, sy, h], 0)
    sim.setObjectAlias(s, name)
    sim.setObjectPosition(s, [x, y, h / 2])
    sim.setObjectParent(s, parent, True)
    sim.setObjectInt32Param(s, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(s, sim.shapeintparam_respondable, 1)
    sim.setObjectSpecialProperty(s, sim.objectspecialproperty_collidable
                                 | sim.objectspecialproperty_measurable
                                 | sim.objectspecialproperty_detectable
                                 | sim.objectspecialproperty_renderable)
    sim.setShapeColor(s, None, sim.colorcomponent_ambient_diffuse, [0.75, 0.78, 0.82])

def zone(name, x, y, color):
    s = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell * 0.7, cell * 0.7, 0.001], 0)
    sim.setObjectAlias(s, name)
    sim.setObjectPosition(s, [x, y, 0.0005])
    sim.setObjectParent(s, parent, True)
    sim.setObjectInt32Param(s, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(s, sim.shapeintparam_respondable, 0)
    sim.setObjectSpecialProperty(s, sim.objectspecialproperty_renderable)
    sim.setShapeColor(s, None, sim.colorcomponent_ambient_diffuse, color)

H = [[0,1,1,1,1],[0,1,0,0,0],[1,0,1,0,1],[1,0,0,1,0],[0,1,0,1,0],[1,1,1,1,0]]
V = [[1,1,0,0,0,1],[1,0,1,1,1,1],[1,0,1,0,0,1],[1,1,0,1,0,1],[1,0,0,1,0,1]]
n = 0
for r in range(6):
    for c in range(5):
        if H[r][c]:
            n += 1
            wall(f'Fal_H_{n}', cell + t, t, -half + (c + 0.5) * cell, -half + r * cell)
for r in range(5):
    for c in range(6):
        if V[r][c]:
            n += 1
            wall(f'Fal_V_{n}', t, cell + t, -half + c * cell, -half + (r + 0.5) * cell)
zone('Start_Zona', -half + cell / 2, -half + cell / 2, [0.2, 0.8, 0.3])
zone('Cel_Zona', half - cell / 2, half - cell / 2, [0.9, 0.25, 0.2])
print(f'Labirintus kesz ({n} falelem).')
