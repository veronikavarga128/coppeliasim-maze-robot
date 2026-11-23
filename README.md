# MazeBot – autonóm labirintus-megoldó robot (CoppeliaSim)

## Fájlok

| Fájl | Tartalom |
|---|---|
| `maze_controller.py` | A navigáció teljes logikája (térkép, Flood-Fill, odometria, falalapú korrekciók, állapotgép). Nem függ a CoppeliaSimtől. |
| `coppelia_adapter.py` | Illesztés a CoppeliaSimhez (`sysCall_init`, `sysCall_actuation`, `sysCall_cleanup`). |
| `build_robot.py` | Építőscript (ZeroMQ Remote API): felépíti a robotot a megnyitott jelenetbe, és beágyazza a vezérlőt. |
| `robot_controller_embedded.py` | Az előző kettő összefűzve. Ezt a scriptet kapja meg a robot; kézi bemásoláshoz is használható. |
| `sim2d.py` | 2D-s tesztszimulátor zajjal és csúszással (a kiírás labirintusa + 200 véletlen labirintus). |
| `Labirintus_MazeBot.ttt` | A leadott jelenet: a kapott labirintus és a robot a beágyazott vezérlővel. |

## Előfeltételek

- **Ajánlott: a legfrissebb CoppeliaSim Edu (4.10).** A kapott jelenetet is ez a verzió mentette. A beágyazott Python scripthez legalább 4.6 kell, a régebbi verziók nem támogatottak.
- A Python scriptekhez a CoppeliaSim-nek egy Python 3 telepítés kell, benne a `pyzmq` és a `cbor2` csomaggal:
  `pip install pyzmq cbor2`. Ha a CoppeliaSim nem találja a Pythont, a `usrset.txt` fájlban add meg a `defaultPython` értékét.
- Az építőscripthez: `pip install coppeliasim-zmqremoteapi-client`

## A robot felépítése (egyszer kell lefuttatni)

1. Nyisd meg a CoppeliaSimben a `Labirintus_1x1m.ttt` jelenetet. A szimuláció legyen leállítva.
2. Futtasd ebben a mappában: `python build_robot.py`
   - A script a Start cellába (−0,40; −0,40) felépíti a `MazeBot` modellt, +Y irányba (észak felé) fordítva.
   - Utána 0,3 másodpercre elindítja a szimulációt, és kiírja a szenzorok méréseit. Ahol fal van, ott 0,05–0,07 m körüli értéket kell látnod.
3. Mentsd el a jelenetet: **File › Save scene as… › `Labirintus_MazeBot.ttt`**.

Ha a kezdőirány nem jó a labirintushoz (például a Start cellából csak kelet felé lehet kijutni), az semmit nem ront: a robot a kezdőirányt az odometriához használja, és bármelyik irányba elfordul.

**A falak érzékelhetősége:** a kapott jelenet scriptje a szimuláció indulásakor hozza létre a falakat, de *detectable* jelző nélkül, így a távolságszenzorok alapból nem látnák őket. Ezt a robot vezérlője maga oldja meg: indításkor és az első 10 szimulációs lépésben a jelenet összes statikus shape-jét (a robot saját részei kivételével) érzékelhetővé teszi. A jelenetet és a labirintus scriptjét ezért nem kell módosítani, és külön falépítő script (`build_maze.py`) sem kell. A Statusbarban ilyenkor ez a sor jelenik meg: `[MazeBot] N statikus shape detektalhato lett.`

## Futtatás

- **1. menet (felfedezés):** nyomd meg a Start gombot. A robot cellánként halad, közben feltérképezi a falakat, a célcellában megáll, és elmenti a térképet. A *Statusbar* ablakban követhető a napló, a végén a labirintus szöveges térképével.
- **2. menet (legrövidebb út, bónusz):** állítsd le, majd indítsd újra a szimulációt. A robot betölti az előző térképet, és a legrövidebb úton, gyorsabban megy végig. A napló kiírja, hogy az út bizonyítottan optimális-e.
- **Új labirintus vagy új felfedezés:** a beágyazott script elején állítsd be: `RESET_MAP = True`. Ha a labirintus megváltozott, ez nem kötelező: a robot az ellentmondó falakat menet közben javítja, és szükség esetén maga törli a térképet.

A térkép a CoppeliaSim memóriájában marad meg (`sim.persistentDataWrite`), a program bezárásakor törlődik.

## Hangolás, ha a valódi szimulációban valami nem stimmel

Minden állítható paraméter a `MazeController` osztály elején van:

| Tünet | Paraméter |
|---|---|
| Túl lassú vagy túl gyors haladás | `V_EXPLORE`, `V_FAST`, `ACC` |
| Kígyózik a folyosóban | csökkentsd a `K_LAT`, `K_HEAD` értékét |
| Megcsúsznak a kerekek induláskor | csökkentsd az `ACC`, `W_ACC` értékét, vagy növeld a kerék súrlódását a `build_robot.py`-ban |
| Rosszul ismeri fel a falakat | `WALL_TH` (a cellaközéptől mért faltávolság kb. 0,055 m) |
| Pontatlan 90°-os fordulás | `W_MAX`, illetve a `RobotParams.wheelBase` (ez a kerekek tényleges távolsága) |

Ha módosítasz valamit, futtasd újra a `build_robot.py`-t, vagy másold be a `robot_controller_embedded.py` új tartalmát a robot scriptjébe (a `python build_robot.py --print-script` paranccsal újragenerálható).

## Kapcsolat az órai anyagokkal

- **Inverz kinematika** (03 Modell, DDR inverz kinematikai modell; `CoppeliasimRobot.m`): `MazeController._inverse_kinematics()`
  `w_left = (v - (w * wheelBase / 2.0)) / wheelRadius`, `w_right = (v + (w * wheelBase / 2.0)) / wheelRadius`.
  Az órai jelenetben a jobb motor tükrözött, ezért ott `-1.0`-s szorzó kell a `w_right`-ra; a MazeBot két csuklója egy irányba néz, így itt nem kell.
- **Odometria / dead reckoning** (03 Modell, Útmérés): `Pose.update()`, `d = R·Δφ` (az órai `d = Δs/q · c`, `c = 2πR` alakja radiánban mért kerékszögre), `ds = (d_r + d_l)/2`, `dθ = (d_r − d_l)/L`.
- **Véges állapotú automata** (03 Modell, FSM; `MobilRobot_StateMachine`): `MazeController.step()`, szöveges állapotnevekkel, mint az órai `FORWARD`/`AVOID`. Az állapotok és átmenetek táblázata a `maze_controller.py` elején van.
- **Észlelés – Tervezés – Végrehajtás** (01–02): szenzorok és odometria → Flood-Fill útkeresés → állapotgép és motorparancsok.
- **ZeroMQ Remote API** (`MobilRobot_ZeroMQ`, `CoppeliasimRobot.m`): a `build_robot.py` ugyanígy csatlakozik (`RemoteAPIClient()`, `client.getObject('sim')`). A vezérlés maga a robotba ágyazott scriptben fut, mert a leadandó `.ttt`-nek tartalmaznia kell a vezérlést.

## 2D teszt

```
python sim2d.py --plot teszt_eredmeny.png
```
