import matplotlib.pyplot as plt     # kirajzolásra- szimuláció           --> cmd:pip install matplotlib numpy
import math                         # matematikai műveletekre pl.:pi

import numpy as np      #gyorsabb számolás miatt --> ez C-ben írodott 


class robotconfig:
    def __init__(robot):     #konstruktor
        robot.v_max = 2 # [m/s]
        robot.v_min = -1 # [m/s]
        robot.w_max = 2  # [rad/s]
        robot.w_min = -1 # [rad/s]
        robot.a_max = 4 # [m/s^2] gyorsulás
        
        # Felbontások a mintavételezéshez
        robot.resolution = 0.1 #A lineáris sebesség  mintavételezési finomsága m/s. --> Szerepe: A DWA nem egy végtelen tartományban keres, hanem „lépked”. Ez a szám adja meg, mekkora közökkel tesztelje a lehetséges sebességeket.Példa: Ha a min sebesség 0, a max 1, és a felbontás 0.1, akkor a kód megnézi a 0.0, 0.1, 0.2 ... 1.0 értékeket.Hatása: Minél kisebb (finomabb), annál pontosabb lesz a mozgás, de annál több számítást kell végeznie a processzornak (lassabb lesz a kód).
        robot.turn_resolution = 5 * math.pi / 180.0  # [rad/s] -- kanyarodási felbontás --> hasonló szerepe mint a fenti sebesség felbontásnak
        
        #DWA időparaméterek
        robot.dt = 0.1 # [s] --> időléspés (felbontás)
        robot.predict_time = 3 # [s] --> mennyi időre lásson előre
        
    
        #G(v,w)= Alfa*heading(v,w)+Beta*dist(v,w)+Gamma*vel(v,w)
        robot.Alfa = 2  #célra tarás súlya
        robot.Beta = 3 # Akadály kerülés súlya
        robot.Gamma = 1 #Sebesség súlya
    
        #robot-alakja--> legyen kör az egyszerűség kedvéért
        robot.rob_radius = 0.1 # [m]
        
class robotstate:
    def __init__(robot, x=0, y=0, v=0, w=0, irany = 0):  #default konstruktor
        robot.x = x
        robot.y = y
        robot.v = v
        robot.w = w
        robot.irany = irany
        
def update(robot, u, dt):
    
        #kezdeti paraméterek
        v = u[0]
        w = u[1]
        theta_0 = robot.irany
        
        #1.eset --> ha kanyarodik w!=0
        robot.x = (v/w) * (math.sin(theta_0) - math.sin(theta_0 + w(robot.))
        
        
        # Visszaadjuk a teljes állapotvektort, ha szükséges
        return np.array([robot.x, robot.y, robot.irany, robot.v, robot.w])

def pairstochoose (robotconfig, robotstate):
    """        
    Összes választható v,w pár kiszámolása
    """
    # 1. Dinamikus ablak meghatározása (sebesség és gyorsulás korlátok)
    v_min_actual = max(robotconfig.v_min, robotstate.v - robotconfig.a_max * robotconfig.dt)
    v_max_actual = min(robotconfig.v_max, robotstate.v + robotconfig.a_max * robotconfig.dt)
        
    w_min_actual = max(robotconfig.w_min, robotstate.w - robotconfig.a_max * robotconfig.dt)
    w_max_actual = min(robotconfig.w_max, robotstate.w + robotconfig.a_max * robotconfig.dt)

    # 2. Listák létrehozása a felbontások alapján (np.arange-dzsel)
    # v_resolution: m/s lépésköz, turn_resolution: rad/s lépésköz
    v_samples = np.arange(v_min_actual, v_max_actual, robotconfig.resolution)
    w_samples = np.arange(w_min_actual, w_max_actual, robotconfig.turn_resolution)

    # 3. Összes párosítás előállítása
    all_pairs = []
    for v in v_samples:
            for w in w_samples:
                all_pairs.append([v, w])
    return all_pairs
    
################################################
    """szimulációs rész --> kirajzolás"""
################################################

def plot_all_trajectories(state, config, all_pairs):
    plt.figure(figsize=(10, 10))
    plt.grid(True)
    plt.axis("equal")
    
    # 1. Összes lehetséges pálya kirajzolása
    for v, w in all_pairs:
        # Létrehozunk egy ideiglenes állapotot (szellemet), 
        # hogy ne az eredeti robotunkat mozgassuk el a rajzolás közben
        ghost_robot = robotstate(x=state.x, y=state.y, v=state.v, w=state.w, irany=state.irany)
        
        path_x, path_y = [ghost_robot.x], [ghost_robot.y]
        
        # A jóslási időn belül lépegetünk
        for _ in np.arange(0, config.predict_time, config.dt):              #egy listát (számsort) hoz létre. Ha a predict_time = 3.0 és a dt = 0.1, akkor ez a sor ilyen számokon megy végig: 0.0, 0.1, 0.2, 0.3 ... 2.9.
            # Itt hívjuk meg a már létező UPDATE fgv-t
            # u = [v, w] páros, dt = időlépés
            update(ghost_robot, [v, w], config.dt)
            
            path_x.append(ghost_robot.x)
            path_y.append(ghost_robot.y)
            
        plt.plot(path_x, path_y, "-g", alpha=0.2) 

    # 2. A robot és az irány kirajzolása
    robot_circle = plt.Circle((state.x, state.y), config.rob_radius, color="b", fill=False)
    plt.gca().add_artist(robot_circle)
    plt.arrow(state.x, state.y, config.rob_radius * math.cos(state.irany), 
              config.rob_radius * math.sin(state.irany), head_width=0.1)
    
    plt.show()

# Futtatás
conf = robotconfig()
# Példa: a robot már 0.5 m/s-mal megy és kicsit kanyarodik
curr_state = robotstate(x=0, y=0, v=0.5, w=0.1, irany=0)
pairs = pairstochoose(conf, curr_state)
plot_all_trajectories(curr_state, conf, pairs)