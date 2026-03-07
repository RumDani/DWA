import matplotlib.pyplot as plt     # kirajzolásra- szimuláció           --> cmd:pip install matplotlib numpy
import math                         # matematikai műveletekre pl.:pi

import numpy as np      #gyorsabb számolás miatt --> ez C-ben írodott

"""
get_robot_kinematics : a kerékadatokból v,w párokat számol --> differenciális meghajtású robot elmélete
"""
def get_robot_kinematics(w_L, w_R, config):
    v = ((w_L + w_R) / 2) * config.r_kerek
    w = ((w_R - w_L) / (2 * config.b)) * config.r_kerek
    return v, w

"""
Akadály osztály
"""
class Obstacle:
    def __init__(self, x, y, radius=0.1):
        self.x = x
        self.y = y
        self.radius = radius


"""        
Akadályok példányosítása
"""
obstacle_list = [
    Obstacle(0.6, 0.25, radius=0.15),
    Obstacle(1.5, 0.5, radius=0.15),  
    Obstacle(2.0, -0.2, radius=0.08),
    Obstacle(0.8, 1.2, radius=0.2),
    Obstacle(2.5, 1.0, radius=0.1)
]

"""
Kiszámítja a megtett utat az ív mentén az első ütközésig.
Ha nincs ütközés a predict_time alatt, egy nagy értéket ad vissza.
"""
def get_dist_on_trajectory(state, config, w_L, w_R, obstacle_list):
    temp_robot = robotstate(x=state.x, y=state.y, irany=state.irany, w_L=state.w_L, w_R=state.w_R)
    accumulated_dist = 0.0
    v_actual, _ = get_robot_kinematics(w_L, w_R, config)
    step_dist = abs(v_actual) * config.dt 
    
    for _ in np.arange(0, config.predict_time, config.dt):
        for obs in obstacle_list:
            dist_centers = math.hypot(temp_robot.x - obs.x, temp_robot.y - obs.y)
            if dist_centers <= (config.rob_radius + obs.radius):
                return accumulated_dist # Azonnali stop
        
        update(temp_robot, config, w_L, w_R, config.dt)
        accumulated_dist += step_dist
                
    return 10.0


#############xx FÉKEZÉSI FELTÉTELEK ###############
def AdmissableVelocity(robotconfig, w_L, w_R, state, obstacle_list):
    """
    Ellenőrzi, hogy a robot meg tud-e állni az adott íven az ütközés előtt.
    """
    # 1. Kiszámítjuk a távolságot az ív mentén az ütközésig
    dist_to_obs = get_dist_on_trajectory(state, robotconfig, w_L, w_R, obstacle_list)
    
    # 2. Megkapjuk a lineáris sebességet (v)
    v, _ = get_robot_kinematics(w_L, w_R, robotconfig)
    
    #Ez a képlet le van vezetve --> admissable velocity lap
    if abs(v) <= math.sqrt(2 * robotconfig.a_max * dist_to_obs):
        return True
    return False
   
############################################################
"""#Differenciális meghajtású robot esetén"""
class robotconfig:
    def __init__(robot):
        # feltételezem hogy a negativ és poziti sebességek megegyeznek v_max = v_min és w_max=w_min --> ezeket kiszámolom a kerekek adataibol és a robot konfigurációjából/ felépítéséből
        robot.r_kerek = 0.033   #[m]
        robot.b = 0.08 #[m]
        robot.w_kerek_max = 15 #rad/s
        robot.a_max = 2 # [m/s^2] gyorsulás
       
        robot.w_kerek_min = -robot.w_kerek_max
       
        # Felbontások a mintavételezéshez (most a kerék szögsebességére vonatkozóan)
        robot.w_kerek_resolution = 1.5 # [rad/s] A kerék szögsebesség mintavételezési finomsága.
       
        #DWA időparaméterek
        robot.dt = 0.1 # [s] --> időléspés (felbontás)
        robot.predict_time = 3 # [s] --> mennyi időre lásson előre
   
        #robot-alakja--> legyen kör az egyszerűség kedvéért
        robot.rob_radius = robot.b + 0.02 #[m]
       
       
        ########Optimalizáció###########
       
        #G(v,w)= Alfa*heading(v,w)+Beta*dist(v,w)+Gamma*vel(v,w)
        robot.Alfa = 2  #célra tarás súlya
        robot.Beta = 3 # Akadály kerülés súlya
        robot.Gamma = 1 #Sebesség súlya
       
"""
Robot jelenlegi állapotának eltárolása
"""
class robotstate:
    def __init__(robot, x=0, y=0, v=0, w=0, irany = 0, w_L=0, w_R=0):  #default konstruktor
        robot.x = x
        robot.y = y
        robot.v = v
        robot.w = w
        robot.irany = irany
        robot.w_L = w_L
        robot.w_R = w_R

"""
Robot állapotának frissítése
"""
def update(robot, config, w_L, w_R, dt):
   
        v, w = get_robot_kinematics(w_L, w_R, config)
        theta_0 = robot.irany
       
        #1.eset --> ha kanyarodik w!=0
        if(abs(w)!=0):
            robot.x += (v/w) * (math.sin(theta_0 + w*dt) - math.sin(theta_0 ))
            robot.y += - (v/w) * (math.cos(theta_0 + w*dt) - math.cos(theta_0))
        else:
            #2.eset --> ha w=0 --> egyenesen halad
            robot.x += v*math.cos(theta_0)*dt
            robot.y += v*math.sin(theta_0)*dt
       
        #otientáció frissítés
        robot.irany += w*dt
       
        robot.v = v
        robot.w = w
       
        robot.w_L = w_L
        robot.w_R = w_R
       
        # Visszaadjuk a teljes állapotvektort
        return np.array([robot.x, robot.y, robot.irany, robot.v, robot.w, robot.w_L, robot.w_R])

"""        
Összes választható v,w pár kiszámolása kerekekből számítva
"""
def pairstochoose (robotconfig, robotstate):

    w_dot_max = robotconfig.a_max / robotconfig.r_kerek         #w=a/r
   
    w_L_min_actual = max(robotconfig.w_kerek_min, robotstate.w_L - w_dot_max * robotconfig.dt)
    w_L_max_actual = min(robotconfig.w_kerek_max, robotstate.w_L + w_dot_max * robotconfig.dt)

    w_R_min_actual = max(robotconfig.w_kerek_min, robotstate.w_R - w_dot_max * robotconfig.dt)
    w_R_max_actual = min(robotconfig.w_kerek_max, robotstate.w_R + w_dot_max * robotconfig.dt)
   
    w_L_samples = np.arange(w_L_min_actual, w_L_max_actual + robotconfig.w_kerek_resolution, robotconfig.w_kerek_resolution)
    w_R_samples = np.arange(w_R_min_actual, w_R_max_actual + robotconfig.w_kerek_resolution, robotconfig.w_kerek_resolution)

    all_pairs = []
    for w_L in w_L_samples:
        for w_R in w_R_samples:
            v, w = get_robot_kinematics(w_L, w_R, robotconfig)

            all_pairs.append([v, w, w_L, w_R])
    return all_pairs

########################Összes lehetséges válaztható pálya eltárolása ami biztonságos megállást biztosít, ez frissüljön minden egyes döntáshozásnál#######################x

def safepairs (robotconfig, state, obstacle_list):
    w_dot_max = robotconfig.a_max / robotconfig.r_kerek
    
    # Dinamikus ablak kiszámítása (mint a pairstochoose-ban)
    w_L_min_actual = max(robotconfig.w_kerek_min, state.w_L - w_dot_max * robotconfig.dt)
    w_L_max_actual = min(robotconfig.w_kerek_max, state.w_L + w_dot_max * robotconfig.dt)

    w_R_min_actual = max(robotconfig.w_kerek_min, state.w_R - w_dot_max * robotconfig.dt)
    w_R_max_actual = min(robotconfig.w_kerek_max, state.w_R + w_dot_max * robotconfig.dt)
    
    w_L_samples = np.arange(w_L_min_actual, w_L_max_actual + robotconfig.w_kerek_resolution, robotconfig.w_kerek_resolution)
    w_R_samples = np.arange(w_R_min_actual, w_R_max_actual + robotconfig.w_kerek_resolution, robotconfig.w_kerek_resolution)

    admissible_list = [] # Ebben csak a "jó" párok lesznek
    
    for w_L in w_L_samples:
        for w_R in w_R_samples:
            if AdmissableVelocity(robotconfig, w_L, w_R, state, obstacle_list):
                v, w = get_robot_kinematics(w_L, w_R, robotconfig)
                admissible_list.append([v, w, w_L, w_R])
                
    return admissible_list

################################################

############# optimalizáció #####################
def optimalisation (robotconfig, state, safe_pairs, goal, obstacle_list):
    return best_pair


################################################

"""
szimulációs rész
"""
def plot_all_trajectories(state, config, all_pairs, obstacles):
    plt.figure(figsize=(10, 10))
    plt.grid(True)
    
    #Kirajzolás miatti rész ez
    # 1. AKADÁLYOK kirajzolása és határaik begyűjtése
    all_x = [obs.x for obs in obstacles] + [state.x]
    all_y = [obs.y for obs in obstacles] + [state.y]
    
    for obs in obstacles:
        circle = plt.Circle((obs.x, obs.y), obs.radius, color="red", alpha=0.6)
        plt.gca().add_artist(circle)
    
    #Kirajzolás miatti rész ez
    # Kézi tengelybeállítás az akadályok és a robot alapján
    # Hagyunk egy kis margót (0.5m) a széleken
    plt.xlim(min(all_x) - 0.5, max(all_x) + 0.5)
    plt.ylim(min(all_y) - 0.5, max(all_y) + 0.5)
    
    plt.gca().set_aspect('equal', adjustable='box')
        
    # 1. Összes lehetséges PÁLYA kirajzolása 
    for p_v, p_w, w_L, w_R in all_pairs:
        
        is_safe = AdmissableVelocity(config, w_L, w_R, state, obstacles)
        dist_to_coll = get_dist_on_trajectory(state, config, w_L, w_R, obstacles)
        if dist_to_coll < (abs(p_v) * config.predict_time) or not is_safe:
            path_color = "-r" 
        elif p_v > 0:
            path_color = "-g"
        else:
            path_color = "-b"

        
        
        # 1. JÓSOLT SZAKASZ SZIMULÁCIÓJA
        ghost_robot = robotstate(x=state.x, y=state.y, irany=state.irany, w_L=state.w_L, w_R=state.w_R)
        path_x, path_y = [ghost_robot.x], [ghost_robot.y]

        for _ in np.arange(0, config.predict_time, config.dt):
            update(ghost_robot, config, w_L, w_R, config.dt)
            path_x.append(ghost_robot.x)
            path_y.append(ghost_robot.y)
           
        plt.plot(path_x, path_y, path_color, alpha=0.2)
        
        # 2. FÉKEZÉSI SZAKASZ SZIMULÁCIÓJA (ÍVES PIROS PÁLYA)
        # Megállási idő kiszámítása: t = v / a
        t_stop = abs(p_v) / config.a_max if abs(p_v) > 0 else 0
        
        # A fékezést a jósolt szakasz végpontjáról (ghost_robot) indítjuk
        brake_robot = robotstate(x=ghost_robot.x, y=ghost_robot.y, irany=ghost_robot.irany, w_L=w_L, w_R=w_R)
        brake_x, brake_y = [brake_robot.x], [brake_robot.y]
        
        # Lassulás szimulációja: a keréksebességek fokozatosan csökkennek
        stop_steps = np.arange(0, t_stop, config.dt)
        for i, _ in enumerate(stop_steps):
            # Sebességcsökkentési faktor (1.0 -> 0.0)
            factor = max(0, 1.0 - (i * config.dt / t_stop)) if t_stop > 0 else 0
            # Frissítjük a szellemrobotot a csökkentett keréksebességekkel
            update(brake_robot, config, w_L * factor, w_R * factor, config.dt)
            brake_x.append(brake_robot.x)
            brake_y.append(brake_robot.y)
            
        # Piros ív kirajzolása (ez már követi a kanyart)
        plt.plot(brake_x, brake_y, "-r", alpha=0.3)
   
   
    # 2. A robot és az irány kirajzolása
    robot_circle = plt.Circle((state.x, state.y), config.rob_radius, color="b", fill=False)
    plt.gca().add_artist(robot_circle)
    plt.arrow(state.x, state.y, 
              config.rob_radius * math.cos(state.irany),
              config.rob_radius * math.sin(state.irany), 
              head_width=0.05,
              head_length=0.07,
              fc='b', ec='b')
    
    # Tengelyfeliratok és mértékegységek hozzáadása
    plt.xlabel("X pozíció [m]")
    plt.ylabel("Y pozíció [m]")
    plt.title("Robot pályák jóslása (zöld és kék)")
   
    plt.show()

# Futtatás
conf = robotconfig()
curr_state = robotstate(x=0, y=0, v=0.5, w=0.1, irany=0)
pairs = pairstochoose(conf, curr_state)
plot_all_trajectories(curr_state, conf, pairs, obstacle_list)


"""
import matplotlib.pyplot as plt
import math
import numpy as np

# --- ALAPOSZTÁLYOK ---

class robotconfig:
    def __init__(self):
        self.r_kerek = 0.033
        self.b = 0.08
        self.w_kerek_max = 15.0
        self.w_kerek_min = -15.0
        self.a_max = 2.0
        self.w_kerek_resolution = 2.0  # Ritkább mintavétel a gyorsabb rajzolásért
        self.dt = 0.1
        self.predict_time = 2.5
        self.rob_radius = 0.12
        
        # Súlyok a döntéshez
        self.Alfa = 4.0   # Célra tartás (magasabb = agresszívabb célkövetés)
        self.Beta = 2.0   # Akadálykerülés
        self.Gamma = 1.0  # Sebesség
        
class robotstate:
    def __init__(self, x=0, y=0, v=0, w=0, irany=0, w_L=0, w_R=0):
        self.x, self.y, self.v, self.w = x, y, v, w
        self.irany, self.w_L, self.w_R = irany, w_L, w_R

class Obstacle:
    def __init__(self, x, y, radius=0.15):
        self.x, self.y, self.radius = x, y, radius

# --- FÜGGVÉNYEK ---

def get_robot_kinematics(w_L, w_R, config):
    v = ((w_L + w_R) / 2) * config.r_kerek
    w = ((w_R - w_L) / (2 * config.b)) * config.r_kerek
    return v, w

def update(robot, config, w_L, w_R, dt):
    v, w = get_robot_kinematics(w_L, w_R, config)
    theta_0 = robot.irany
    if abs(w) > 1e-5:
        robot.x += (v/w) * (math.sin(theta_0 + w*dt) - math.sin(theta_0))
        robot.y += - (v/w) * (math.cos(theta_0 + w*dt) - math.cos(theta_0))
    else:
        robot.x += v * math.cos(theta_0) * dt
        robot.y += v * math.sin(theta_0) * dt
    robot.irany += w * dt
    robot.v, robot.w, robot.w_L, robot.w_R = v, w, w_L, w_R
    return robot

def get_dist_on_trajectory(state, config, w_L, w_R, obstacle_list):
    temp_robot = robotstate(x=state.x, y=state.y, irany=state.irany, w_L=state.w_L, w_R=state.w_R)
    accumulated_dist = 0.0
    v_actual, _ = get_robot_kinematics(w_L, w_R, config)
    for _ in np.arange(0, config.predict_time, config.dt):
        for obs in obstacle_list:
            if math.hypot(temp_robot.x - obs.x, temp_robot.y - obs.y) <= (config.rob_radius + obs.radius):
                return accumulated_dist 
        update(temp_robot, config, w_L, w_R, config.dt)
        accumulated_dist += abs(v_actual) * config.dt
    return 10.0

def AdmissableVelocity(robotconfig, w_L, w_R, state, obstacle_list):
    dist = get_dist_on_trajectory(state, robotconfig, w_L, w_R, obstacle_list)
    v, _ = get_robot_kinematics(w_L, w_R, robotconfig)
    return abs(v) <= math.sqrt(2 * robotconfig.a_max * dist)

def pairstochoose(robotconfig, robotstate):
    w_dot_max = robotconfig.a_max / robotconfig.r_kerek
    w_L_min = max(robotconfig.w_kerek_min, robotstate.w_L - w_dot_max * robotconfig.dt)
    w_L_max = min(robotconfig.w_kerek_max, robotstate.w_L + w_dot_max * robotconfig.dt)
    w_R_min = max(robotconfig.w_kerek_min, robotstate.w_R - w_dot_max * robotconfig.dt)
    w_R_max = min(robotconfig.w_kerek_max, robotstate.w_R + w_dot_max * robotconfig.dt)
    
    samples_L = np.arange(w_L_min, w_L_max + 0.1, robotconfig.w_kerek_resolution)
    samples_R = np.arange(w_R_min, w_R_max + 0.1, robotconfig.w_kerek_resolution)
    return [[get_robot_kinematics(l, r, robotconfig)[0], get_robot_kinematics(l, r, robotconfig)[1], l, r] 
            for l in samples_L for r in samples_R]

def optimisation(robotconfig, state, goal, obstacle_list):
    all_pairs = pairstochoose(robotconfig, state)
    best_score = -float('inf')
    best_pair = None
    safe_pairs = []

    for v, w, w_L, w_R in all_pairs:
        is_safe = AdmissableVelocity(robotconfig, w_L, w_R, state, obstacle_list)
        if not is_safe: continue
        
        # Predikció végpontja a pontozáshoz
        temp_robot = robotstate(x=state.x, y=state.y, irany=state.irany, w_L=state.w_L, w_R=state.w_R)
        for _ in np.arange(0, robotconfig.predict_time, robotconfig.dt):
            update(temp_robot, robotconfig, w_L, w_R, robotconfig.dt)
        
        angle_to_goal = math.atan2(goal[1] - temp_robot.y, goal[0] - temp_robot.x)
        score_heading = math.pi - abs(math.atan2(math.sin(angle_to_goal - temp_robot.irany), math.cos(angle_to_goal - temp_robot.irany)))
        score_dist = get_dist_on_trajectory(state, robotconfig, w_L, w_R, obstacle_list)
        score_vel = abs(v)

        score = (robotconfig.Alfa * score_heading + robotconfig.Beta * score_dist + robotconfig.Gamma * score_vel)
        
        safe_pairs.append([v, w, w_L, w_R])
        if score > best_score:
            best_score = score
            best_pair = [v, w, w_L, w_R]
            
    return best_pair, all_pairs

# --- SZIMULÁCIÓ ---

def run_dwa():
    conf = robotconfig()
    state = robotstate(x=0, y=0, irany=0, w_L=0, w_R=0)
    goal = [4.0, 3.0]
    obstacles = [Obstacle(1.2, 0.8), Obstacle(2.5, 2.0), Obstacle(1.0, 2.5), Obstacle(3.0, 1.0)]
    
    traj_x, traj_y = [state.x], [state.y]
    
    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 7))

    for step in range(200):
        best, all_pairs = optimisation(conf, state, goal, obstacles)
        
        if best is None: break
        
        ax.clear()
        # 1. Összes pálya rajzolása (piros = veszélyes, szürke = lassú/lehetséges)
        for v, w, w_L, w_R in all_pairs:
            is_safe = AdmissableVelocity(conf, w_L, w_R, state, obstacles)
            ghost = robotstate(x=state.x, y=state.y, irany=state.irany)
            px, py = [ghost.x], [ghost.y]
            for _ in np.arange(0, conf.predict_time, conf.dt * 2): # Ritkább pontok a gyorsasághoz
                update(ghost, conf, w_L, w_R, conf.dt * 2)
                px.append(ghost.x)
                py.append(ghost.y)
            color = 'red' if not is_safe else 'gray'
            ax.plot(px, py, color=color, alpha=0.15, lw=0.5)

        # 2. VÁLASZTOTT pálya kiemelése (Zöld)
        ghost = robotstate(x=state.x, y=state.y, irany=state.irany)
        gx, gy = [ghost.x], [ghost.y]
        for _ in np.arange(0, conf.predict_time, conf.dt):
            update(ghost, conf, best[2], best[3], conf.dt)
            gx.append(ghost.x)
            gy.append(ghost.y)
        ax.plot(gx, gy, color='lime', lw=3, label="Választott irány")

        # 3. Robot, akadályok és cél
        ax.plot(traj_x, traj_y, "-b", lw=1)
        ax.plot(goal[0], goal[1], "gx", markersize=12, mew=3, label="Cél")
        for obs in obstacles:
            ax.add_patch(plt.Circle((obs.x, obs.y), obs.radius, color="red", alpha=0.5))
        ax.add_patch(plt.Circle((state.x, state.y), conf.rob_radius, color="blue", fill=False, lw=2))

        # Állapot frissítése
        update(state, conf, best[2], best[3], conf.dt)
        traj_x.append(state.x)
        traj_y.append(state.y)

        # Keret beállítása (dinamikus)
        ax.set_xlim(min(0, state.x)-1, max(goal[0], state.x)+1)
        ax.set_ylim(min(0, state.y)-1, max(goal[1], state.y)+1)
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', alpha=0.6)
        plt.title(f"DWA Szimuláció - Lépés: {step} | v: {state.v:.2f} m/s")
        plt.pause(0.001)

        if math.hypot(state.x - goal[0], state.y - goal[1]) < 0.15:
            print("Célba ért!")
            break

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    run_dwa()
    """