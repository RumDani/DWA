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
    def __init__(self, x, y, radius=0.1, vx=0.0, vy=0.0):
        self.x = x
        self.y = y
        self.radius = radius
        self.vx = vx  # sebesség x irányban
        self.vy = vy  # sebesség y irányban

    def move(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt


"""        
Akadályok példányosítása
"""
obstacle_list = [
    
    # Fix akadályok, amik "folyosóra" kényszerítik a robotot
    Obstacle(2.0, 3.5, radius=0.5),
    Obstacle(4.5, 1.0, radius=0.5),
    
    # A BLOKKOLÓ: Ez az óriási akadály keresztbe úszik a robot előtt.
    # A mérete (radius=1.2) biztosítja, hogy a robot aurájával (safety_margin) 
    # együtt ne maradjon biztonságos ív.
    Obstacle(3.0, -1.0, radius=1.2, vx=0.0, vy=0.45), 
    
    # Kisebb zavaró mozgó akadályok
    Obstacle(5.0, 4.0, radius=0.2, vx=-0.3, vy=0.0),
    Obstacle(1.0, 4.5, radius=0.15, vx=0.2, vy=-0.1)
    ]
    
"""""
    Obstacle(0.6, 0.25, radius=0.15),
    Obstacle(1.5, 0.5, radius=0.15),  
    Obstacle(2.0, -0.2, radius=0.08),
    Obstacle(0.8, 1.2, radius=0.2),
    Obstacle(2.5, 1.0, radius=0.1),
    Obstacle(1.0, 2.5, radius=0.15),
    Obstacle(3.0, 0.5, radius=0.3),
    Obstacle(4.5, 1.5, radius=0.2),
    Obstacle(2.0, 3.5, radius=0.25),
    Obstacle(5.5, 3.5, radius=0.3),
    Obstacle(4.0, 4.5, radius=0.2),
    Obstacle(6.5, 2.0, radius=0.2),
    Obstacle(2.5, 1.5, 0.25, vx=0.2, vy=0.1),  # Mozgó
    Obstacle(4.5, 1.5, 0.2, vx=-0.15, vy=0.3), # Mozgó
    Obstacle(3.0, 3.0, 0.25, vx=0.3, vy=-0.2), # Mozgó
    Obstacle(3.5, 0.0, radius=0.6, vx=0.0, vy=0.4)
    """


"""
Cél kitűzése
"""
class Goal:
    def __init__(self, x, y, tolerance=0.15):
        self.x = x
        self.y = y
        self.tolerance = tolerance

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
            if dist_centers <= (config.rob_radius + obs.radius + config.safety_margin):
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
        robot.w_kerek_resolution = 0.7 # [rad/s] A kerék szögsebesség mintavételezési finomsága.
       
        #DWA időparaméterek
        robot.dt = 0.1 # [s] --> időléspés (felbontás)
        robot.predict_time = 3 # [s] --> mennyi időre lásson előre
   
        #robot-alakja--> legyen kör az egyszerűség kedvéért
        robot.rob_radius = robot.b + 0.02 #[m]
        
        robot.safety_margin = 0.15
       
        ########Optimalizáció###########
       
        #G(v,w)= Alfa*heading(v,w)+Beta*dist(v,w)+Gamma*vel(v,w)
        robot.Alfa = 3  #célra tarás súlya  2
        robot.Beta = 3 # Akadály kerülés súlya  2
        robot.Gamma = 4 #Sebesség súlya 5
       
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
def optimisation(robotconfig, state, goal, obstacle_list):
    # 1. Biztonságos sebességpárok lekérése (Dinamikus ablak + Ütközésvizsgálat)
    safe_list = safepairs(robotconfig, state, obstacle_list)
    
    # Ha nincs biztonságos út, vészfékezés
    if not safe_list:
        return [0.0, 0.0, 0.0, 0.0], []

    best_score = -float('inf')
    best_pair = None
    
    dist_to_goal = math.hypot(goal.x - state.x, goal.y - state.y)
    v_max_robot = robotconfig.w_kerek_max * robotconfig.r_kerek
    
    # 2. Adaptív predikció: közelebb rövidebb időre nézünk előre a precizitásért
    adaptive_predict_time = robotconfig.predict_time
    if dist_to_goal < 1.0:
        adaptive_predict_time = max(0.5, dist_to_goal * 2.0)

    alfa = robotconfig.Alfa    # Célra tartás (pl. 2.0)
    beta = robotconfig.Beta    # Akadálykerülés (pl. 1.0)
    gamma = robotconfig.Gamma          # Sebességszabályozás súlya (magasabb, hogy hallgasson a lassításra)

    for v, w, w_L, w_R in safe_list:  
        # Gyors szimuláció a jövőbeli állapot becsléséhez
        temp_robot = robotstate(x=state.x, y=state.y, irany=state.irany, w_L=state.w_L, w_R=state.w_R)
        for _ in np.arange(0, adaptive_predict_time, robotconfig.dt):
            update(temp_robot, robotconfig, w_L, w_R, robotconfig.dt)
        
        # --- PONTOZÁSI LOGIKA ---

        # A: Heading score (Irányhiba normalizálva 0 és 1 közé)
        angle_to_goal = math.atan2(goal.y - temp_robot.y, goal.x - temp_robot.x)
        error_angle = math.atan2(math.sin(angle_to_goal - temp_robot.irany), 
                                 math.cos(angle_to_goal - temp_robot.irany))
        score_heading = (math.pi - abs(error_angle)) / math.pi
        
        # B: Dist score (Akadályoktól való távolság)
        score_dist = get_dist_on_trajectory(state, robotconfig, w_L, w_R, obstacle_list)
        
        # C: Velocity score (Folytonos lassulási görbe)
        if dist_to_goal > 0.8:
            # Messze: egyszerűen jutalmazzuk a sebességet (0.0 - 1.0)
            score_vel = v / v_max_robot
        else:
            # Közel: v_target követése. v_target = távolság (pl. 0.5m-nél 0.5m/s az ideális)
            # A pontszám 1.0 ha tökéletesen tartja a lassulást, és csökken ha eltér tőle.
            v_target = max(0.0, dist_to_goal) 
            score_vel = 1.0 - (abs(v_target - v) / v_max_robot)

        # Végső pontszám kiszámítása
        score = (alfa * score_heading + 
                 beta * score_dist + 
                 gamma * score_vel)

        if score > best_score:
            best_score = score
            best_pair = [v, w, w_L, w_R]

    return best_pair, safe_list

"""
szimulációs rész
"""
def plot_all_trajectories(state, config, all_pairs, best_pair, goal, obstacles):
    plt.clf()
    ax = plt.gca()
    
    # Beállítjuk a keretet, hogy ne mozogjon a robot után
    ax.set_xlim(-1.0, 8.5) 
    ax.set_ylim(-1.0, 6.5)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linestyle='--', alpha=0.5)

    # 1. Cél és Akadályok rajzolása
    ax.plot(goal.x, goal.y, "gx", markersize=12, mew=3, label="Cél")
    for obs in obstacles:
        ax.add_patch(plt.Circle((obs.x, obs.y), obs.radius, color="red", alpha=0.3))
    
    # 2. Összes lehetséges pálya végigpörgetése
    for v, w, w_L, w_R in all_pairs:
        # PONTOS ELLENŐRZÉS: Ez-e a kiválasztott legjobb páros?
        is_best = False
        if best_pair is not None:
            if math.isclose(w_L, best_pair[2], abs_tol=1e-3) and \
               math.isclose(w_R, best_pair[3], abs_tol=1e-3):
                is_best = True

        is_safe = AdmissableVelocity(config, w_L, w_R, state, obstacles)
        dist_to_coll = get_dist_on_trajectory(state, config, w_L, w_R, obstacles)
        
        # --- SZÍNMEGHATÁROZÁS ---
        if dist_to_coll < (abs(v) * config.predict_time) or not is_safe:
            path_color = "red"    # Ütközésveszély = Piros
        elif is_best:
            path_color = "lime"   # Választott legjobb = Élénkzöld
        elif v > 0:
            path_color = "gray"   # Többi biztonságos előre = Szürke (halvány)
        else:
            path_color = "blue"   # Hátramenet = Kék (ha v < 0)

        # Megjelenítési beállítások: a legjobb út legyen legfelül és legvastagabb
        z_ord = 10 if is_best else 1
        alpha_val = 1.0 if is_best else 0.15
        lw_val = 3.5 if is_best else 0.8

        # --- JÓSOLT SZAKASZ (Trajectory rollout) ---
        ghost = robotstate(x=state.x, y=state.y, irany=state.irany, w_L=state.w_L, w_R=state.w_R)
        path_x, path_y = [ghost.x], [ghost.y]

        # Predikciós szimuláció
        for _ in np.arange(0, config.predict_time, config.dt):
            update(ghost, config, w_L, w_R, config.dt)
            path_x.append(ghost.x)
            path_y.append(ghost.y)
        
        ax.plot(path_x, path_y, color=path_color, alpha=alpha_val, linewidth=lw_val, zorder=z_ord)

        # --- FÉKEZÉSI SZAKASZ (A vonalak végén lévő ív) ---
        t_stop = abs(v) / config.a_max if abs(v) > 0 else 0
        brake_robot = robotstate(x=ghost.x, y=ghost.y, irany=ghost.irany, w_L=w_L, w_R=w_R)
        brake_x, brake_y = [brake_robot.x], [brake_robot.y]
        
        for t in np.arange(0, t_stop, config.dt):
            factor = max(0, 1.0 - (t / t_stop)) if t_stop > 0 else 0
            update(brake_robot, config, w_L * factor, w_R * factor, config.dt)
            brake_x.append(brake_robot.x)
            brake_y.append(brake_robot.y)
            
        # A fékezési szakasz mindig piros marad
        ax.plot(brake_x, brake_y, color="red", alpha=alpha_val * 0.7, linewidth=lw_val * 0.7, zorder=z_ord)

    # 3. A Robot és az aktuális megtett út nyomvonala (trail) rajzolása
    ax.add_patch(plt.Circle((state.x, state.y), config.rob_radius, color="blue", fill=False, lw=2))
    plt.arrow(state.x, state.y, config.rob_radius * math.cos(state.irany),
              config.rob_radius * math.sin(state.irany), head_width=0.05, fc='b', ec='b')

    plt.title(f"DWA Szimuláció - v: {state.v:.2f} m/s")
    plt.draw()
    plt.pause(0.01)

# --- FŐ PROGRAMFUTTATÁS ---

if __name__ == "__main__":
    conf = robotconfig()
    state = robotstate(x=0, y=0, irany=0, w_L=0, w_R=0)
    goal = Goal(x=7.0, y=5.0)
    
    plt.ion()
    plt.figure(figsize=(10, 8))

    for step in range(1000): # Hosszabb futás a várakozás miatt
        # 1. Akadályok mozgatása
        for obs in obstacle_list:
            obs.move(conf.dt)

        # 2. Optimalizáció
        best, all_pairs = optimisation(conf, state, goal, obstacle_list)
        
        # 3. Megjelenítés
        plot_all_trajectories(state, conf, all_pairs, best, goal, obstacle_list)
        
        # 4. Mozgatási logika + Várakozás
        if best:
            # Van út -> megyünk tovább
            update(state, conf, best[2], best[3], conf.dt)
        else:
            # NINCS ÚT -> Várakozás (kerekek megállítása)
            # Frissítjük az állapotot v=0-val, hogy a szimuláció ne álljon le
            update(state, conf, 0, 0, conf.dt)
            print(f"Lépés {step}: Akadály blokkol, várakozás...")
            
        # 5. Célba érés
        if math.hypot(state.x - goal.x, state.y - goal.y) < goal.tolerance:
            print("Célba ért!")
            break

    plt.ioff()
    plt.show()