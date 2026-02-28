import matplotlib.pyplot as plt     # kirajzolásra- szimuláció           --> cmd:pip install matplotlib numpy
import math                         # matematikai műveletekre pl.:pi

import numpy as np      #gyorsabb számolás miatt --> ez C-ben írodott


def get_robot_kinematics(w_L, w_R, config):
    v = ((w_L + w_R) / 2) * config.r_kerek
    w = ((w_R - w_L) / (2 * config.b)) * config.r_kerek
    return v, w

class Obstacle:
    def __init__(self, x, y, radius=0.1):
        self.x = x
        self.y = y
        self.radius = radius
        
obstacle_list = [
    Obstacle(1.5, 0.5, radius=0.15),  
    Obstacle(2.0, -0.2, radius=0.08),
    Obstacle(0.8, 1.2, radius=0.2),
    Obstacle(2.5, 1.0, radius=0.1)
]


def get_dist_on_trajectory(state, config, w_L, w_R, obstacle_list):
    """
    Kiszámítja a megtett utat az ív mentén az első ütközésig.
    Ha nincs ütközés a predict_time alatt, egy nagy értéket ad vissza.
    """
    # Létrehozunk egy szimulációs robotot
    temp_robot = robotstate(x=state.x, y=state.y, irany=state.irany, w_L=state.w_L, w_R=state.w_R)
    
    accumulated_dist = 0.0
    v_actual, _ = get_robot_kinematics(w_L, w_R, config)
    step_dist = abs(v_actual) * config.dt # Egy időlépés alatt megtett út
    
    # Végigszimuláljuk az utat
    for _ in np.arange(0, config.predict_time, config.dt):
        update(temp_robot, config, w_L, w_R, config.dt)
        accumulated_dist += step_dist
        
        # Ellenőrizzük az összes akadályt ebben a pontban
        for obs in obstacle_list:
            dist_centers = math.hypot(temp_robot.x - obs.x, temp_robot.y - obs.y)
            # Ha a robot széle és az akadály széle összeér
            if dist_centers <= (config.rob_radius + obs.radius):
                return accumulated_dist # Megállunk, megvan az ütközési távolság
                
    return 10.0 # Ha nincs ütközés a jósolt távon belül, egy fix nagy értéket adunk vissza



#############xx FÉKEZÉSI FELTÉTELEK ###############
def AdmissableVelocity(robotconfig, w_L, w_R, state, obstacle_list):
    """
    Ellenőrzi, hogy a robot meg tud-e állni az adott íven az ütközés előtt.
    """
    # 1. Kiszámítjuk a távolságot az ív mentén az ütközésig
    dist_to_obs = get_dist_on_trajectory(state, robotconfig, w_L, w_R, obstacle_list)
    
    # 2. Megkapjuk a lineáris sebességet (v)
    v, _ = get_robot_kinematics(w_L, w_R, robotconfig)
    
    # Biztonsági feltétel: v <= sqrt(2 * a_max * dist)
    # Ez a fizikai határhelyzet (v^2 = 2as)
    if abs(v) <= math.sqrt(2 * robotconfig.a_max * dist_to_obs):
        return True
    return False
   
############################################################



class robotconfig:
    def __init__(robot):     #konstruktor
        """ #kör alakú robot esetén
        robot.v_max = 2 # [m/s]
        robot.v_min = -1 # [m/s]
        robot.w_max = 2  # [rad/s]
        robot.w_min = -1 # [rad/s]
        robot.a_max = 4 # [m/s^2] gyorsulás
        """
        """#Differenciális meghajtású robot esetén"""
        # feltételezem hogy a negativ és poziti sebességek megegyeznek v_max = v_min és w_max=w_min --> ezeket kiszámolom a kerekek adataibol és a robot konfigurációjából/ felépítéséből
        robot.r_kerek = 0.033   #[m]
        robot.b = 0.08 #[m]
        robot.w_kerek_max = 15 #rad/s
        robot.a_max = 2 # [m/s^2] gyorsulás
       
        # v kiszámitása a paraméterek alapján
        #robot.v_max = robot.w_kerek_max * robot.r_kerek
        #robot.v_min = -robot.v_max
       
        # w kiszámítása a paraméterek alapján
        #robot.w_max = (robot.w_kerek_max* robot.r_kerek)/ robot.b
        #robot.w_min = -robot.w_max
       
        robot.w_kerek_min = -robot.w_kerek_max
       
        # Felbontások a mintavételezéshez (most a kerék szögsebességére vonatkozóan)
        robot.w_kerek_resolution = 1.5 # [rad/s] A kerék szögsebesség mintavételezési finomsága.
       
       
        # Felbontások a mintavételezéshez
        #robot.resolution = 0.1 #A lineáris sebesség  mintavételezési finomsága m/s. --> Szerepe: A DWA nem egy végtelen tartományban keres, hanem „lépked”. Ez a szám adja meg, mekkora közökkel tesztelje a lehetséges sebességeket.Példa: Ha a min sebesség 0, a max 1, és a felbontás 0.1, akkor a kód megnézi a 0.0, 0.1, 0.2 ... 1.0 értékeket.Hatása: Minél kisebb (finomabb), annál pontosabb lesz a mozgás, de annál több számítást kell végeznie a processzornak (lassabb lesz a kód).
        #robot.turn_resolution = 5 * math.pi / 180.0  # [rad/s] -- kanyarodási felbontás --> hasonló szerepe mint a fenti sebesség felbontásnak
       
        #DWA időparaméterek
        robot.dt = 0.1 # [s] --> időléspés (felbontás)
        robot.predict_time = 2 # [s] --> mennyi időre lásson előre
   
        #robot-alakja--> legyen kör az egyszerűség kedvéért
        #robot.rob_radius = 0.1 # [m]
        robot.rob_radius = robot.b + 0.02 #[m]
       
       
        #G(v,w)= Alfa*heading(v,w)+Beta*dist(v,w)+Gamma*vel(v,w)
        robot.Alfa = 2  #célra tarás súlya
        robot.Beta = 3 # Akadály kerülés súlya
        robot.Gamma = 1 #Sebesség súlya
       
class robotstate:
    def __init__(robot, x=0, y=0, v=0, w=0, irany = 0, w_L=0, w_R=0):  #default konstruktor
        robot.x = x
        robot.y = y
        robot.v = v
        robot.w = w
        robot.irany = irany
        robot.w_L = w_L
        robot.w_R = w_R
       
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
       
        # Visszaadjuk a teljes állapotvektort, ha szükséges
        return np.array([robot.x, robot.y, robot.irany, robot.v, robot.w, robot.w_L, robot.w_R])

def pairstochoose (robotconfig, robotstate):
    """        
    Összes választható v,w pár kiszámolása
    """
    #v_min_actual = max(robotconfig.v_min, robotstate.v - robotconfig.a_max * robotconfig.dt)
    #v_max_actual = min(robotconfig.v_max, robotstate.v + robotconfig.a_max * robotconfig.dt)
       
    #w_min_actual = max(robotconfig.w_min, robotstate.w - robotconfig.a_max * robotconfig.dt)
    #w_max_actual = min(robotconfig.w_max, robotstate.w + robotconfig.a_max * robotconfig.dt)
   
    # Aktuális kerék szögsebesség határok a gyorsulás figyelembevételével
    # Feltételezzük, hogy a_max a lineáris gyorsulásra vonatkozik,
    # és ebből számoljuk vissza a kerék szöggyorsulását.
    w_dot_max = robotconfig.a_max / robotconfig.r_kerek
   
    w_L_min_actual = max(robotconfig.w_kerek_min, robotstate.w_L - w_dot_max * robotconfig.dt)
    w_L_max_actual = min(robotconfig.w_kerek_max, robotstate.w_L + w_dot_max * robotconfig.dt)

    w_R_min_actual = max(robotconfig.w_kerek_min, robotstate.w_R - w_dot_max * robotconfig.dt)
    w_R_max_actual = min(robotconfig.w_kerek_max, robotstate.w_R + w_dot_max * robotconfig.dt)
   
    w_L_samples = np.arange(w_L_min_actual, w_L_max_actual + robotconfig.w_kerek_resolution, robotconfig.w_kerek_resolution)
    w_R_samples = np.arange(w_R_min_actual, w_R_max_actual + robotconfig.w_kerek_resolution, robotconfig.w_kerek_resolution)

    # 2. Listák létrehozása a felbontások alapján (np.arange-dzsel)
    # v_resolution: m/s lépésköz, turn_resolution: rad/s lépésköz
    #v_samples = np.arange(v_min_actual, v_max_actual, robotconfig.resolution)
    #w_samples = np.arange(w_min_actual, w_max_actual, robotconfig.turn_resolution)

    # 3. Összes párosítás előállítása
    #all_pairs = []
    #for v in v_samples:
    #        for w in w_samples:
    #            all_pairs.append([v, w])
    #return all_pairs
   
    all_pairs = []      #a kerekekbol szamolva
    for w_L in w_L_samples:
        for w_R in w_R_samples:
            v, w = get_robot_kinematics(w_L, w_R, robotconfig)

            all_pairs.append([v, w, w_L, w_R])
    return all_pairs

    """szimulációs rész --> kirajzolás"""
################################################

def get_braking_limit_distance(config, v):
    """
    Kiszámítja azt a minimális távolságot (határhelyzetet), 
    amin belül ha akadály van, a robot már nem tudna megállni.
    
    v: a robot aktuális lineáris sebessége [m/s]
    """
    # A képlet: v^2 = 2 * a * s  --> s = v^2 / (2 * a)
    # Az abszolút értéket használjuk, hogy hátramenetben is pozitív távolságot kapjunk.
    stopping_distance = (v**2) / (2 * config.a_max)
    
    # Érdemes hozzáadni egy kis biztonsági puffert (pl. a robot sugarát), 
    # hogy ne centire az akadály előtt álljon meg a robot közepe.
    safety_margin = config.rob_radius 
    
    return stopping_distance + safety_margin

def plot_all_trajectories(state, config, all_pairs, obstacles):
    plt.figure(figsize=(10, 10))
    plt.grid(True)
    
    #Kirajzolás miatti rész ez
    # 1. Akadályok kirajzolása és határaik begyűjtése
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
        
    # 1. Összes lehetséges pálya kirajzolása
    """
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
    """
   
    for p_v, p_w, w_L, w_R in all_pairs:
        # Szín: Előre -> zöld, Hátra -> kék
        path_color = "-g" if p_v >= 0 else "-b"
        
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


""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"""
#v,w választott párból a kerekeknek a vezérlésének kiszámítása
def wheelcontrollcalcuate(robotconfig, v, w):
    Kiszámítja a kerekek lineáris és szögsebességét.
    v: választott sebesség [m/s]
    w: választott szögsebesség [rad/s]
   
    w1_kerek = (v-w*robotconfig.b)/robotconfig.r_kerek
    w2_kerek = (v+w*robotconfig.b)/robotconfig.r_kerek
   
    return (w1_kerek, w2_kerek)
"""
   
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

# Futtatás
conf = robotconfig()
# Példa: a robot már 0.5 m/s-mal megy és kicsit kanyarodik
curr_state = robotstate(x=0, y=0, v=0.5, w=0.1, irany=0)
pairs = pairstochoose(conf, curr_state)
plot_all_trajectories(curr_state, conf, pairs, obstacle_list)