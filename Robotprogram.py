#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import copy

# -----------------------------------
# Initialisation
# -----------------------------------

ev3 = EV3Brick()

# Initialisation des moteurs et capteurs
try:
    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)
    gripper_motor = Motor(Port.A)
    line_sensor = ColorSensor(Port.S3)
    obstacle_sensor = UltrasonicSensor(Port.S1)
    gyro_sensor = GyroSensor(Port.S2)  # Ajout du gyroscope sur le port S2

    # Test des moteurs
    left_motor.run(200)
    right_motor.run(200)
    wait(500)
    left_motor.stop()
    right_motor.stop()

    # Réinitialisation du gyroscope
    gyro_sensor.reset_angle(0)
    wait(100)  # Attendre que le gyroscope se stabilise

except OSError as e:
    ev3.screen.clear()
    ev3.screen.print("Erreur connexion:")
    ev3.screen.print(str(e))
    wait(5000)
    raise SystemExit

# Initialisation de la base motrice
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Paramètres du suivi de ligne
BLACK = 8
WHITE = 80
THRESHOLD = (BLACK + WHITE) / 2
PROPORTIONAL_GAIN = 1.2
DRIVE_SPEED = 150
OBSTACLE_DISTANCE = 50  # Distance en mm pour détecter un obstacle

# -----------------------------------
# Classes
# -----------------------------------

class Node:
    def __init__(self, idx):
        self.__idx = idx
        self.__x1 = -1
        self.__x2 = -1
        self.__graph = None
        self.__neighbors = {}
        self.__distances = {}
        self.__init_neighbors()

    def get_idx(self): return self.__idx
    def get_location(self): return self.__x1, self.__x2
    def set_location(self, loc): 
        self.__x1 = loc[0]
        self.__x2 = loc[1]
    def get_neighbors(self): return copy.deepcopy(self.__neighbors)
    def get_distances(self): return copy.deepcopy(self.__distances)
    def get_graph(self): return self.__graph

    def set_graph(self, graph):
        if isinstance(graph, Grid):
            if self.__graph is None:
                self.__graph = graph
            else:
                raise RuntimeError("Node already in graph")
        else:
            raise TypeError("Graph must be Grid")

    def add_neighbor(self, node, distance, direction):
        if self.__graph is None or node.get_graph() is None:
            raise ValueError("Nodes must be in a graph before adding neighbors")
        if node.get_graph() == self.__graph:
            if node not in self.__neighbors.values() and self.__neighbors.get(direction) is None:
                self.__neighbors[direction] = node
                self.__distances[direction] = distance
                return True
            else:
                raise ValueError("Neighbor exists or direction taken: " + direction)
        raise ValueError("Not in same graph")

    def distance_to_neighbor(self, node):
        for direction, neighbor in self.__neighbors.items():
            if neighbor == node:
                return self.__distances[direction]
        return -1

    def direction_to_neighbor(self, node):
        for direction, neighbor in self.__neighbors.items():
            if neighbor == node:
                return direction
        return None

    def __init_neighbors(self):
        self.__neighbors = {'left': None, 'right': None, 'up': None, 'down': None}
        self.__distances = {'left': -1, 'right': -1, 'up': -1, 'down': -1}

    def __str__(self):
        return "Node " + str(self.__idx)

class Grid:
    def __init__(self, row, col):
        self.__col = col
        self.__row = row
        self.__nodes = {}

    def get_size(self): return len(self.__nodes)
    def get_max_size(self): return self.__col * self.__row

    def add_node(self, node, row=-1, col=-1):
        if self.get_size() >= self.get_max_size():
            return False
        if not isinstance(node, Node):
            return False
        if node.get_idx() not in self.__nodes:
            node.set_graph(self)
            node.set_location((row, col))
            self.__nodes[node.get_idx()] = node
            return True
        return False

    def add_edge(self, n1, n2, distance, direction):
        if not isinstance(n1, Node): n1 = self.node_of(n1)
        if not isinstance(n2, Node): n2 = self.node_of(n2)
        if n1 is None or n2 is None:
            raise ValueError("One or both nodes not found in grid")
        success1 = n1.add_neighbor(n2, distance, direction)
        success2 = n2.add_neighbor(n1, distance, self._opposite_direction(direction))
        return success1, success2

    def node_of(self, idx):
        return self.__nodes.get(idx)

    def _opposite_direction(self, direction):
        opposites = {'left': 'right', 'right': 'left', 'up': 'down', 'down': 'up'}
        return opposites.get(direction, direction)

    def __str__(self):
        return "Grid [size=" + str(self.get_size()) + "]"

# -----------------------------------
# Fonctions de gestion du gyroscope
# -----------------------------------

def reset_gyro():
    """Réinitialise le gyroscope à zéro."""
    gyro_sensor.reset_angle(0)
    wait(100)  # Attendre que le gyroscope se stabilise

def turn_with_gyro(angle):
    """
    Fait tourner le robot d'un angle précis en utilisant le gyroscope.
    Angle positif = rotation vers la droite, angle négatif = rotation vers la gauche.
    """
    # Réinitialiser l'angle du gyroscope
    reset_gyro()
    
    # Calculer l'angle cible
    target_angle = angle
    
    # Définir la vitesse de rotation (plus lente quand on s'approche de l'angle cible)
    base_speed = 100
    
    # Tourner jusqu'à atteindre l'angle souhaité
    if angle > 0:  # Tourner à droite
        while gyro_sensor.angle() < target_angle:
            remaining = abs(target_angle - gyro_sensor.angle())
            speed = min(base_speed, max(30, remaining))
            left_motor.run(speed)
            right_motor.run(-speed)
            wait(10)
    else:  # Tourner à gauche
        while gyro_sensor.angle() > target_angle:
            remaining = abs(target_angle - gyro_sensor.angle())
            speed = min(base_speed, max(30, remaining))
            left_motor.run(-speed)
            right_motor.run(speed)
            wait(10)
    
    # Arrêter les moteurs
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)  # Pause pour stabilisation
    
    # Afficher l'angle final pour débogage
    ev3.screen.clear()
    ev3.screen.print("Angle cible: " + str(target_angle))
    ev3.screen.print("Angle final: " + str(gyro_sensor.angle()))
    wait(100)

# -----------------------------------
# Fonctions
# -----------------------------------

def adjust_direction(current_node, next_node, is_return=False):
    """Ajuste la direction du robot avant de suivre un segment en utilisant le gyroscope."""
    direction = current_node.direction_to_neighbor(next_node)
    if is_return:
        direction = current_node.get_graph()._opposite_direction(direction)
    
    if direction == "right":
        turn_with_gyro(90)
    elif direction == "left":
        turn_with_gyro(-90)
    elif direction == "down":
        turn_with_gyro(90 if is_return else -90)
    elif direction == "up":
        turn_with_gyro(-90 if is_return else 90)
    wait(200)  # Stabilisation après le virage

def check_for_obstacle():
    """Vérifie la présence d'un obstacle et le manipule si nécessaire."""
    if obstacle_sensor.distance() < OBSTACLE_DISTANCE:
        robot.stop()
        ev3.screen.clear()
        ev3.screen.print("Obstacle détecté!")
        gripper_motor.run_target(450, -700)  # Fermer la pince
        wait(500)
        return True
    return False

def follow_segment(current_node, next_node, is_return=False):
    """Suit un segment entre deux nœuds avec suivi de ligne et gestion d'obstacles."""
    distance = current_node.distance_to_neighbor(next_node)
    direction = current_node.direction_to_neighbor(next_node)
    has_object = False

    ev3.screen.clear()
    ev3.screen.print("De " + str(current_node.get_idx()) + " à " + str(next_node.get_idx()))
    ev3.screen.print(str(direction) + ": " + str(distance) + "mm")

    # Réinitialiser la distance parcourue
    robot.reset()

    # Ajuster la direction avec le gyroscope
    adjust_direction(current_node, next_node, is_return)

    # Gestion des obstacles (seulement à l'aller)
    if not is_return:
        has_object = check_for_obstacle()

    # Suivi de ligne
    while robot.distance() < distance:
        # Vérifier les obstacles pendant le trajet (seulement à l'aller)
        if not is_return and not has_object:
            has_object = check_for_obstacle()
            if has_object:
                ev3.screen.print("Objet saisi")
                
        deviation = line_sensor.reflection() - THRESHOLD
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)

    robot.stop()
    return has_object

def execute_path(path, nodes, is_return=False, action_at_end=None):
    """Exécute un trajet le long d'un chemin de nœuds avec des actions optionnelles à la fin."""
    ev3.screen.clear()
    ev3.screen.print("Trajet: " + str(path[0]) + "->" + str(path[-1]))
    ev3.speaker.beep()
    
    has_object = False
    
    for i in range(len(path) - 1):
        current = nodes[path[i]]
        next_node = nodes[path[i + 1]]
        
        # Suivre le segment
        segment_has_object = follow_segment(current, next_node, is_return=is_return)
        
        # Si un objet a été saisi pendant ce segment
        if segment_has_object:
            has_object = True
            
    # Exécuter l'action de fin si spécifiée
    if action_at_end and callable(action_at_end):
        action_at_end(nodes[path[-1]])
        
    return has_object

# -----------------------------------
# Actions spécifiques
# -----------------------------------

def action_at_f(node):
    """Action à effectuer au nœud F."""
    ev3.screen.clear()
    ev3.screen.print("Tourne droite F")
    turn_with_gyro(70)
    wait(500)
    ev3.screen.clear()
    ev3.screen.print("Dépôt objet")
    gripper_motor.run_target(500, 0)  # Ouvrir la pince
    wait(1000)

def action_at_n(node):
    """Action à effectuer au nœud N."""
    ev3.screen.clear()
    ev3.screen.print("Prise objet en N")
    gripper_motor.run_target(500, -700)  # Fermer la pince
    wait(1000)

def action_at_k(node):
    """Action à effectuer au nœud K."""
    ev3.screen.clear()
    ev3.screen.print("Dépôt objet en K")
    gripper_motor.run_target(500, 0)  # Ouvrir la pince
    wait(1000)

def action_at_g(node):
    """Action à effectuer au nœud G."""
    ev3.screen.clear()
    ev3.screen.print("Arrivée à G")
    ev3.speaker.beep()
    wait(500)
    gripper_motor.run_target(500, 0)

def action_at_j(node):
    """Action à effectuer au nœud J."""
    ev3.screen.clear()
    ev3.screen.print("Dépôt objet en J")
    gripper_motor.run_target(500, 0)  # Ouvrir la pince
    wait(1000)

# -----------------------------------
# Programme principal
# -----------------------------------

if __name__ == '__main__':
    try:
        # Création de la grille
        g = Grid(4, 4)

        # Ajout des nœuds
        nodes = {idx: Node(idx) for idx in ['A', 'B', 'C', 'D', 'E', 'F', 'N', 'G', 'I', 'J', 'K', 'L']}
        for node in nodes.values():
            g.add_node(node)

        # Ajout des arêtes
        edges = [
            ('A', 'B', 780, 'right'), ('B', 'C', 470, 'down'), ('C', 'D', 300, 'left'),
            ('D', 'E', 1600, 'up'), ('E', 'F', 350, 'right'), ('D', 'N', 420, 'down'),
            ('N', 'G', 330, 'left'), ('G', 'I', 420, 'down'), ('I', 'J', 555, 'left'),
            ('J', 'K', 940, 'up'), ('I', 'L', 300, 'right')
        ]
        
        for n1, n2, dist, dir in edges:
            g.add_edge(n1, n2, dist, dir)

        # Réinitialiser le gyroscope au démarrage
        reset_gyro()
       
        # Exécution des différents trajets
        # --- Parcours aller A -> F ---
        execute_path(['A', 'B', 'C', 'D', 'E', 'F'], nodes, is_return=False, action_at_end=action_at_f)
         
        # --- Parcours retour F -> D ---
        execute_path(['F', 'E', 'D'], nodes, is_return=True)
        
        # Préparation pour D -> N
        ev3.screen.clear()
        wait(500)
        
        # --- Parcours D -> N ---

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(90)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de I")
        robot.straight(450)  # Reculer de J
        gripper_motor.run_target(500, -700) 
        robot.stop()
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de I")
        robot.straight(-400)  # Reculer de J
        robot.stop()
        wait(300)

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(90)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de I")
        robot.straight(650)  # Reculer de J
        robot.stop()
        wait(300)

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(-90)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de I")
        robot.straight(500)  # Reculer de J
        robot.stop()
        wait(300)

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(90)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de I")
        robot.straight(900)  # Reculer de J
        robot.stop()
        wait(500)
        gripper_motor.run_target(500, 0)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de I")
        robot.straight(-900)  # Reculer de J
        robot.stop()
        wait(300)

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 2")
        turn_with_gyro(90)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de I")
        robot.straight(600)  # Reculer de J
        robot.stop()
        wait(300) 

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 2")
        turn_with_gyro(90)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de I")
        robot.straight(400)  # Reculer de J 
        robot.stop()
        wait(300) 


        # Retour final optimisé
        ev3.screen.clear()
        ev3.screen.print("Retour de I")
        robot.straight(-400)  # Reculer de J
        robot.stop()
        wait(300)

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(90)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de J")
        robot.straight(450)  # Reculer de J
        robot.stop()
        wait(500)
        gripper_motor.run_target(500, -700) 
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de J")
        robot.straight(-350)  # Reculer de J
        robot.stop()
        wait(300)

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(90)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de J")
        robot.straight(1000)  # Reculer de J
        robot.stop()

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(-80)

        wait(500)
        gripper_motor.run_target(500, 0)  # Ouvrir la pince
        wait(300)

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(80)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de J")
        robot.straight(-1300)  # Reculer de J
        robot.stop()
        
        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(180)
        wait(500)
        gripper_motor.run_target(500, -700) 
        wait(300)
        
        ev3.screen.clear()
        ev3.screen.print("Retour de J")
        robot.straight(-450)  # Reculer de J
        robot.stop()
        wait(500)
        gripper_motor.run_target(500, -700) 
        wait(300)

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(90)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de J")
        robot.straight(500)  # Reculer de J
        robot.stop()
        wait(500)
        gripper_motor.run_target(500, 0)  # Ouvrir la pince
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de J")
        robot.straight(-500)  # Reculer de J
        robot.stop()
        wait(500)
 
        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(-90)
        wait(300)

        ev3.screen.clear()
        ev3.screen.print("Retour de J")
        robot.straight(600)  # Reculer de J
        robot.stop()
        wait(500)
        # Prendre un objet en G
        ev3.screen.clear()
        ev3.screen.print("Prise objet en G")
        wait(500)
        gripper_motor.run_target(500, -700)  # Fermer la pince
        wait(1000)
        # Reculer et aller vers L avec dépôt

        # Reculer et aller vers L avec dépôt
        ev3.screen.clear()
        ev3.screen.print("Recul de 380mm")
        robot.stop()  # S'assurer que le robot est arrêté avant de commencer
        wait(200)
        robot.straight(-380)  # Reculer de 380 mm
        robot.stop()
        wait(300)  # Pause pour stabilisation

        # Réinitialiser le gyroscope avant le virage
        reset_gyro()
        ev3.screen.print("Tournant vers L")
        turn_with_gyro(-90)
        wait(300)  # Pause pour stabilisation

        robot.straight(300)   # Avancer vers L
        robot.stop()
        wait(200)
        gripper_motor.run_target(500, 0)  # Ouvrir la pince
        wait(500)

        # Retour final optimisé
        ev3.screen.clear()
        ev3.screen.print("Retour de L")
        robot.straight(-300)  # Reculer de L
        robot.stop()
        wait(300)

        # Réinitialiser le gyroscope avant chaque virage
        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 1")
        turn_with_gyro(-90)
        wait(300)

        ev3.screen.print("Segment 1")
        robot.straight(-420)  # Premier segment de retour
        robot.stop()
        wait(300)

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage 2")
        turn_with_gyro(90)
        wait(300)

        ev3.screen.print("Segment 2")
        robot.straight(-680)  # Deuxième segment
        robot.stop()
        wait(300)

        reset_gyro()
        wait(100)
        ev3.screen.print("Virage final")
        turn_with_gyro(-90)
        wait(300)

        ev3.screen.print("Segment final")
        robot.straight(-560)  # Dernier segment
        robot.stop()
        wait(300)

        # --- Finalisation ---
        ev3.screen.clear()
        ev3.screen.print("Programme terminé")
        ev3.speaker.beep()
        wait(2000)
        robot.stop()
        
    except Exception as e:
        # Gestion globale des exceptions
        ev3.screen.clear()
        ev3.screen.print("Erreur:")
        ev3.screen.print(str(e))
        ev3.speaker.beep(frequency=200, duration=1000)  # Son d'erreur
        wait(5000)
        robot.stop()
        gripper_motor.stop()
