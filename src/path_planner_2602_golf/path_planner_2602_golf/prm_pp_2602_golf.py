#!/usr/bin/env python3
"""Probabilistic Roadmap (PRM) planner (ROS 2)

Este nodo procesa el mapa, infla obstáculos y construye un roadmap
usando muestreo aleatorio y árboles KD (KDTree) de SciPy.
"""

import math
import random
from collections import deque
from typing import List, Tuple, Optional, Dict

import numpy as np
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
#import tf2_ros

import heapq
from geometry_msgs.msg import Quaternion

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from nav_msgs.srv import GetPlan

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q

class PRMNode(Node):
    def __init__(self):
        super().__init__('prm_pp_2602_golf')

        # --- Parámetros Base ---
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('treat_unknown_as_obstacle', True)
        self.declare_parameter('inflate_radius', 0.25)

        # --- Parámetros PRM Específicos ---
        self.declare_parameter('density', 15.0)       # (N) Cantidad de puntos aleatorios a generar
        self.declare_parameter('k_neighbors', 3)         # (k) Máximo de vecinos a conectar por nodo
        self.declare_parameter('max_edge_dist', 2.0)     # Distancia máxima para conectar dos nodos (metros)

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_cb, qos_map)

        # Variables del Mapa
        self._map: Optional[OccupancyGrid] = None
        self._obstacles: Optional[np.ndarray] = None

        # Variables del Roadmap (Grafo PRM)
        self.prm_nodes_world: List[Tuple[float, float]] = []  # Coordenadas (x, y) en el mundo
        self.prm_nodes_grid: List[Tuple[int, int]] = []       # Índices (ix, iy) en la matriz
        self.prm_graph: Dict[int, List[Tuple[int, float]]] = {} # Lista de adyacencia 
        self.kdtree: Optional[KDTree] = None
        self.roadmap_built = False

        self.get_logger().info('PRM Planner Node inicializado. Esperando mapa...')

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        
        #self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        self.srv = self.create_service(GetPlan, 'get_prm_plan', self.plan_service_cb)
        self.get_logger().info('PRM listo. Esperando coordenadas en /get_prm_plan')

        qos_marker = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.graph_pub = self.create_publisher(MarkerArray, '/prm_roadmap', qos_marker)
        
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ---------------- FASE 1: PROCESAMIENTO DEL MAPA ----------------

    def map_cb(self, msg: OccupancyGrid):
        if self.roadmap_built:
            return  
        self.get_logger().info('Mapa recibido. Iniciando procesamiento e inflado...')
        self._map = msg
        W = msg.info.width
        H = msg.info.height
        res = msg.info.resolution

        grid = np.array(msg.data, dtype=np.int16).reshape((H, W))
        occ_th = self.get_parameter('occupied_threshold').get_parameter_value().integer_value
        unknown_as_obs = self.get_parameter('treat_unknown_as_obstacle').get_parameter_value().bool_value

        obstacles = (grid >= occ_th)
        if unknown_as_obs:
            obstacles = np.logical_or(obstacles, grid == -1)

        # Inflado usando Brushfire (Reutilizado de tu Dijkstra)
        dist_cells = self.compute_distance_to_obstacles(obstacles)
        #self.dist_cells = self.compute_distance_to_obstacles(obstacles)
        inflate_radius = float(self.get_parameter('inflate_radius').get_parameter_value().double_value)
        if inflate_radius > 1e-6:
            inflation_cells = int(math.ceil(inflate_radius / res))
            obstacles = np.logical_or(obstacles, dist_cells <= inflation_cells)

        self._obstacles = obstacles

        self.build_roadmap()

    def compute_distance_to_obstacles(self, obstacles: np.ndarray) -> np.ndarray:
        H, W = obstacles.shape
        INF = 1_000_000
        dist = np.full((H, W), INF, dtype=np.int32)
        q = deque()
        ys, xs = np.nonzero(obstacles)
        for y, x in zip(ys, xs):
            dist[y, x] = 0
            q.append((x, y))
        if not q:
            return dist
        nbr4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        while q:
            x, y = q.popleft()
            d = dist[y, x]
            nd = d + 1
            for dx, dy in nbr4:
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H and dist[ny, nx] > nd:
                    dist[ny, nx] = nd
                    q.append((nx, ny))
        return dist

    # ---------------- FASE 2: CONSTRUCCIÓN DEL ROADMAP (APRENDIZAJE) ----------------

    def build_roadmap(self):
        self.get_logger().info('Iniciando muestreo aleatorio (PRM)...')
        density = self.get_parameter('density').get_parameter_value().double_value
        k_neighbors = self.get_parameter('k_neighbors').get_parameter_value().integer_value
        max_dist = self.get_parameter('max_edge_dist').get_parameter_value().double_value
        
        info = self._map.info
        res = info.resolution
        W, H = info.width, info.height
        x0, y0 = info.origin.position.x, info.origin.position.y

        # --- EL CÁLCULO INTELIGENTE DEL NÚMERO DE MUESTRAS ---
        # 1. Contar cuántas celdas libres (no obstáculos) hay en la matriz
        free_cells = np.sum(self._obstacles == False)
        
        # 2. Calcular el área libre real en metros cuadrados
        free_area_m2 = free_cells * (res * res)
        
        
        # 4. Calcular el número óptimo de muestras
        num_samples = int(free_area_m2 * density)
        
        self.get_logger().info(
            f'Área libre detectada: {free_area_m2:.2f} m^2. '
            f'Generando {num_samples} nodos óptimos...'
        )
        # -----------------------------------------------------

        k_neighbors = self.get_parameter('k_neighbors').get_parameter_value().integer_value
        max_dist = self.get_parameter('max_edge_dist').get_parameter_value().double_value

        self.prm_nodes_grid = []
        self.prm_nodes_world = []

        # 1. Generar Nudos 
        while len(self.prm_nodes_grid) < num_samples:
            ix = random.randint(0, W - 1)
            iy = random.randint(0, H - 1)
            
            # Si el punto cae en espacio libre y no inflado, es un nodo válido
            if not self._obstacles[iy, ix]:
                self.prm_nodes_grid.append((ix, iy))
                # Guardar también sus coordenadas en metros para el KDTree
                x = x0 + (ix + 0.5) * res
                y = y0 + (iy + 0.5) * res
                self.prm_nodes_world.append((x, y))

        # 2. Construir KDTree para búsqueda ultrarrápida
        self.kdtree = KDTree(self.prm_nodes_world)
        self.prm_graph = {i: [] for i in range(num_samples)}

        self.get_logger().info(f'Conectando {num_samples} nodos a sus {k_neighbors} vecinos más cercanos...')

        # 3. Conectar vecinos validando colisiones (Bresenham)
        edges_added = 0
        for i, node_world in enumerate(self.prm_nodes_world):
            # Buscar k+1 vecinos porque el punto se encontrará a sí mismo a distancia 0
            distances, indices = self.kdtree.query(node_world, k=k_neighbors + 1, distance_upper_bound=max_dist)
            
            for d, neighbor_idx in zip(distances, indices):
                if neighbor_idx == i or neighbor_idx == len(self.prm_nodes_world):
                    continue # Es el mismo nodo o un vecino inválido devuelto por scipy
                
                # Verificar colisión con el mapa usando índices de cuadrícula
                ix1, iy1 = self.prm_nodes_grid[i]
                ix2, iy2 = self.prm_nodes_grid[neighbor_idx]
                
                if self.is_collision_free(ix1, iy1, ix2, iy2):
                    # Agregar arista bidireccional al grafo (nodo_destino, distancia_costo)
                    self.prm_graph[i].append((neighbor_idx, d))
                    self.prm_graph[neighbor_idx].append((i, d))
                    edges_added += 1

        self.get_logger().info(f'Grafo PRM completado con {edges_added} aristas válidas.')
        # Llamar a la Fase 4 para dibujar
        self.publish_roadmap_markers()
        self.roadmap_built = True
        

    # ---------------- VERIFICADOR DE COLISIONES (Algoritmo de Bresenham) ----------------
    
    def is_collision_free(self, x0: int, y0: int, x1: int, y1: int) -> bool:
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            # Si tocamos un obstáculo, la línea es inválida
            if self._obstacles[y0, x0]:
                return False
                
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return True
    # ---------------- FASE 3: CONSULTA Y BÚSQUEDA (QUERY PHASE) ----------------

    # def goal_cb(self, goal_msg: PoseStamped):
    #     if not self.prm_nodes_world:
    #         self.get_logger().warn('Grafo PRM no está listo aún.')
    #         return

    #     global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
    #     base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

    #     # 1. Obtener la posición del Start (Robot) usando TF
    #     try:
    #         trans = self.tf_buffer.lookup_transform(global_frame, base_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
    #         start_x = trans.transform.translation.x
    #         start_y = trans.transform.translation.y
    #     except Exception as e:
    #         self.get_logger().error(f'No se pudo obtener TF del robot: {e}')
    #         return

    #     goal_x = goal_msg.pose.position.x
    #     goal_y = goal_msg.pose.position.y

    #     self.get_logger().info(f'Buscando ruta PRM hacia ({goal_x:.2f}, {goal_y:.2f})')

    #     # 2. Conectar Start y Goal al grafo
    #     start_node_idx = self.connect_to_graph(start_x, start_y)
    #     goal_node_idx = self.connect_to_graph(goal_x, goal_y)

    #     if start_node_idx is None or goal_node_idx is None:
    #         self.get_logger().warn('Imposible conectar Start/Goal al PRM. ¿Estás dentro de un obstáculo?')
    #         return

    #     # 3. Correr Dijkstra sobre el Grafo PRM
    #     path_indices = self.dijkstra_on_graph(start_node_idx, goal_node_idx)

    #     if not path_indices:
    #         self.get_logger().warn('No hay ruta posible en este roadmap.')
    #         return

    #     # 4. Publicar la ruta final
    #     self.publish_path(path_indices, (start_x, start_y), (goal_x, goal_y), global_frame)

    def plan_service_cb(self, request, response):
        if not self.prm_nodes_world:
            self.get_logger().warn('Grafo PRM no está listo aún.')
            return response

        # 1. Extraemos explícitamente el Inicio y el Fin de la petición del Manager
        start_x = request.start.pose.position.x
        start_y = request.start.pose.position.y
        goal_x = request.goal.pose.position.x
        goal_y = request.goal.pose.position.y

        self.get_logger().info(f'Calculando tramo: ({start_x:.2f}, {start_y:.2f}) -> ({goal_x:.2f}, {goal_y:.2f})')

        # 2. Conectar al grafo
        start_node_idx = self.connect_to_graph(start_x, start_y)
        goal_node_idx = self.connect_to_graph(goal_x, goal_y)

        if start_node_idx is None or goal_node_idx is None:
            self.get_logger().warn('Imposible conectar Start/Goal al PRM.')
            return response

        # 3. Ejecutar Dijkstra
        path_indices = self.dijkstra_on_graph(start_node_idx, goal_node_idx)

        if not path_indices:
            self.get_logger().warn('No hay ruta posible para este tramo.')
            return response

        # 4. Construir la ruta y empaquetarla en la respuesta
        global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        path_msg = self.create_path_msg(path_indices, (start_x, start_y), (goal_x, goal_y), global_frame)
        
        # Seguimos publicando en el tópico para que puedas ver el proceso en RViz2
        self.path_pub.publish(path_msg) 
        
        # Devolvemos la ruta al Race Manager
        response.plan = path_msg
        return response

    def connect_to_graph(self, x: float, y: float) -> Optional[int]:
        
        info = self._map.info
        res = info.resolution
        x0, y0 = info.origin.position.x, info.origin.position.y
        
        ix = int((x - x0) / res)
        iy = int((y - y0) / res)
        
        # Consultar los K nodos más cercanos geográficamente
        distances, indices = self.kdtree.query((x, y), k=15)
        
        # Si k=1, scipy devuelve escalares en vez de listas, esto lo arregla:
        if isinstance(indices, int) or isinstance(indices, np.integer):
            indices = [indices]
            
        for idx in indices:
            if idx >= len(self.prm_nodes_world):
                continue
            n_ix, n_iy = self.prm_nodes_grid[idx]
            if self.is_collision_free(ix, iy, n_ix, n_iy):
                return int(idx)
        return None

    def dijkstra_on_graph(self, start_idx: int, goal_idx: int) -> List[int]:
    
        num_nodes = len(self.prm_nodes_world)
        dist = {i: float('inf') for i in range(num_nodes)}
        parent = {i: -1 for i in range(num_nodes)}
        
        dist[start_idx] = 0.0
        pq = [(0.0, start_idx)]
        
        while pq:
            current_dist, u = heapq.heappop(pq)
            
            if u == goal_idx:
                break
                
            if current_dist > dist[u]:
                continue
                
            for v, weight in self.prm_graph[u]:
                alt = current_dist + weight
                if alt < dist[v]:
                    dist[v] = alt
                    parent[v] = u
                    heapq.heappush(pq, (alt, v))
                    
        if dist[goal_idx] == float('inf'):
            return []
            
        path = []
        curr = goal_idx
        while curr != -1:
            path.append(curr)
            curr = parent[curr]
        path.reverse()
        return path

    def create_path_msg(self, path_indices: List[int], start_pt: Tuple[float, float], goal_pt: Tuple[float, float], frame_id: str):
        
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Start -> Nodos del PRM -> Goal
        full_points = [start_pt] + [self.prm_nodes_world[i] for i in path_indices] + [goal_pt]

        last_yaw = 0.0
        for i, (px, py) in enumerate(full_points):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            
            if i + 1 < len(full_points):
                nx, ny = full_points[i+1]
                last_yaw = math.atan2(ny - py, nx - px)
            
            pose.pose.orientation = yaw_to_quaternion(last_yaw)
            path_msg.poses.append(pose)

        #self.path_pub.publish(path_msg)
        #self.get_logger().info(f'Ruta enviada al Tracker: {len(path_msg.poses)} waypoints.')
        return path_msg

    # ---------------- FASE 4: VISUALIZACIÓN DEL GRAFO ----------------

    def publish_roadmap_markers(self):
        
        marker_array = MarkerArray()
        
        # 1. Marcador para los Nodos (Puntos Azules)
        nodes_marker = Marker()
        nodes_marker.header.frame_id = self.get_parameter('global_frame').get_parameter_value().string_value
        nodes_marker.header.stamp = self.get_clock().now().to_msg()
        nodes_marker.ns = 'prm_nodes'
        nodes_marker.id = 0
        nodes_marker.type = Marker.SPHERE_LIST
        nodes_marker.action = Marker.ADD
        nodes_marker.scale.x = 0.05  # Tamaño de los puntos
        nodes_marker.scale.y = 0.05
        nodes_marker.scale.z = 0.05
        # Color azul brillante
        nodes_marker.color.a = 1.0
        nodes_marker.color.r = 0.0
        nodes_marker.color.g = 0.5
        nodes_marker.color.b = 1.0

        for (x, y) in self.prm_nodes_world:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.0
            nodes_marker.points.append(p)
            
        marker_array.markers.append(nodes_marker)

        # 2. Marcador para las Aristas (Líneas Grises)
        edges_marker = Marker()
        edges_marker.header = nodes_marker.header
        edges_marker.ns = 'prm_edges'
        edges_marker.id = 1
        edges_marker.type = Marker.LINE_LIST
        edges_marker.action = Marker.ADD
        edges_marker.scale.x = 0.05  # Grosor de la línea
        # Color gris semitransparente para no saturar la vista
        edges_marker.color.a = 1.0
        edges_marker.color.r = 1.0
        edges_marker.color.g = 0.5
        edges_marker.color.b = 0.5

        # Usar un set para no dibujar la misma línea de ida y de vuelta
        drawn_edges = set()
        
        for u, neighbors in self.prm_graph.items():
            for v, _ in neighbors:
                edge = tuple(sorted((u, v)))
                if edge not in drawn_edges:
                    drawn_edges.add(edge)
                    
                    p1 = Point()
                    p1.x, p1.y, p1.z = self.prm_nodes_world[u][0], self.prm_nodes_world[u][1], 0.0
                    
                    p2 = Point()
                    p2.x, p2.y, p2.z = self.prm_nodes_world[v][0], self.prm_nodes_world[v][1], 0.0
                    
                    edges_marker.points.append(p1)
                    edges_marker.points.append(p2)
                    
        marker_array.markers.append(edges_marker)
        
        self.graph_pub.publish(marker_array)
        self.get_logger().info('Telaraña PRM (MarkerArray) enviada a RViz2.')


def main(args=None):
    rclpy.init(args=args)
    node = PRMNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()