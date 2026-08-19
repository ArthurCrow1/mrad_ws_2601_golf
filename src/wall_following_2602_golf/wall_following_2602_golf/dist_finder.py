#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
import math

class DistFinder(Node):
    def __init__(self):
        super().__init__('dist_finder')
        
        # Declarar la distancia deseada como parámetro (Trajd)
        # Esto permite que el Launch file cambie el valor de 0.5 a otra cosa
        self.declare_parameter('trajd', 0.5)
        
        self.L = 0.5      # Lookahead: Distancia de proyección a futuro (0.5 metros)
        self.theta_rad = math.radians(45.0) 
        
        # Suscribirse a /scan
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publicar el error
        self.error_pub = self.create_publisher(Float32, '/error', 10)
        
        self.get_logger().info("Nodo Dist Finder (Percepción Geométrica) iniciado con Trajd paramétrico.")

    def getRange(self, msg, angle_rad):
        """
        Encuentra el índice correcto en el arreglo y retorna la distancia.
        """
        index = int((angle_rad - msg.angle_min) / msg.angle_increment)
        
        # Proteger contra índices fuera de rango (Saturación)
        index = max(0, min(index, len(msg.ranges) - 1))
        
        dist = msg.ranges[index]
        
        # Manejar lecturas infinitas o erróneas del simulador
        if math.isinf(dist) or math.isnan(dist) or dist == 0.0:
            return float('inf')
            
        return dist

    def scan_callback(self, msg):
        """
        Cada vez que llega un LaserScan, se invoca este callback.
        """
        # Paso 1: Elegir 2 rayos en el lado derecho.
        angle_b = -math.pi / 2.0                # Rayo a 0 grados respecto a la derecha (-90 front)
        angle_a = angle_b + self.theta_rad      # Rayo a theta grados hacia adelante
        
        # Paso 2: Llamar a getRange para obtener 'a' y 'b'
        a = self.getRange(msg, angle_a)
        b = self.getRange(msg, angle_b)
        
        # Si el sensor no ve la pared, no publicamos error
        if math.isinf(a) or math.isinf(b):
            return
            
        # Paso 3: Calcular alpha, AB y CD
        numerator = a * math.cos(self.theta_rad) - b
        denominator = a * math.sin(self.theta_rad)
        
        alpha = math.atan2(numerator, denominator)
        
        AB = b * math.cos(alpha)
        CD = AB + (self.L * math.sin(alpha))
        
        # Paso 4: Calcular el error e(t) = Trajd - CD
        # AQUÍ ES DONDE LEEMOS EL PARÁMETRO EN TIEMPO REAL
        trajd = self.get_parameter('trajd').value
        error = trajd - CD
        
        # Paso 5: Construir y publicar el mensaje
        error_msg = Float32()
        error_msg.data = float(error)
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DistFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()