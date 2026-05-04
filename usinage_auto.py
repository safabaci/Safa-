#!/usr/bin/env python3
"""
Simulation d'usinage réaliste - DMG MORI 5 axes
La machine usine automatiquement une pièce cylindrique
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class UsinageAuto(Node):

    def __init__(self):
        super().__init__('usinage_auto')

        # Publier les positions des joints
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Position courante de chaque axe
        self.pos = {
            'joint_x': 0.0,   # -0.45 à +0.45 m
            'joint_y': 0.0,   # -0.35 à +0.35 m
            'joint_z': 0.0,   #  0.0  à +0.70 m
            'joint_a': 0.0,   # -1.22 à +1.22 rad
            'joint_c': 0.0,   # -3.14 à +3.14 rad
        }

        # Démarrer la simulation
        self.etape = 0
        self.t = 0.0
        self.timer = self.create_timer(0.05, self.cycle_usinage)

        self.get_logger().info('═══════════════════════════════════')
        self.get_logger().info('  SIMULATION USINAGE DMG MORI 5X  ')
        self.get_logger().info('═══════════════════════════════════')
        self.get_logger().info('Démarrage du cycle automatique...')

    def publier(self):
        """Publie l'état des joints"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.pos.keys())
        msg.position = list(self.pos.values())
        msg.velocity = [0.0] * 5
        msg.effort = [0.0] * 5
        self.pub.publish(msg)

    def interpoler(self, cible, vitesse=0.002):
        """Déplace tous les axes vers la cible progressivement"""
        atteint = True
        for axe, val_cible in cible.items():
            diff = val_cible - self.pos[axe]
            if abs(diff) > 0.001:
                atteint = False
                pas = min(abs(diff), vitesse) * (1 if diff > 0 else -1)
                self.pos[axe] += pas
        return atteint

    def cycle_usinage(self):
        """Cycle complet d'usinage d'une pièce cylindrique"""
        self.t += 0.05

        # ─────────────────────────────────────────
        # ÉTAPE 0 : Position initiale (HOME)
        # ─────────────────────────────────────────
        if self.etape == 0:
            self.get_logger().info('► Étape 0 : Retour position HOME...')
            cible = {'joint_x': 0.0, 'joint_y': 0.0,
                     'joint_z': 0.0, 'joint_a': 0.0, 'joint_c': 0.0}
            if self.interpoler(cible, 0.004):
                self.etape = 1
                self.t = 0.0
                self.get_logger().info('✓ HOME atteint')

        # ─────────────────────────────────────────
        # ÉTAPE 1 : Chargement pièce brute
        # Plateau tourne pour se positionner
        # ─────────────────────────────────────────
        elif self.etape == 1:
            self.get_logger().info('► Étape 1 : Chargement pièce - plateau en position...')
            cible = {'joint_x': 0.0, 'joint_y': 0.2,
                     'joint_z': 0.0, 'joint_a': 0.0, 'joint_c': 0.0}
            if self.interpoler(cible, 0.003):
                self.etape = 2
                self.t = 0.0
                self.get_logger().info('✓ Pièce chargée - Début usinage !')

        # ─────────────────────────────────────────
        # ÉTAPE 2 : Approche outil sur la pièce
        # Z descend lentement
        # ─────────────────────────────────────────
        elif self.etape == 2:
            self.get_logger().info('► Étape 2 : Approche outil...')
            cible = {'joint_x': 0.0, 'joint_y': 0.2,
                     'joint_z': 0.35, 'joint_a': 0.0, 'joint_c': 0.0}
            if self.interpoler(cible, 0.002):
                self.etape = 3
                self.t = 0.0
                self.get_logger().info('✓ Outil en contact avec la pièce')

        # ─────────────────────────────────────────
        # ÉTAPE 3 : Passe d'ébauche
        # X balaie de gauche à droite
        # ─────────────────────────────────────────
        elif self.etape == 3:
            self.get_logger().info('► Étape 3 : Passe ébauche (axe X)...')
            cible = {'joint_x': 0.35, 'joint_y': 0.2,
                     'joint_z': 0.40, 'joint_a': 0.0, 'joint_c': 0.0}
            if self.interpoler(cible, 0.0015):
                self.etape = 4
                self.t = 0.0
                self.get_logger().info('✓ Passe ébauche 1 terminée')

        # ─────────────────────────────────────────
        # ÉTAPE 4 : Retour + décalage Y
        # ─────────────────────────────────────────
        elif self.etape == 4:
            cible = {'joint_x': -0.35, 'joint_y': 0.1,
                     'joint_z': 0.40, 'joint_a': 0.0, 'joint_c': 0.0}
            if self.interpoler(cible, 0.0015):
                self.etape = 5
                self.t = 0.0
                self.get_logger().info('✓ Passe ébauche 2 terminée')

        # ─────────────────────────────────────────
        # ÉTAPE 5 : Rotation plateau C (360°)
        # Simulation tournage extérieur
        # ─────────────────────────────────────────
        elif self.etape == 5:
            self.get_logger().info('► Étape 5 : Rotation plateau C - tournage...')
            # Rotation continue du plateau
            self.pos['joint_c'] += 0.03
            self.pos['joint_x'] = 0.0
            self.pos['joint_y'] = 0.15
            self.pos['joint_z'] = 0.38

            if self.pos['joint_c'] >= math.pi * 2:
                self.pos['joint_c'] = math.pi * 2
                self.etape = 6
                self.t = 0.0
                self.get_logger().info('✓ Tour complet effectué')

        # ─────────────────────────────────────────
        # ÉTAPE 6 : Inclinaison berceau A
        # Usinage face inclinée (5 axes simultanés)
        # ─────────────────────────────────────────
        elif self.etape == 6:
            self.get_logger().info('► Étape 6 : Usinage 5 axes simultanés...')
            # Mouvement 5 axes : X+Z+A+C en même temps
            angle = self.t * 0.8
            self.pos['joint_a'] = math.sin(angle) * 0.6
            self.pos['joint_c'] += 0.02
            self.pos['joint_x'] = math.cos(angle * 0.5) * 0.25
            self.pos['joint_z'] = 0.30 + math.sin(angle * 0.3) * 0.08

            if self.t > 8.0:
                self.etape = 7
                self.t = 0.0
                self.get_logger().info('✓ Usinage 5 axes terminé')

        # ─────────────────────────────────────────
        # ÉTAPE 7 : Passes de finition
        # Cercles concentriques sur le plateau
        # ─────────────────────────────────────────
        elif self.etape == 7:
            self.get_logger().info('► Étape 7 : Passes de finition...')
            rayon = 0.15 + self.t * 0.01
            self.pos['joint_x'] = math.cos(self.t * 1.5) * min(rayon, 0.30)
            self.pos['joint_y'] = math.sin(self.t * 1.5) * min(rayon, 0.25) + 0.1
            self.pos['joint_z'] = 0.42
            self.pos['joint_a'] = 0.0
            self.pos['joint_c'] += 0.015

            if self.t > 10.0:
                self.etape = 8
                self.t = 0.0
                self.get_logger().info('✓ Finition terminée - pièce usinée !')

        # ─────────────────────────────────────────
        # ÉTAPE 8 : Dégagement et retour HOME
        # ─────────────────────────────────────────
        elif self.etape == 8:
            self.get_logger().info('► Étape 8 : Dégagement outil...')
            cible = {'joint_x': 0.0, 'joint_y': 0.0,
                     'joint_z': 0.0, 'joint_a': 0.0, 'joint_c': 0.0}
            if self.interpoler(cible, 0.003):
                self.etape = 9
                self.get_logger().info('✓ Dégagement terminé')

        # ─────────────────────────────────────────
        # ÉTAPE 9 : Fin de cycle
        # ─────────────────────────────────────────
        elif self.etape == 9:
            self.get_logger().info('')
            self.get_logger().info('══════════════════════════════════')
            self.get_logger().info('  ✅ PIÈCE USINÉE AVEC SUCCÈS !   ')
            self.get_logger().info('  Nouveau cycle dans 5 secondes...')
            self.get_logger().info('══════════════════════════════════')
            time.sleep(5)
            self.etape = 0  # Recommencer le cycle

        self.publier()


def main(args=None):
    rclpy.init(args=args)
    node = UsinageAuto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt de la simulation.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
