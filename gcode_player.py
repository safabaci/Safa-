#!/usr/bin/env python3
"""
Interpréteur G-code réel pour DMG MORI 5 axes
Supporte : G00 G01 G02 G03 G17 G20 G21 G90 G91
           M03 M05 M30
           Axes : X Y Z A C
           Paramètres : F (vitesse) S (broche)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
import re
import os
import sys

# ─────────────────────────────────────────────────────
# CONSTANTES DE CONVERSION
# ─────────────────────────────────────────────────────
MM_TO_M   = 0.001      # G-code en mm → ROS en mètres
DEG_TO_RAD = math.pi / 180.0

# Limites physiques machine (en mètres)
LIMITES = {
    'X': (-0.45,  0.45),
    'Y': (-0.35,  0.35),
    'Z': ( 0.00,  0.70),
    'A': (-1.22,  1.22),   # ±70°
    'C': (-math.pi, math.pi),
}

class GCodePlayer(Node):

    def __init__(self, fichier_gcode):
        super().__init__('gcode_player')

        # Publishers
        self.pub_joints  = self.create_publisher(
            JointState,  '/joint_states',      10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/gcode_markers',      10)

        # État machine
        self.pos = {'X':0.0, 'Y':0.0, 'Z':50.0,
                    'A':0.0, 'C':0.0}
        self.vitesse      = 500.0   # mm/min
        self.broche_on    = False
        self.mode_absolu  = True    # G90
        self.unite_mm     = True    # G21
        self.plan         = 'XY'    # G17

        # Trajectoire pour affichage
        self.trajectoire  = []
        self.couleur_traj = (0.0, 1.0, 0.0)  # vert par défaut

        # Charger et parser le G-code
        self.commandes = self.parser_gcode(fichier_gcode)
        self.idx_cmd   = 0
        self.en_mouvement = False
        self.pos_cible    = dict(self.pos)
        self.pos_ros      = {
            'joint_x': 0.0,
            'joint_y': 0.0,
            'joint_z': 0.0,
            'joint_a': 0.0,
            'joint_c': 0.0,
        }

        self.get_logger().info(
            f'G-code chargé : {len(self.commandes)} commandes')
        self.afficher_programme()

        # Timer principal 50Hz
        self.timer = self.create_timer(0.02, self.step)

    # ─────────────────────────────────────────────────
    # PARSER G-CODE
    # ─────────────────────────────────────────────────
    def parser_gcode(self, fichier):
        commandes = []
        try:
            with open(fichier, 'r') as f:
                lignes = f.readlines()
        except FileNotFoundError:
            self.get_logger().error(f'Fichier non trouvé : {fichier}')
            return []

        for num, ligne in enumerate(lignes, 1):
            # Supprimer commentaires
            ligne = re.sub(r';.*', '', ligne).strip().upper()
            if not ligne:
                continue

            cmd = {'ligne': num, 'raw': ligne,
                   'G': None, 'M': None,
                   'X': None, 'Y': None, 'Z': None,
                   'A': None, 'C': None,
                   'I': None, 'J': None,
                   'F': None, 'S': None}

            # Extraire tous les mots G-code
            tokens = re.findall(
                r'([GMXYZACIFJSF])(-?\d+\.?\d*)', ligne)

            for lettre, valeur in tokens:
                if lettre in ('G', 'M'):
                    cmd[lettre] = int(float(valeur))
                elif lettre in ('X','Y','Z','A','C',
                                'I','J','F','S'):
                    cmd[lettre] = float(valeur)

            # Garder seulement les lignes utiles
            if any(cmd[k] is not None
                   for k in ('G','M','X','Y','Z','A','C')):
                commandes.append(cmd)

        return commandes

    def afficher_programme(self):
        self.get_logger().info(
            '╔════════════════════════════════════╗')
        self.get_logger().info(
            '║     LECTEUR G-CODE DMG MORI 5X     ║')
        self.get_logger().info(
            '╠════════════════════════════════════╣')
        for i, cmd in enumerate(self.commandes[:8]):
            self.get_logger().info(
                f'║  {i+1:02d}. {cmd["raw"][:32]:<32} ║')
        if len(self.commandes) > 8:
            self.get_logger().info(
                f'║  ... {len(self.commandes)-8} autres commandes    ║')
        self.get_logger().info(
            '╚════════════════════════════════════╝')

    # ─────────────────────────────────────────────────
    # CONVERSION COORDONNÉES G-CODE → ROS
    # ─────────────────────────────────────────────────
    def gcode_to_ros(self):
        """Convertit positions G-code (mm/deg) en joints ROS (m/rad)"""
        x = max(LIMITES['X'][0],
                min(LIMITES['X'][1],
                    self.pos['X'] * MM_TO_M))
        y = max(LIMITES['Y'][0],
                min(LIMITES['Y'][1],
                    self.pos['Y'] * MM_TO_M))
        z = max(LIMITES['Z'][0],
                min(LIMITES['Z'][1],
                    (self.pos['Z'] - 0) * MM_TO_M))
        a = max(LIMITES['A'][0],
                min(LIMITES['A'][1],
                    self.pos['A'] * DEG_TO_RAD))
        c = self.pos['C'] * DEG_TO_RAD

        self.pos_ros = {
            'joint_x':  x,
            'joint_y':  y,
            'joint_z':  max(0.0, min(0.70, z)),
            'joint_a':  a,
            'joint_c':  c,
        }

    # ─────────────────────────────────────────────────
    # PUBLICATION
    # ─────────────────────────────────────────────────
    def publier_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = list(self.pos_ros.keys())
        msg.position     = list(self.pos_ros.values())
        msg.velocity     = [0.0] * 5
        msg.effort       = [0.0] * 5
        self.pub_joints.publish(msg)

    def publier_trajectoire(self):
        array = MarkerArray()

        if len(self.trajectoire) > 1:
            ligne = Marker()
            ligne.header.frame_id = 'world'
            ligne.header.stamp    = self.get_clock().now().to_msg()
            ligne.ns   = 'gcode_path'
            ligne.id   = 0
            ligne.type = Marker.LINE_STRIP
            ligne.action = Marker.ADD
            ligne.scale.x = 0.004

            for pt in self.trajectoire[-500:]:
                p = Point()
                p.x, p.y, p.z = pt['pos']
                ligne.points.append(p)
                c = ColorRGBA()
                c.r = pt['couleur'][0]
                c.g = pt['couleur'][1]
                c.b = pt['couleur'][2]
                c.a = 0.9
                ligne.colors.append(c)
            array.markers.append(ligne)

        # Afficher position outil courante
        outil = Marker()
        outil.header.frame_id = 'world'
        outil.header.stamp    = self.get_clock().now().to_msg()
        outil.ns   = 'outil_pos'
        outil.id   = 1
        outil.type = Marker.SPHERE
        outil.action = Marker.ADD
        outil.pose.position.x = self.pos['X'] * MM_TO_M
        outil.pose.position.y = self.pos['Y'] * MM_TO_M
        outil.pose.position.z = max(0.0,
            (50 - self.pos['Z']) * MM_TO_M)
        outil.pose.orientation.w = 1.0
        outil.scale.x = 0.015
        outil.scale.y = 0.015
        outil.scale.z = 0.015
        outil.color.r = 1.0
        outil.color.g = 1.0
        outil.color.b = 0.0
        outil.color.a = 1.0
        array.markers.append(outil)

        # Label commande courante
        if self.idx_cmd < len(self.commandes):
            txt = Marker()
            txt.header.frame_id = 'world'
            txt.header.stamp    = self.get_clock().now().to_msg()
            txt.ns   = 'gcode_label'
            txt.id   = 2
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = -0.5
            txt.pose.position.y = -0.5
            txt.pose.position.z =  1.5
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.06
            cmd = self.commandes[self.idx_cmd]
            txt.text = (
                f"Ligne {cmd['ligne']}: {cmd['raw']}\n"
                f"X:{self.pos['X']:6.1f} "
                f"Y:{self.pos['Y']:6.1f} "
                f"Z:{self.pos['Z']:6.1f}\n"
                f"A:{self.pos['A']:5.1f}° "
                f"C:{self.pos['C']:5.1f}° "
                f"F:{self.vitesse:.0f}mm/min"
            )
            txt.color.r = 0.2
            txt.color.g = 1.0
            txt.color.b = 0.8
            txt.color.a = 1.0
            array.markers.append(txt)

        self.pub_markers.publish(array)

    # ─────────────────────────────────────────────────
    # EXÉCUTION DES COMMANDES G-CODE
    # ─────────────────────────────────────────────────
    def executer_commande(self, cmd):
        """Exécute une commande G-code et retourne True si mouvement"""

        # ── Codes M ──────────────────────────────────
        if cmd['M'] == 3:
            self.broche_on = True
            self.couleur_traj = (0.0, 1.0, 0.0)  # vert
            self.get_logger().info(
                f'  M03 — Broche ON ({self.vitesse:.0f} tr/min)')
            return False

        if cmd['M'] == 5:
            self.broche_on = False
            self.get_logger().info('  M05 — Broche OFF')
            return False

        if cmd['M'] == 30:
            self.get_logger().info('')
            self.get_logger().info(
                '╔══════════════════════════════════╗')
            self.get_logger().info(
                '║   ✅  FIN DE PROGRAMME G-CODE    ║')
            self.get_logger().info(
                '╚══════════════════════════════════╝')
            return False

        # ── Paramètres F et S ────────────────────────
        if cmd['F'] is not None:
            self.vitesse = cmd['F']
        if cmd['S'] is not None:
            self.get_logger().info(
                f'  S{cmd["S"]:.0f} — Broche {cmd["S"]:.0f} tr/min')

        # ── Codes G modaux ───────────────────────────
        if cmd['G'] == 17:
            self.plan = 'XY'
            return False
        if cmd['G'] == 20:
            self.unite_mm = False
            return False
        if cmd['G'] == 21:
            self.unite_mm = True
            return False
        if cmd['G'] == 90:
            self.mode_absolu = True
            return False
        if cmd['G'] == 91:
            self.mode_absolu = False
            return False

        # ── Calcul position cible ────────────────────
        cible = dict(self.pos)

        for axe in ('X', 'Y', 'Z', 'A', 'C'):
            if cmd[axe] is not None:
                val = cmd[axe]
                if not self.unite_mm and axe in ('X','Y','Z'):
                    val *= 25.4   # pouces → mm
                if self.mode_absolu:
                    cible[axe] = val
                else:
                    cible[axe] = self.pos[axe] + val

        # ── G00 : déplacement rapide ──────────────────
        if cmd['G'] == 0:
            self.couleur_traj = (0.5, 0.5, 0.5)  # gris
            self.pos_cible = cible
            self.get_logger().info(
                f"  G00 → X{cible['X']:.1f} "
                f"Y{cible['Y']:.1f} "
                f"Z{cible['Z']:.1f}")
            return True

        # ── G01 : interpolation linéaire ─────────────
        if cmd['G'] == 1:
            self.couleur_traj = (0.0, 0.8, 1.0)  # bleu
            self.pos_cible = cible
            self.get_logger().info(
                f"  G01 → X{cible['X']:.1f} "
                f"Y{cible['Y']:.1f} "
                f"Z{cible['Z']:.1f} "
                f"F{self.vitesse:.0f}")
            return True

        # ── G02/G03 : interpolation circulaire ───────
        if cmd['G'] in (2, 3):
            sens = 'CW' if cmd['G'] == 2 else 'CCW'
            self.couleur_traj = (1.0, 0.5, 0.0)  # orange
            # Calculer les points de l'arc
            cx = self.pos['X'] + (cmd['I'] or 0)
            cy = self.pos['Y'] + (cmd['J'] or 0)
            r  = math.sqrt(
                (self.pos['X']-cx)**2 +
                (self.pos['Y']-cy)**2)
            a0 = math.atan2(
                self.pos['Y']-cy, self.pos['X']-cx)
            a1 = math.atan2(
                cible['Y']-cy,    cible['X']-cx)

            # Arc complet si même point départ/arrivée
            if abs(a1 - a0) < 0.01:
                a1 = a0 + (
                    -2*math.pi if cmd['G']==2
                    else 2*math.pi)

            # Générer points intermédiaires de l'arc
            n_pts = max(20, int(abs(a1-a0) * r / 2))
            for i in range(n_pts + 1):
                t   = i / n_pts
                ang = a0 + (a1 - a0) * t
                self.trajectoire.append({
                    'pos': (cx + r*math.cos(ang)) * MM_TO_M,
                    'couleur': self.couleur_traj
                })
                # Hack: on stocke comme tuple
                self.trajectoire[-1] = {
                    'pos': (
                        (cx + r*math.cos(ang)) * MM_TO_M,
                        (cy + r*math.sin(ang)) * MM_TO_M,
                        max(0.0,
                            (50-self.pos['Z']) * MM_TO_M)
                    ),
                    'couleur': self.couleur_traj
                }

            self.pos_cible = cible
            self.get_logger().info(
                f"  G0{cmd['G']} {sens} arc "
                f"R={r:.1f}mm → "
                f"X{cible['X']:.1f} "
                f"Y{cible['Y']:.1f}")
            return True

        # Axe seul (A ou C sans G)
        if cmd['G'] is None and any(
                cmd[k] is not None for k in ('A','C')):
            self.couleur_traj = (1.0, 0.2, 1.0)  # violet
            self.pos_cible = cible
            self.get_logger().info(
                f"  Rotation → A{cible['A']:.1f}° "
                f"C{cible['C']:.1f}°")
            return True

        return False

    # ─────────────────────────────────────────────────
    # INTERPOLATION MOUVEMENT
    # ─────────────────────────────────────────────────
    def deplacer_vers_cible(self):
        """Déplace progressivement vers pos_cible"""
        # Vitesse en m/s selon F (mm/min)
        vitesse_ms = (self.vitesse * MM_TO_M) / 60.0
        vitesse_ms = max(0.0005,
                         min(0.005, vitesse_ms))

        atteint = True
        for axe in ('X', 'Y', 'Z', 'A', 'C'):
            diff = self.pos_cible[axe] - self.pos[axe]
            if abs(diff) > 0.1:
                atteint = False
                pas_max = vitesse_ms / MM_TO_M * 2
                pas     = min(abs(diff), pas_max)
                self.pos[axe] += pas * (
                    1 if diff > 0 else -1)

        # Enregistrer point trajectoire
        pt = (
            self.pos['X'] * MM_TO_M,
            self.pos['Y'] * MM_TO_M,
            max(0.0, (50-self.pos['Z']) * MM_TO_M)
        )
        if (not self.trajectoire or
                abs(pt[0]-self.trajectoire[-1]['pos'][0])
                > 0.003 or
                abs(pt[1]-self.trajectoire[-1]['pos'][1])
                > 0.003):
            self.trajectoire.append({
                'pos': pt,
                'couleur': self.couleur_traj
            })

        return atteint

    # ─────────────────────────────────────────────────
    # BOUCLE PRINCIPALE
    # ─────────────────────────────────────────────────
    def step(self):
        if self.en_mouvement:
            if self.deplacer_vers_cible():
                self.en_mouvement = False
                self.idx_cmd += 1
        else:
            # Passer à la prochaine commande
            while self.idx_cmd < len(self.commandes):
                cmd = self.commandes[self.idx_cmd]
                if self.executer_commande(cmd):
                    self.en_mouvement = True
                    break
                else:
                    self.idx_cmd += 1
            else:
                # Fin du programme — attendre
                pass

        self.gcode_to_ros()
        self.publier_joints()
        self.publier_trajectoire()


def main(args=None):
    rclpy.init(args=args)

    # Chemin du fichier G-code
    fichier = os.path.expanduser(
        '~/dmg_mori_ws/src/dmg_mori_5axis/'
        'gcode/piece_demo.ngc')

    if len(sys.argv) > 1:
        fichier = sys.argv[1]

    node = GCodePlayer(fichier)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
