#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math, os

MESHES = os.path.expanduser("~/dmg_mori_ws/src/dmg_mori_5axis/meshes/")

PIECES = [
    {"fichier": "piece_brute.stl",     "label": "Piece brute",   "couleur": (0.65,0.60,0.55)},
    {"fichier": "piece_ebauche.stl",   "label": "Ebauche",       "couleur": (0.58,0.55,0.52)},
    {"fichier": "piece_semifinie.stl", "label": "Semi-finition", "couleur": (0.72,0.72,0.75)},
    {"fichier": "piece_finie.stl",     "label": "Piece finie !", "couleur": (0.88,0.88,0.92)},
]

class UsinageSTL(Node):
    def __init__(self):
        super().__init__("usinage_stl")
        self.pub_joints  = self.create_publisher(JointState,  "/joint_states", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/piece_stl",    10)

        self.pos = {"joint_x":0.0,"joint_y":0.0,"joint_z":0.0,
                    "joint_a":0.0,"joint_c":0.0}
        self.phase = 0
        self.etape = 0
        self.t     = 0.0
        self.traj  = []
        self.timer = self.create_timer(0.02, self.cycle)
        self.get_logger().info("=== USINAGE STL DEMARRE ===")

    def pub_j(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(self.pos.keys())
        msg.position = list(self.pos.values())
        msg.velocity = [0.0]*5
        msg.effort   = [0.0]*5
        self.pub_joints.publish(msg)

    def pub_m(self):
        arr = MarkerArray()
        p = PIECES[self.phase]

        # Piece STL
        m = Marker()
        m.header.frame_id = "link_c"
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = "piece"; m.id = 0
        m.type   = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = "file://" + MESHES + p["fichier"]
        m.mesh_use_embedded_materials = False
        m.pose.position.z    = 0.06
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        r,g,b = p["couleur"]
        m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=1.0
        arr.markers.append(m)

        # Label
        lb = Marker()
        lb.header.frame_id = "link_c"
        lb.header.stamp    = self.get_clock().now().to_msg()
        lb.ns="label"; lb.id=1
        lb.type   = Marker.TEXT_VIEW_FACING
        lb.action = Marker.ADD
        lb.pose.position.z    = 0.30
        lb.pose.orientation.w = 1.0
        lb.scale.z = 0.05
        lb.text    = p["label"]
        lb.color.r=1.0; lb.color.g=0.9; lb.color.b=0.2; lb.color.a=1.0
        arr.markers.append(lb)

        # Copeaux
        if self.phase >= 1:
            for i in range(10):
                c = Marker()
                c.header.frame_id = "link_c"
                c.header.stamp    = self.get_clock().now().to_msg()
                c.ns="cop"; c.id=10+i
                c.type=Marker.SPHERE; c.action=Marker.ADD
                ang = i*math.pi/5 + self.t*0.15
                ray = 0.035 + (i%3)*0.008
                c.pose.position.x = math.cos(ang)*ray
                c.pose.position.y = math.sin(ang)*ray
                c.pose.position.z = 0.065
                c.pose.orientation.w = 1.0
                c.scale.x=c.scale.y=c.scale.z=0.006
                c.color.r=0.8; c.color.g=0.65; c.color.b=0.35; c.color.a=0.9
                arr.markers.append(c)

        # Trajectoire
        if len(self.traj) > 1:
            ln = Marker()
            ln.header.frame_id = "world"
            ln.header.stamp    = self.get_clock().now().to_msg()
            ln.ns="traj"; ln.id=50
            ln.type=Marker.LINE_STRIP; ln.action=Marker.ADD
            ln.scale.x = 0.004
            cols=[(0.3,1.0,0.3),(1.0,0.6,0.0),(0.2,0.7,1.0),(1.0,0.3,0.3)]
            rc,gc,bc = cols[self.phase]
            for pt in self.traj[-400:]:
                p2=Point(); p2.x,p2.y,p2.z=pt
                ln.points.append(p2)
                c2=ColorRGBA(); c2.r=rc; c2.g=gc; c2.b=bc; c2.a=0.85
                ln.colors.append(c2)
            arr.markers.append(ln)

        # Sphere outil
        ot = Marker()
        ot.header.frame_id = "world"
        ot.header.stamp    = self.get_clock().now().to_msg()
        ot.ns="outil"; ot.id=99
        ot.type=Marker.SPHERE; ot.action=Marker.ADD
        ot.pose.position.x = self.pos["joint_x"]
        ot.pose.position.y = self.pos["joint_y"] - 0.5
        ot.pose.position.z = max(0.0, 1.5 - self.pos["joint_z"])
        ot.pose.orientation.w = 1.0
        ot.scale.x=ot.scale.y=ot.scale.z=0.018
        ot.color.r=1.0; ot.color.g=1.0; ot.color.b=0.0; ot.color.a=1.0
        arr.markers.append(ot)

        self.pub_markers.publish(arr)

    def interp(self, cible, v=0.003):
        ok = True
        for k,val in cible.items():
            d = val - self.pos[k]
            if abs(d) > 0.001:
                ok = False
                self.pos[k] += min(abs(d),v)*(1 if d>0 else -1)
        return ok

    def enreg(self):
        pt = (self.pos["joint_x"],
              self.pos["joint_y"]-0.5,
              max(0.0, 1.5-self.pos["joint_z"]))
        if not self.traj or abs(pt[0]-self.traj[-1][0])>0.004 or abs(pt[1]-self.traj[-1][1])>0.004:
            self.traj.append(pt)

    def cycle(self):
        self.t += 0.02

        if self.etape == 0:
            if self.interp({"joint_x":0.0,"joint_y":0.0,"joint_z":0.0,"joint_a":0.0,"joint_c":0.0},0.005):
                self.phase=0; self.traj=[]; self.etape=1; self.t=0.0
                self.get_logger().info("ETAPE 1 : Piece brute chargee")

        elif self.etape == 1:
            if self.interp({"joint_x":0.0,"joint_y":0.15,"joint_z":0.0,"joint_a":0.0,"joint_c":0.0},0.003):
                self.etape=2; self.t=0.0
                self.get_logger().info("ETAPE 2 : Approche outil")

        elif self.etape == 2:
            if self.interp({"joint_x":0.0,"joint_y":0.15,"joint_z":0.32,"joint_a":0.0,"joint_c":0.0},0.002):
                self.etape=3; self.t=0.0
                self.get_logger().info("ETAPE 3 : Ebauche passes paralleles")

        elif self.etape == 3:
            ys=[-0.12,-0.06,0.0,0.06,0.12,0.18]
            idx=int(self.t/2.5)
            if idx >= len(ys):
                self.phase=1; self.etape=4; self.t=0.0
                self.get_logger().info("ETAPE 4 : Ebauche OK -> tournage")
            else:
                prog=(self.t%2.5)/2.5
                self.pos["joint_x"]=-0.35+prog*0.70
                self.pos["joint_y"]=ys[idx]
                self.pos["joint_z"]=0.34+idx*0.01

        elif self.etape == 4:
            self.pos["joint_c"]+=0.022
            self.pos["joint_x"]=math.cos(self.t*0.6)*0.10
            self.pos["joint_y"]=0.15
            self.pos["joint_z"]=0.36
            if self.pos["joint_c"]>=math.pi*4:
                self.phase=2; self.etape=5; self.t=0.0
                self.get_logger().info("ETAPE 5 : Usinage 5 axes")

        elif self.etape == 5:
            self.pos["joint_a"]=math.sin(self.t*0.55)*0.60
            self.pos["joint_c"]+=0.015
            self.pos["joint_x"]=math.cos(self.t*0.40)*0.22
            self.pos["joint_y"]=math.sin(self.t*0.28)*0.14+0.10
            self.pos["joint_z"]=0.29+math.sin(self.t*0.5)*0.07
            if self.t > 12.0:
                self.etape=6; self.t=0.0
                self.get_logger().info("ETAPE 6 : Finition")

        elif self.etape == 6:
            self.pos["joint_x"]=math.cos(self.t*1.3)*0.18
            self.pos["joint_y"]=math.sin(self.t*1.3)*0.18+0.10
            self.pos["joint_z"]=0.39
            self.pos["joint_a"]=0.0
            self.pos["joint_c"]+=0.008
            if self.t > 9.0:
                self.phase=3; self.etape=7; self.t=0.0
                self.get_logger().info("ETAPE 7 : PIECE FINIE !")

        elif self.etape == 7:
            if self.interp({"joint_x":0.0,"joint_y":0.0,"joint_z":0.0,"joint_a":0.0,"joint_c":0.0},0.004):
                self.etape=8
                self.get_logger().info("*** CYCLE TERMINE - nouveau cycle dans 5 sec ***")

        elif self.etape == 8:
            if self.t > 5.0:
                self.etape=0; self.t=0.0; self.traj=[]

        self.enreg()
        self.pub_j()
        self.pub_m()

def main(args=None):
    rclpy.init(args=args)
    node = UsinageSTL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
