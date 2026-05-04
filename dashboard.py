#!/usr/bin/env python3
"""
Interface web DMG MORI 5 axes
Ouvre http://localhost:8080 dans ton navigateur
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import math
import json
import os
from http.server import HTTPServer, BaseHTTPRequestHandler

# ─── État global partagé ─────────────────────────────
state = {
    "x": 0.0, "y": 0.0, "z": 0.0,
    "a": 0.0, "c": 0.0,
    "vitesse": 500.0, "broche": 8000,
    "statut": "ARRETE",
    "etape": 0, "phase": 0,
    "ligne_gcode": 0, "total_lignes": 47,
    "message": "En attente...",
    "commande": None,
}

HTML = """<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DMG MORI 5X — Controle</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#0d1117;color:#e2e8f0;padding:16px}
h1{font-size:18px;font-weight:500;color:#60a5fa;margin-bottom:16px}
.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-bottom:12px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:12px}
.card{background:#161b22;border:1px solid #21262d;border-radius:8px;padding:12px}
.lbl{font-size:11px;color:#8b949e;text-transform:uppercase;letter-spacing:.05em;margin-bottom:2px}
.val{font-size:22px;font-weight:500}
.unit{font-size:12px;color:#6b7280;margin-left:3px}
.sub{font-size:11px;color:#4b5563;margin-top:2px}
.btn-row{display:flex;gap:8px;margin-bottom:12px}
.btn{padding:10px 0;border-radius:8px;border:none;font-size:14px;font-weight:500;cursor:pointer;flex:1;transition:opacity .15s}
.btn:hover{opacity:.85}
.btn:active{transform:scale(.97)}
.start{background:#166534;color:#4ade80}
.pause{background:#92400e;color:#fbbf24}
.stop{background:#7f1d1d;color:#f87171}
.reset{background:#1e3a5f;color:#60a5fa}
.prog-wrap{margin-bottom:12px}
.prog-info{display:flex;justify-content:space-between;font-size:12px;color:#8b949e;margin-bottom:4px}
.prog-bg{height:8px;background:#21262d;border-radius:4px;overflow:hidden}
.prog-fill{height:100%;background:#2563eb;border-radius:4px;transition:width .5s}
.badge{display:inline-block;padding:3px 10px;border-radius:20px;font-size:12px;font-weight:500;margin-left:8px}
.run{background:#0f3d1f;color:#4ade80;border:1px solid #166534}
.stp{background:#3d0f0f;color:#f87171;border:1px solid #7f1d1d}
.pau{background:#3d2d0f;color:#fbbf24;border:1px solid #92400e}
.gcode{background:#0d1117;border:1px solid #21262d;border-radius:8px;padding:12px;font-family:monospace;font-size:12px;margin-bottom:12px;max-height:160px;overflow-y:auto}
.gl{color:#6b7280;padding:2px 0}
.ga{color:#60a5fa;background:#0f2040;padding:2px 6px;border-radius:3px}
.gd{color:#238636}
.axis-row{display:flex;align-items:center;padding:5px 0;border-bottom:1px solid #21262d}
.axis-row:last-child{border:none}
.an{width:20px;font-size:13px;color:#8b949e}
.ab{flex:1;height:6px;background:#21262d;border-radius:3px;margin:0 10px;overflow:hidden}
.af{height:100%;border-radius:3px;transition:width .3s}
.av{width:70px;text-align:right;font-family:monospace;font-size:13px}
.msg{font-size:13px;color:#8b949e;padding:8px 12px;background:#161b22;border-radius:6px;border-left:3px solid #2563eb;margin-bottom:12px}
</style>
</head>
<body>
<div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:16px">
  <h1>DMG MORI 5X — Panneau de controle</h1>
  <span id="badge" class="badge stp">ARRETE</span>
</div>

<div class="grid4">
  <div class="card"><div class="lbl">Axe X</div><div class="val" id="vx">0.0<span class="unit">mm</span></div><div class="sub">lim ±450mm</div></div>
  <div class="card"><div class="lbl">Axe Y</div><div class="val" id="vy">0.0<span class="unit">mm</span></div><div class="sub">lim ±350mm</div></div>
  <div class="card"><div class="lbl">Axe Z</div><div class="val" id="vz">0.0<span class="unit">mm</span></div><div class="sub">lim 0–700mm</div></div>
  <div class="card"><div class="lbl">Broche</div><div class="val" id="vb" style="color:#4ade80">OFF<span class="unit">tr/m</span></div><div class="sub" id="vf">F: 0 mm/min</div></div>
</div>

<div class="grid2">
  <div class="card"><div class="lbl">Axe A (berceau)</div><div class="val" id="va">0.0<span class="unit">°</span></div></div>
  <div class="card"><div class="lbl">Axe C (plateau)</div><div class="val" id="vc">0.0<span class="unit">°</span></div></div>
</div>

<div class="btn-row">
  <button class="btn start" onclick="cmd("start")">&#9654; Start</button>
  <button class="btn pause" onclick="cmd("pause")">&#9646;&#9646; Pause</button>
  <button class="btn stop"  onclick="cmd("stop")">&#9632; Stop</button>
  <button class="btn reset" onclick="cmd("reset")">&#8635; Reset</button>
</div>

<div class="prog-wrap">
  <div class="prog-info"><span>Progression</span><span id="pinfo">Ligne 0 / 47</span></div>
  <div class="prog-bg"><div class="prog-fill" id="pfill" style="width:0%"></div></div>
</div>

<div class="msg" id="msg">En attente de demarrage...</div>

<div class="gcode" id="gbox">
  <div class="gl">G21 — unites mm</div>
  <div class="gl">G90 — mode absolu</div>
  <div class="ga">&#9654; G00 X0 Y0 Z50 — HOME</div>
  <div class="gl">G01 X80 Y0 F500</div>
  <div class="gl">G02 X80 Y0 I-80 J0</div>
  <div class="gl">A30 — inclinaison berceau</div>
  <div class="gl">C90 — rotation plateau</div>
  <div class="gl">M30 — fin programme</div>
</div>

<div class="card">
  <div class="lbl" style="margin-bottom:8px">Position axes</div>
  <div class="axis-row"><span class="an">X</span><div class="ab"><div class="af" id="bx" style="width:50%;background:#2563eb"></div></div><span class="av" id="tx">0.0</span></div>
  <div class="axis-row"><span class="an">Y</span><div class="ab"><div class="af" id="by" style="width:50%;background:#7c3aed"></div></div><span class="av" id="ty">0.0</span></div>
  <div class="axis-row"><span class="an">Z</span><div class="ab"><div class="af" id="bz" style="width:0%;background:#059669"></div></div><span class="av" id="tz">0.0</span></div>
  <div class="axis-row"><span class="an">A</span><div class="ab"><div class="af" id="ba" style="width:50%;background:#d97706"></div></div><span class="av" id="ta">0.0°</span></div>
  <div class="axis-row" style="border:none"><span class="an">C</span><div class="ab"><div class="af" id="bc" style="width:50%;background:#dc2626"></div></div><span class="av" id="tc">0.0°</span></div>
</div>

<script>
function cmd(c){fetch("/cmd?c="+c)}

function refresh(){
  fetch("/state").then(r=>r.json()).then(d=>{
    document.getElementById("vx").innerHTML = d.x.toFixed(1)+"<span class=unit>mm</span>"
    document.getElementById("vy").innerHTML = d.y.toFixed(1)+"<span class=unit>mm</span>"
    document.getElementById("vz").innerHTML = d.z.toFixed(1)+"<span class=unit>mm</span>"
    document.getElementById("va").innerHTML = d.a.toFixed(1)+"<span class=unit>°</span>"
    document.getElementById("vc").innerHTML = d.c.toFixed(1)+"<span class=unit>°</span>"
    document.getElementById("vb").innerHTML = (d.statut=="EN MARCHE"?d.broche:"OFF")+"<span class=unit>tr/m</span>"
    document.getElementById("vf").textContent = "F: "+d.vitesse.toFixed(0)+" mm/min"
    document.getElementById("msg").textContent = d.message

    var pct = Math.round(d.ligne_gcode/d.total_lignes*100)
    document.getElementById("pfill").style.width = pct+"%"
    document.getElementById("pinfo").textContent = "Ligne "+d.ligne_gcode+" / "+d.total_lignes+" ("+pct+"%)"

    var b = document.getElementById("badge")
    b.textContent = d.statut
    b.className = "badge "+(d.statut=="EN MARCHE"?"run":d.statut=="PAUSE"?"pau":"stp")

    // Barres axes
    var bx = (d.x+450)/900*100
    var by = (d.y+350)/700*100
    var bz = d.z/700*100
    var ba = (d.a+70)/140*100
    var bc = (d.c%360+360)%360/360*100
    document.getElementById("bx").style.width=Math.max(0,Math.min(100,bx))+"%"
    document.getElementById("by").style.width=Math.max(0,Math.min(100,by))+"%"
    document.getElementById("bz").style.width=Math.max(0,Math.min(100,bz))+"%"
    document.getElementById("ba").style.width=Math.max(0,Math.min(100,ba))+"%"
    document.getElementById("bc").style.width=Math.max(0,Math.min(100,bc))+"%"
    document.getElementById("tx").textContent=d.x.toFixed(1)
    document.getElementById("ty").textContent=d.y.toFixed(1)
    document.getElementById("tz").textContent=d.z.toFixed(1)
    document.getElementById("ta").textContent=d.a.toFixed(1)+"°"
    document.getElementById("tc").textContent=d.c.toFixed(1)+"°"
  }).catch(()=>{})
}
setInterval(refresh, 200)
refresh()
</script>
</body>
</html>"""

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): pass

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type","text/html;charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML.encode())
        elif self.path == "/state":
            self.send_response(200)
            self.send_header("Content-Type","application/json")
            self.send_header("Access-Control-Allow-Origin","*")
            self.end_headers()
            self.wfile.write(json.dumps(state).encode())
        elif self.path.startswith("/cmd"):
            c = self.path.split("=")[-1]
            state["commande"] = c
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"ok")

class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.t = 0.0
        self.en_pause = False
        self.actif = False
        self.pos = {"joint_x":0.0,"joint_y":0.0,"joint_z":0.0,
                    "joint_a":0.0,"joint_c":0.0}
        self.etape = 0
        self.timer = self.create_timer(0.02, self.step)

        srv = HTTPServer(("0.0.0.0", 8080), Handler)
        th = threading.Thread(target=srv.serve_forever, daemon=True)
        th.start()
        self.get_logger().info("Interface web : http://localhost:8080")

    def pub_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(self.pos.keys())
        msg.position = list(self.pos.values())
        msg.velocity = [0.0]*5
        msg.effort   = [0.0]*5
        self.pub.publish(msg)
        state["x"] = self.pos["joint_x"]*1000
        state["y"] = self.pos["joint_y"]*1000
        state["z"] = self.pos["joint_z"]*1000
        state["a"] = math.degrees(self.pos["joint_a"])
        state["c"] = math.degrees(self.pos["joint_c"])

    def interp(self, cible, v=0.003):
        ok = True
        for k,val in cible.items():
            d = val - self.pos[k]
            if abs(d) > 0.001:
                ok = False
                self.pos[k] += min(abs(d),v)*(1 if d>0 else -1)
        return ok

    def step(self):
        # Gestion commandes
        cmd = state.get("commande")
        if cmd:
            state["commande"] = None
            if cmd == "start":
                self.actif = True
                self.en_pause = False
                state["statut"] = "EN MARCHE"
                state["broche"] = 8000
                self.get_logger().info("START")
            elif cmd == "pause":
                self.en_pause = not self.en_pause
                state["statut"] = "PAUSE" if self.en_pause else "EN MARCHE"
                self.get_logger().info("PAUSE" if self.en_pause else "REPRISE")
            elif cmd == "stop":
                self.actif = False
                self.en_pause = False
                state["statut"] = "ARRETE"
                state["broche"] = 0
                state["message"] = "Arrete par operateur"
                self.get_logger().info("STOP")
            elif cmd == "reset":
                self.actif = False
                self.en_pause = False
                self.etape = 0
                self.t = 0.0
                state["statut"] = "ARRETE"
                state["broche"] = 0
                state["ligne_gcode"] = 0
                state["message"] = "Reset effectue"
                for k in self.pos: self.pos[k] = 0.0
                self.get_logger().info("RESET")

        if not self.actif or self.en_pause:
            self.pub_joints()
            return

        self.t += 0.02

        if self.etape == 0:
            state["message"] = "Retour HOME..."
            state["ligne_gcode"] = 1
            if self.interp({"joint_x":0.0,"joint_y":0.0,"joint_z":0.0,
                            "joint_a":0.0,"joint_c":0.0}, 0.005):
                self.etape = 1; self.t = 0.0
                state["message"] = "HOME atteint — piece brute chargee"

        elif self.etape == 1:
            state["message"] = "Approche outil..."
            state["ligne_gcode"] = 5
            if self.interp({"joint_x":0.0,"joint_y":0.15,
                            "joint_z":0.0,"joint_a":0.0,"joint_c":0.0}, 0.003):
                self.etape = 2; self.t = 0.0
                state["message"] = "Approche OK — descente Z"

        elif self.etape == 2:
            state["message"] = "Descente axe Z..."
            state["ligne_gcode"] = 8
            if self.interp({"joint_x":0.0,"joint_y":0.15,
                            "joint_z":0.32,"joint_a":0.0,"joint_c":0.0}, 0.002):
                self.etape = 3; self.t = 0.0
                state["message"] = "Contact piece — ebauche en cours"

        elif self.etape == 3:
            state["ligne_gcode"] = int(10 + self.t*2)
            state["vitesse"] = 500.0
            ys = [-0.12,-0.06,0.0,0.06,0.12,0.18]
            idx = int(self.t/2.5)
            if idx >= len(ys):
                self.etape = 4; self.t = 0.0
                state["message"] = "Ebauche terminee — tournage C"
            else:
                prog = (self.t%2.5)/2.5
                self.pos["joint_x"] = -0.35+prog*0.70
                self.pos["joint_y"] = ys[idx]
                self.pos["joint_z"] = 0.34+idx*0.01
                state["message"] = f"Ebauche — passe {idx+1}/6"

        elif self.etape == 4:
            state["ligne_gcode"] = 22
            state["message"] = "Tournage axe C..."
            self.pos["joint_c"] += 0.022
            self.pos["joint_x"] = math.cos(self.t*0.6)*0.10
            self.pos["joint_y"] = 0.15
            self.pos["joint_z"] = 0.36
            if self.pos["joint_c"] >= math.pi*4:
                self.etape = 5; self.t = 0.0
                state["message"] = "Tournage OK — usinage 5 axes"

        elif self.etape == 5:
            state["ligne_gcode"] = 28
            state["vitesse"] = 300.0
            state["message"] = "Usinage 5 axes simultanes"
            self.pos["joint_a"] = math.sin(self.t*0.55)*0.60
            self.pos["joint_c"] += 0.015
            self.pos["joint_x"] = math.cos(self.t*0.40)*0.22
            self.pos["joint_y"] = math.sin(self.t*0.28)*0.14+0.10
            self.pos["joint_z"] = 0.29+math.sin(self.t*0.5)*0.07
            if self.t > 12.0:
                self.etape = 6; self.t = 0.0

        elif self.etape == 6:
            state["ligne_gcode"] = 38
            state["vitesse"] = 600.0
            state["message"] = "Finition surface..."
            self.pos["joint_x"] = math.cos(self.t*1.3)*0.18
            self.pos["joint_y"] = math.sin(self.t*1.3)*0.18+0.10
            self.pos["joint_z"] = 0.39
            self.pos["joint_a"] = 0.0
            self.pos["joint_c"] += 0.008
            if self.t > 9.0:
                self.etape = 7; self.t = 0.0

        elif self.etape == 7:
            state["ligne_gcode"] = 45
            state["message"] = "Degagement outil..."
            if self.interp({"joint_x":0.0,"joint_y":0.0,"joint_z":0.0,
                            "joint_a":0.0,"joint_c":0.0}, 0.004):
                self.etape = 8
                state["ligne_gcode"] = 47
                state["message"] = "PIECE FINIE ! Cycle termine."

        elif self.etape == 8:
            state["message"] = "Cycle termine — appuie Start pour recommencer"
            state["statut"] = "ARRETE"
            state["broche"] = 0
            self.actif = False
            self.etape = 0

        self.pub_joints()

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
