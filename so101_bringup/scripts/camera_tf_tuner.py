#!/usr/bin/env python3

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def euler_to_quat(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class CameraTfTuner(Node):
    def __init__(self, parent: str, child: str, xyz, rpy, publish_rate_hz: float):
        super().__init__('camera_tf_tuner')
        self.parent_frame = parent
        self.child_frame = child
        self.xyz = list(xyz)
        self.rpy = list(rpy)
        self._lock = threading.Lock()

        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self.publish_tf)
        self.get_logger().info(
            f'TF tuner started: {self.parent_frame} -> {self.child_frame} '
            f'xyz={self.xyz}, rpy={self.rpy}'
        )

    def publish_tf(self):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.child_frame

        with self._lock:
            x, y, z = float(self.xyz[0]), float(self.xyz[1]), float(self.xyz[2])
            roll, pitch, yaw = float(self.rpy[0]), float(self.rpy[1]), float(self.rpy[2])

        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z

        qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform(msg)

    def yaml_snippet(self):
        with self._lock:
            x, y, z = self.xyz[0], self.xyz[1], self.xyz[2]
            roll, pitch, yaw = self.rpy[0], self.rpy[1], self.rpy[2]
        return (
            f'tf_parent_frame: {self.parent_frame}\n'
            f'tf_child_frame: {self.child_frame}\n'
            f'tf_xyz: [{x:.4f}, {y:.4f}, {z:.4f}]\n'
            f'tf_rpy: [{roll:.4f}, {pitch:.4f}, {yaw:.4f}]'
        )

    def get_state(self):
        with self._lock:
            return {
                'x': float(self.xyz[0]),
                'y': float(self.xyz[1]),
                'z': float(self.xyz[2]),
                'roll': float(self.rpy[0]),
                'pitch': float(self.rpy[1]),
                'yaw': float(self.rpy[2]),
            }

    def set_state(self, state):
        with self._lock:
            self.xyz[0] = float(state.get('x', self.xyz[0]))
            self.xyz[1] = float(state.get('y', self.xyz[1]))
            self.xyz[2] = float(state.get('z', self.xyz[2]))
            self.rpy[0] = float(state.get('roll', self.rpy[0]))
            self.rpy[1] = float(state.get('pitch', self.rpy[1]))
            self.rpy[2] = float(state.get('yaw', self.rpy[2]))


def parse_args():
    parser = argparse.ArgumentParser(description='Interactive camera TF tuner with sliders.')
    parser.add_argument('--parent', required=True, help='Parent frame id')
    parser.add_argument('--child', required=True, help='Child frame id')
    parser.add_argument('--x', type=float, default=0.0, help='Initial x in meters')
    parser.add_argument('--y', type=float, default=0.0, help='Initial y in meters')
    parser.add_argument('--z', type=float, default=0.0, help='Initial z in meters')
    parser.add_argument('--roll', type=float, default=0.0, help='Initial roll in radians')
    parser.add_argument('--pitch', type=float, default=0.0, help='Initial pitch in radians')
    parser.add_argument('--yaw', type=float, default=0.0, help='Initial yaw in radians')
    parser.add_argument('--pos-range-mm', type=float, default=150.0, help='Slider range +/- mm')
    parser.add_argument(
        '--angle-range-deg', type=float, default=180.0, help='Slider range +/- degrees'
    )
    parser.add_argument('--rate', type=float, default=30.0, help='TF publish rate in Hz')
    parser.add_argument(
        '--web-host',
        type=str,
        default='0.0.0.0',
        help='Web slider bind host (e.g. 0.0.0.0 or 127.0.0.1)',
    )
    parser.add_argument('--web-port', type=int, default=8765, help='Web slider server port')
    # Ignore ROS launch/appended args like:
    # --ros-args -r __node:=...
    args, _unknown = parser.parse_known_args()
    return args


def build_web_handler(node: CameraTfTuner, pos_range_mm: float, angle_range_deg: float):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, content: str, content_type: str = 'text/html'):
            body = content.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _fmt, *_args):
            return

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == '/state':
                self._send(200, json.dumps(node.get_state()), 'application/json')
                return
            if parsed.path == '/yaml':
                self._send(200, node.yaml_snippet(), 'text/plain')
                return
            html = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>Camera TF Tuner</title>
<style>
body {{ font-family: sans-serif; max-width: 760px; margin: 20px auto; }}
.row {{ margin: 10px 0; }}
label {{ display: inline-block; width: 120px; }}
input[type=range] {{ width: 470px; }}
pre {{ background: #f5f5f5; padding: 10px; }}
button {{ margin-right: 8px; }}
</style></head>
<body>
<h2>Camera TF Tuner (Web)</h2>
<div>Parent: <code>{node.parent_frame}</code></div>
<div>Child: <code>{node.child_frame}</code></div>
<div class="row"><label>x (mm)</label><input id="x" type="range" min="-{pos_range_mm}" max="{pos_range_mm}" step="0.5"></div>
<div class="row"><label>y (mm)</label><input id="y" type="range" min="-{pos_range_mm}" max="{pos_range_mm}" step="0.5"></div>
<div class="row"><label>z (mm)</label><input id="z" type="range" min="-{pos_range_mm}" max="{pos_range_mm}" step="0.5"></div>
<div class="row"><label>roll (deg)</label><input id="roll" type="range" min="-{angle_range_deg}" max="{angle_range_deg}" step="0.1"></div>
<div class="row"><label>pitch (deg)</label><input id="pitch" type="range" min="-{angle_range_deg}" max="{angle_range_deg}" step="0.1"></div>
<div class="row"><label>yaw (deg)</label><input id="yaw" type="range" min="-{angle_range_deg}" max="{angle_range_deg}" step="0.1"></div>
<pre id="vals"></pre>
<button onclick="printYaml()">Print YAML to Terminal</button>
<button onclick="copyYaml()">Copy YAML</button>
<pre id="yaml"></pre>
<script>
const ids = ['x','y','z','roll','pitch','yaw'];
function rad2deg(v) {{ return v * 180.0 / Math.PI; }}
function deg2rad(v) {{ return v * Math.PI / 180.0; }}
async function refresh() {{
  const st = await (await fetch('/state')).json();
  x.value = st.x * 1000.0; y.value = st.y * 1000.0; z.value = st.z * 1000.0;
  roll.value = rad2deg(st.roll); pitch.value = rad2deg(st.pitch); yaw.value = rad2deg(st.yaw);
  updateText();
}}
function statePayload() {{
  return {{
    x: parseFloat(x.value)/1000.0, y: parseFloat(y.value)/1000.0, z: parseFloat(z.value)/1000.0,
    roll: deg2rad(parseFloat(roll.value)), pitch: deg2rad(parseFloat(pitch.value)), yaw: deg2rad(parseFloat(yaw.value))
  }};
}}
function updateText() {{
  const p = statePayload();
  vals.textContent = `x=${{p.x.toFixed(4)}} m  y=${{p.y.toFixed(4)}} m  z=${{p.z.toFixed(4)}} m\\n` +
                     `roll=${{p.roll.toFixed(4)}} rad  pitch=${{p.pitch.toFixed(4)}} rad  yaw=${{p.yaw.toFixed(4)}} rad`;
}}
let t = null;
function send() {{
  updateText();
  if (t) clearTimeout(t);
  t = setTimeout(async () => {{
    await fetch('/set', {{method:'POST', headers:{{'Content-Type':'application/json'}}, body: JSON.stringify(statePayload())}});
  }}, 10);
}}
ids.forEach(i => document.getElementById(i).addEventListener('input', send));
async function printYaml() {{
  await fetch('/print', {{method:'POST'}});
  yaml.textContent = await (await fetch('/yaml')).text();
}}
async function copyYaml() {{
  const txt = await (await fetch('/yaml')).text();
  yaml.textContent = txt;
  await navigator.clipboard.writeText(txt);
}}
refresh();
</script></body></html>"""
            self._send(200, html, 'text/html')

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path == '/set':
                length = int(self.headers.get('Content-Length', '0'))
                body = self.rfile.read(length).decode('utf-8')
                try:
                    payload = json.loads(body) if body else {}
                except json.JSONDecodeError:
                    payload = {}
                if not payload:
                    qs = parse_qs(parsed.query)
                    payload = {k: float(v[0]) for k, v in qs.items() if v}
                node.set_state(payload)
                self._send(200, 'ok', 'text/plain')
                return
            if parsed.path == '/print':
                print('\n=== Camera TF (copy to so101_cameras.yaml) ===')
                print(node.yaml_snippet())
                print('=============================================\n')
                self._send(200, 'printed', 'text/plain')
                return
            self._send(404, 'not found', 'text/plain')

    return Handler


def main():
    args = parse_args()
    rclpy.init()

    node = CameraTfTuner(
        parent=args.parent,
        child=args.child,
        xyz=[args.x, args.y, args.z],
        rpy=[args.roll, args.pitch, args.yaw],
        publish_rate_hz=args.rate,
    )

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    handler = build_web_handler(
        node=node,
        pos_range_mm=abs(args.pos_range_mm),
        angle_range_deg=abs(args.angle_range_deg),
    )
    server = ThreadingHTTPServer((str(args.web_host), int(args.web_port)), handler)
    web_thread = threading.Thread(target=server.serve_forever, daemon=True)
    web_thread.start()

    print('\n=== Camera TF Tuner (web mode) ===')
    print(f'Bind: {args.web_host}:{args.web_port}')
    print(f'Open (same runtime): http://127.0.0.1:{args.web_port}')
    print(f'Open (host browser):  http://<runtime-ip>:{args.web_port}')
    print('Current values:')
    print(node.yaml_snippet())
    print('Press Ctrl+C to stop.\n')

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
