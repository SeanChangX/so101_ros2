import json
import math
import os
import ssl
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import SetBool

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
ALL_JOINTS = ARM_JOINTS + ['gripper']

DEG2RAD = math.pi / 180.0


def _wrap_delta(deg: float) -> float:
    """Normalise an angle delta to [-180, 180] degrees."""
    deg = deg % 360.0
    if deg > 180.0:
        deg -= 360.0
    return deg


# ---------------------------------------------------------------------------
# Shared state (HTTP thread <-> ROS callbacks)
# ---------------------------------------------------------------------------

class _State:
    def __init__(self) -> None:
        self._mu = threading.Lock()
        self.token: Optional[str] = None
        self.last_heartbeat: float = 0.0
        self.control_active: bool = False
        self.home_alpha: float = 0.0
        self.home_beta: float = 0.0
        self.home_gamma: float = 0.0
        self.home_joints: Optional[np.ndarray] = None
        self.current_joints: Optional[np.ndarray] = None
        self.gripper_pct: float = 0.0
        self.enabled: bool = True
        self._reset_requested: bool = False
        self._pending_torque: Optional[bool] = None

    def acquire(self) -> Optional[str]:
        with self._mu:
            if self.token is not None:
                return None
            tok = str(uuid.uuid4())
            self.token = tok
            self.last_heartbeat = time.monotonic()
            return tok

    def release(self, token: str) -> bool:
        with self._mu:
            if self.token != token:
                return False
            self.token = None
            self.control_active = False
            return True

    def heartbeat(self, token: str) -> bool:
        with self._mu:
            if self.token != token:
                return False
            self.last_heartbeat = time.monotonic()
            return True

    def expire_if_stale(self, timeout: float) -> bool:
        with self._mu:
            if self.token is None:
                return False
            if time.monotonic() - self.last_heartbeat > timeout:
                self.token = None
                self.control_active = False
                return True
        return False

    def start_control(self, token: str, alpha: float, beta: float, gamma: float) -> bool:
        with self._mu:
            if self.token != token or self.current_joints is None:
                return False
            self.control_active = True
            self.enabled = True
            self._pending_torque = True
            self.home_alpha = alpha
            self.home_beta = beta
            self.home_gamma = gamma
            self.home_joints = self.current_joints.copy()
            return True

    def stop_control(self, token: str) -> bool:
        with self._mu:
            if self.token != token:
                return False
            self.control_active = False
            self.enabled = True
            return True

    def reset_home(self, token: str, alpha: float, beta: float, gamma: float) -> bool:
        """Zero offsets so arm returns to original start position.

        home_joints stays unchanged. Only the phone orientation reference
        is updated so the current phone angle becomes the new neutral.
        """
        with self._mu:
            if self.token != token or self.home_joints is None:
                return False
            self.home_alpha = alpha
            self.home_beta = beta
            self.home_gamma = gamma
            self._reset_requested = True
            return True

    def set_enabled(self, token: str, on: bool,
                    alpha: float = 0.0, beta: float = 0.0, gamma: float = 0.0) -> bool:
        with self._mu:
            if self.token != token or not self.control_active:
                return False
            self.enabled = on
            self._pending_torque = on
            if on and self.current_joints is not None:
                self.home_joints = self.current_joints.copy()
                self.home_alpha = alpha
                self.home_beta = beta
                self.home_gamma = gamma
                self._reset_requested = True
            return True

    def status(self, token: Optional[str] = None) -> dict:
        with self._mu:
            return {
                'locked': self.token is not None,
                'is_mine': token is not None and token == self.token,
                'control_active': self.control_active,
                'enabled': self.enabled,
            }

    def snapshot(self):
        with self._mu:
            if not self.control_active or self.home_joints is None:
                return None
            return (
                self.home_alpha,
                self.home_beta,
                self.home_gamma,
                self.home_joints.copy(),
                self.gripper_pct,
                self.enabled,
            )


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------

class _Handler(BaseHTTPRequestHandler):
    state: _State
    web_dir: str
    node_logger = None

    def log_message(self, fmt, *args):
        if self.node_logger:
            self.node_logger.debug('HTTP %s' % (fmt % args))

    def _json(self, code: int, data: dict) -> None:
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def _file(self, rel: str, ctype: str) -> None:
        path = os.path.join(self.web_dir, rel)
        try:
            with open(path, 'rb') as fh:
                data = fh.read()
        except FileNotFoundError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> dict:
        length = int(self.headers.get('Content-Length', 0))
        if length == 0:
            return {}
        try:
            return json.loads(self.rfile.read(length))
        except Exception:
            return {}

    def _token_from_request(self, body: dict) -> Optional[str]:
        return body.get('token') or self.headers.get('X-Session-Token')

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._file('index.html', 'text/html; charset=utf-8')
        elif self.path == '/roslib.min.js':
            self._file('roslib.min.js', 'application/javascript')
        elif self.path == '/api/lock/status':
            tok = self.headers.get('X-Session-Token')
            self._json(200, self.state.status(tok))
        else:
            self.send_error(404)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type, X-Session-Token')
        self.end_headers()

    def do_POST(self):
        body = self._body()
        p = self.path

        if p == '/api/lock/acquire':
            tok = self.state.acquire()
            if tok:
                self._json(200, {'token': tok})
            else:
                self._json(423, {'error': 'locked'})

        elif p == '/api/lock/release':
            if self.state.release(self._token_from_request(body) or ''):
                self._json(200, {'ok': True})
            else:
                self._json(401, {'error': 'invalid token'})

        elif p == '/api/lock/heartbeat':
            if self.state.heartbeat(self._token_from_request(body) or ''):
                self._json(200, {'ok': True})
            else:
                self._json(401, {'error': 'invalid token'})

        elif p == '/api/control/start':
            tok = body.get('token')
            alpha = float(body.get('alpha', 0.0))
            beta = float(body.get('beta', 0.0))
            gamma = float(body.get('gamma', 0.0))
            if self.state.start_control(tok or '', alpha, beta, gamma):
                self._json(200, {'ok': True})
            else:
                self._json(409, {'error': 'not locked or no joint feedback yet'})

        elif p == '/api/control/stop':
            if self.state.stop_control(self._token_from_request(body) or ''):
                self._json(200, {'ok': True})
            else:
                self._json(401, {'error': 'invalid token'})

        elif p == '/api/control/enable':
            tok = self._token_from_request(body)
            on = bool(body.get('enabled', True))
            alpha = float(body.get('alpha', 0.0))
            beta = float(body.get('beta', 0.0))
            gamma = float(body.get('gamma', 0.0))
            if self.state.set_enabled(tok or '', on, alpha, beta, gamma):
                self._json(200, {'ok': True, 'enabled': on})
            else:
                self._json(409, {'error': 'not controlling'})

        elif p == '/api/control/reset':
            tok = body.get('token')
            alpha = float(body.get('alpha', 0.0))
            beta = float(body.get('beta', 0.0))
            gamma = float(body.get('gamma', 0.0))
            if self.state.reset_home(tok or '', alpha, beta, gamma):
                self._json(200, {'ok': True})
            else:
                self._json(409, {'error': 'not locked or no joint feedback'})

        else:
            self.send_error(404)


def _make_handler(state: _State, web_dir: str, logger) -> type:
    return type('Handler', (_Handler,), {
        'state': state,
        'web_dir': web_dir,
        'node_logger': logger,
    })


# ---------------------------------------------------------------------------
# ROS node -- direct position mapping (phone = gripper)
# ---------------------------------------------------------------------------

class WebTeleopNode(Node):
    """
    Direct position mapping: phone IS the gripper.

    Orientation axes use POSITION control (tilt 30 deg = joint offset 30 deg).
    Z axis uses the phone's vertical acceleration (DeviceMotion API),
    integrated into a height offset distributed across multiple joints
    for approximately vertical end-effector movement.

    Mapping (phone held portrait, screen facing user):
      Yaw   (alpha)  -->  shoulder_pan
      Pitch (beta)   -->  shoulder_lift + elbow_flex + wrist_flex
      Roll  (gamma)  -->  wrist_roll
      Vertical accel  -->  height offset (shoulder_lift, elbow_flex, wrist_flex)
    """

    JOINT_LIMITS = {
        'shoulder_pan':  (-2.09, 2.09),
        'shoulder_lift': (-2.09, 2.09),
        'elbow_flex':    (-2.09, 2.09),
        'wrist_flex':    (-1.57, 1.57),
        'wrist_roll':    (-2.09, 2.09),
    }

    CTRL_HZ = 30

    def __init__(self) -> None:
        super().__init__('web_teleop_node')

        self.declare_parameter('http_port', 8080)
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('ssl_cert', '')
        self.declare_parameter('ssl_key', '')
        self.declare_parameter('session_timeout', 30.0)
        self.declare_parameter('deadzone_deg', 3.0)

        # Gains: ratio of joint movement per degree of phone tilt.
        self.declare_parameter('gain_pan',   1.0)
        self.declare_parameter('gain_lift',  0.6)
        self.declare_parameter('gain_elbow', 0.4)
        self.declare_parameter('gain_wrist', 0.4)
        self.declare_parameter('gain_roll',  1.0)

        # Z axis (phone vertical acceleration -> height offset)
        self.declare_parameter('z_speed', 0.5)
        self.declare_parameter('z_elbow_comp', 0.5)
        self.declare_parameter('z_wrist_comp', 0.3)

        self._http_port = self.get_parameter('http_port').value
        self._ssl_cert = self.get_parameter('ssl_cert').value
        self._ssl_key = self.get_parameter('ssl_key').value
        self._session_timeout = self.get_parameter('session_timeout').value
        self._deadzone = self.get_parameter('deadzone_deg').value

        self._gain = np.array([
            self.get_parameter('gain_pan').value,
            self.get_parameter('gain_lift').value,
            self.get_parameter('gain_elbow').value,
            self.get_parameter('gain_wrist').value,
            self.get_parameter('gain_roll').value,
        ])
        self._z_speed = self.get_parameter('z_speed').value
        self._z_elbow_comp = self.get_parameter('z_elbow_comp').value
        self._z_wrist_comp = self.get_parameter('z_wrist_comp').value

        self._state = _State()
        self._n_joints = len(ARM_JOINTS)

        # IMU data (written by _on_imu, read by _control_tick)
        self._imu_lock = threading.Lock()
        self._d_alpha = 0.0
        self._d_beta = 0.0
        self._d_gamma = 0.0
        self._z_vel = 0.0
        self._z_last_t = 0.0

        # Z height offset (integrated from buttons, the ONLY integration)
        self._z_offset = 0.0
        self._last_q_delta = np.zeros(self._n_joints)
        self._was_active = False

        # ROS
        self.create_subscription(
            JointState, '/follower/joint_states', self._on_follower_js, 10)
        self.create_subscription(
            Float64MultiArray, '/web_teleop/imu_command', self._on_imu, 10)
        self.create_subscription(
            Float64, '/web_teleop/gripper_cmd', self._on_gripper, 10)
        self.create_subscription(
            Float64, '/web_teleop/z_cmd', self._on_z_cmd, 10)
        self._leader_pub = self.create_publisher(
            JointState, '/leader/joint_states', 10)
        self._torque_cli = self.create_client(SetBool, '/follower/set_torque')

        self._ctrl_dt = 1.0 / self.CTRL_HZ
        self.create_timer(self._ctrl_dt, self._control_tick)
        self.create_timer(1.0, self._watchdog)

        self._start_http_server()

        proto = 'HTTPS' if (self._ssl_cert and self._ssl_key) else 'HTTP'
        self.get_logger().info(
            f'so101_web_teleop ready  {proto} :{self._http_port}  '
            f'mode=DIRECT_POSITION  gains={self._gain.tolist()}  '
            f'dz={self._deadzone}deg  z_spd={self._z_speed}  '
            f'z_comp=({self._z_elbow_comp},{self._z_wrist_comp})'
        )

    # -- HTTP server --

    def _start_http_server(self) -> None:
        web_dir = os.path.join(
            get_package_share_directory('so101_web_teleop'), 'web')
        handler_cls = _make_handler(self._state, web_dir, self.get_logger())
        server = ThreadingHTTPServer(('0.0.0.0', self._http_port), handler_cls)

        if self._ssl_cert and self._ssl_key:
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(self._ssl_cert, self._ssl_key)
            server.socket = ctx.wrap_socket(server.socket, server_side=True)

        threading.Thread(target=server.serve_forever, daemon=True).start()

    # -- ROS callbacks --

    def _on_follower_js(self, msg: JointState) -> None:
        joints = np.zeros(self._n_joints)
        for i, name in enumerate(ARM_JOINTS):
            if name in msg.name:
                joints[i] = msg.position[list(msg.name).index(name)]
        with self._state._mu:
            self._state.current_joints = joints

    def _on_imu(self, msg: Float64MultiArray) -> None:
        """Compute angle deltas from home and store them."""
        if len(msg.data) < 3:
            return
        snap = self._state.snapshot()
        if snap is None:
            return
        home_alpha, home_beta, home_gamma, _, _, _ = snap
        alpha, beta, gamma = msg.data[0], msg.data[1], msg.data[2]

        d_alpha = self._dz(_wrap_delta(alpha - home_alpha))
        d_beta = self._dz(beta - home_beta)
        d_gamma = self._dz(gamma - home_gamma)

        with self._imu_lock:
            self._d_alpha = d_alpha
            self._d_beta = d_beta
            self._d_gamma = d_gamma

    def _on_z_cmd(self, msg: Float64) -> None:
        with self._imu_lock:
            self._z_vel = float(np.clip(msg.data, -1.0, 1.0))
            self._z_last_t = time.monotonic()

    def _on_gripper(self, msg: Float64) -> None:
        """Publish immediately so slider feels responsive. Only when control is active."""
        with self._state._mu:
            self._state.gripper_pct = float(np.clip(msg.data, 0.0, 100.0))
            gripper_pct = self._state.gripper_pct
            active = self._state.control_active
            enabled = self._state.enabled
            home = (self._state.home_joints.copy()
                    if self._state.home_joints is not None else None)

        if not active or not enabled or home is None:
            return

        gripper_rad = gripper_pct / 100.0 * 1.5
        self._publish(home + self._last_q_delta, gripper_rad)

    # -- control loop --

    def _control_tick(self) -> None:
        """30 Hz: map phone orientation directly to joint offsets."""
        with self._state._mu:
            pending_torque = self._state._pending_torque
            if pending_torque is not None:
                self._state._pending_torque = None
            if self._state._reset_requested:
                self._state._reset_requested = False
                self._z_offset = 0.0
                self._last_q_delta = np.zeros(self._n_joints)
                with self._imu_lock:
                    self._d_alpha = 0.0
                    self._d_beta = 0.0
                    self._d_gamma = 0.0
                    self._z_vel = 0.0

        if pending_torque is not None:
            self._request_torque(pending_torque)

        snap = self._state.snapshot()
        if snap is None:
            if self._was_active:
                self._z_offset = 0.0
                self._was_active = False
            return

        if not self._was_active:
            self._was_active = True
            self._z_offset = 0.0
            with self._imu_lock:
                self._d_alpha = 0.0
                self._d_beta = 0.0
                self._d_gamma = 0.0
            return

        _, _, _, home_joints, gripper_pct, enabled = snap

        if not enabled:
            return

        with self._imu_lock:
            da = self._d_alpha
            db = self._d_beta
            dg = self._d_gamma
            z_vel = self._z_vel
            if z_vel != 0.0 and time.monotonic() - self._z_last_t > 0.3:
                z_vel = 0.0
                self._z_vel = 0.0

        # Z height integration (the ONLY integration in the whole system)
        self._z_offset += z_vel * self._z_speed * self._ctrl_dt
        self._z_offset = np.clip(self._z_offset, -1.5, 1.5)

        # ---- DIRECT POSITION MAPPING ----
        # phone_deg_delta * DEG2RAD * gain = joint_rad_delta
        #
        # shoulder_pan  : yaw   (alpha)
        # shoulder_lift : pitch (beta) + z_offset (up)
        # elbow_flex    : pitch (beta) - z_offset * comp (maintain reach)
        # wrist_flex    : pitch (beta) + z_offset * comp (maintain gripper angle)
        # wrist_roll    : roll  (gamma)
        zo = self._z_offset
        q_delta = np.array([
            -da * DEG2RAD * self._gain[0],
            -db * DEG2RAD * self._gain[1] + zo,
            -db * DEG2RAD * self._gain[2] - zo * self._z_elbow_comp,
             db * DEG2RAD * self._gain[3] + zo * self._z_wrist_comp,
            -dg * DEG2RAD * self._gain[4],
        ])

        # Clamp to joint limits
        for i, name in enumerate(ARM_JOINTS):
            lo, hi = self.JOINT_LIMITS[name]
            q_delta[i] = np.clip(q_delta[i], lo - home_joints[i], hi - home_joints[i])

        self._last_q_delta = q_delta
        new_joints = home_joints + q_delta
        gripper_rad = gripper_pct / 100.0 * 1.5
        self._publish(new_joints, gripper_rad)

    # -- helpers --

    def _request_torque(self, enable: bool) -> None:
        if not self._torque_cli.service_is_ready():
            self.get_logger().warn('set_torque service not available')
            return
        req = SetBool.Request()
        req.data = enable
        future = self._torque_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Torque {'enabled' if enable else 'disabled'}: "
                f"{f.result().message if f.result() else 'no response'}"))

    def _dz(self, deg: float) -> float:
        if abs(deg) < self._deadzone:
            return 0.0
        return (abs(deg) - self._deadzone) * (1.0 if deg > 0 else -1.0)

    def _publish(self, arm_joints: np.ndarray, gripper_rad: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = list(arm_joints) + [gripper_rad]
        self._leader_pub.publish(msg)

    def _watchdog(self) -> None:
        if self._state.expire_if_stale(self._session_timeout):
            self.get_logger().info('Web teleop session timed out; lock released')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
