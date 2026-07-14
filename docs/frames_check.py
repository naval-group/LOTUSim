"""
vehicle_state.py
================
Utility module for representing and converting vehicle kinematic states between
the ENU (East-North-Up) and NED (North-East-Down) coordinate frames.

Coordinate Frames
-----------------
ENU (East-North-Up):
    x -> East
    y -> North
    z -> Up

NED (North-East-Down):
    x -> North
    y -> East
    z -> Down

The ENU<->NED frame change is a 180° rotation about the (X+Y)/√2 axis,
represented by the unit quaternion Q_R = (0, 1/√2, 1/√2, 0).

State Variables
---------------
Position        : x, y, z         [m]         fixed frame
Linear velocity : u, v, w         [m/s]       body frame
Angular velocity: p, q, r         [rad/s]     body frame
Attitude        : qr, qi, qj, qk  [-]         unit quaternion (scalar-first)
Euler angles    : phi, theta, psi [rad]       derived from quaternion, ZYX convention

ENU <-> NED Conversion Rules
-----------------------------
Position        : x_ned = y_enu,  y_ned = x_enu,  z_ned = -z_enu
Linear velocity : u_ned = u_enu,  v_ned = -v_enu, w_ned = -w_enu
Angular velocity: p_ned = p_enu,  q_ned = -q_enu, r_ned = -r_enu
Quaternion      : q_ned = Q_R ⊗ q_enu   (Q_R is self-inverse)

Quaternion Convention
---------------------
Scalar-first Hamilton convention: q = qr + qi*i + qj*j + qk*k
Canonical form enforces qr >= 0 to avoid the q / -q ambiguity.
Attitude quaternion encodes the rotation from body frame to fixed frame,
so that fixed-frame vectors are obtained via: v_fixed = q ⊗ v_body ⊗ q*

Contents
--------
Functions:
    quat_mult(a, b)         Hamilton product of two quaternions
    quat_conj(q)            Conjugate of a quaternion
    normalize_quat(q)       Normalize to unit norm and canonical form (qr >= 0)
    quat_from_rot_x(angle)  Quaternion for a rotation about X axis
    quat_from_rot_y(angle)  Quaternion for a rotation about Y axis
    quat_from_rot_z(angle)  Quaternion for a rotation about Z axis
    random_vehicle_state()  Generate a random VehicleState

Classes:
    Convention              Enum for ENU / NED frame selection
    VehicleState            Dataclass holding the full kinematic state

"""

import math
import random
from dataclasses import dataclass
from enum import Enum


class Convention(Enum):
    ENU = "ENU"
    NED = "NED"


def quat_mult(a, b):
    """Hamilton product of two quaternions (wr,wi,wj,wk)."""
    ar, ai, aj, ak = a
    br, bi, bj, bk = b
    return (
        ar * br - ai * bi - aj * bj - ak * bk,
        ar * bi + ai * br + aj * bk - ak * bj,
        ar * bj - ai * bk + aj * br + ak * bi,
        ar * bk + ai * bj - aj * bi + ak * br,
    )


def quat_conj(q):
    qr, qi, qj, qk = q
    return (qr, -qi, -qj, -qk)


def normalize_quat(q):
    qr, qi, qj, qk = q
    norm = math.sqrt(qr**2 + qi**2 + qj**2 + qk**2)
    qr, qi, qj, qk = qr / norm, qi / norm, qj / norm, qk / norm
    if qr < 0:
        qr, qi, qj, qk = -qr, -qi, -qj, -qk
    return qr, qi, qj, qk


def quat_from_rot_x(angle: float) -> tuple:
    """Quaternion for a rotation of `angle` (rad) about the X axis."""
    return (
        math.cos(angle / 2),
        math.sin(angle / 2),
        0.0,
        0.0,
    )


def quat_from_rot_y(angle: float) -> tuple:
    """Quaternion for a rotation of `angle` (rad) about the Y axis."""
    return (
        math.cos(angle / 2),
        0.0,
        math.sin(angle / 2),
        0.0,
    )


def quat_from_rot_z(angle: float) -> tuple:
    """Quaternion for a rotation of `angle` (rad) about the Z axis."""
    return (
        math.cos(angle / 2),
        0.0,
        0.0,
        math.sin(angle / 2),
    )


# q_R: 180° rotation about (X+Y)/√2 axis — the ENU<->NED frame rotation
# = (cos(90°), sin(90°)/√2, sin(90°)/√2, 0) = (0, 1/√2, 1/√2, 0)
SQRT2_INV = 1.0 / math.sqrt(2.0)
Q_R = (0.0, SQRT2_INV, SQRT2_INV, 0.0)
Q_FLU_FRD = (0.0, 1.0, 0.0, 0.0)


@dataclass
class VehicleState:
    convention: Convention

    # Position
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    # Linear velocities (body frame)
    u: float = 0.0
    v: float = 0.0
    w: float = 0.0

    # Angular velocities (body frame)
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0

    # Quaternion (scalar-first: qr + qi*i + qj*j + qk*k)
    qr: float = 1.0
    qi: float = 0.0
    qj: float = 0.0
    qk: float = 0.0

    def __post_init__(self):
        self.qr, self.qi, self.qj, self.qk = normalize_quat(
            (self.qr, self.qi, self.qj, self.qk)
        )

    @property
    def phi(self) -> float:
        """Roll (rad), ZYX convention."""
        return math.atan2(
            2.0 * (self.qr * self.qi + self.qj * self.qk),
            1.0 - 2.0 * (self.qi**2 + self.qj**2),
        )

    @property
    def theta(self) -> float:
        """Pitch (rad), ZYX convention."""
        sin_pitch = 2.0 * (self.qr * self.qj - self.qk * self.qi)
        return math.asin(max(-1.0, min(1.0, sin_pitch)))

    @property
    def psi(self) -> float:
        """Yaw (rad), ZYX convention."""
        return math.atan2(
            2.0 * (self.qr * self.qk + self.qi * self.qj),
            1.0 - 2.0 * (self.qj**2 + self.qk**2),
        )

    @property
    def x_dot(self) -> float:
        """Fixed-frame velocity along X, derived from quaternion and body velocities."""
        return self._body_to_fixed()[0]

    @property
    def y_dot(self) -> float:
        """Fixed-frame velocity along Y, derived from quaternion and body velocities."""
        return self._body_to_fixed()[1]

    @property
    def z_dot(self) -> float:
        """Fixed-frame velocity along Z, derived from quaternion and body velocities."""
        return self._body_to_fixed()[2]

    def _body_to_fixed(self) -> tuple:
        """Rotate body velocities (u,v,w) to fixed frame via q ⊗ v_body ⊗ q*."""
        q = (self.qr, self.qi, self.qj, self.qk)
        v_body = (0.0, self.u, self.v, self.w)  # pure quaternion
        v_fixed = quat_mult(quat_mult(q, v_body), quat_conj(q))
        # v_fixed is a pure quaternion: (0, x_dot, y_dot, z_dot)
        return v_fixed[1], v_fixed[2], v_fixed[3]

    def convert(self) -> "VehicleState":
        """Return a new VehicleState with the opposite convention (ENU <-> NED)."""

        # Position: x<->y swap, z flips
        new_x = self.y
        new_y = self.x
        new_z = -self.z

        # Body linear velocities: u unchanged, v and w flip
        new_u = self.u
        new_v = -self.v
        new_w = -self.w

        # Body angular velocities: p unchanged, q and r flip
        new_p = self.p
        new_q = -self.q
        new_r = -self.r

        # Quaternion frame change
        q = (self.qr, self.qi, self.qj, self.qk)
        new_q_tuple = quat_mult(quat_mult(Q_R, q), Q_FLU_FRD)

        # This is not sufficient to do this:
        # new_q_tuple = quat_mult(Q_R, q)
        new_qr, new_qi, new_qj, new_qk = normalize_quat(new_q_tuple)

        new_convention = (
            Convention.NED if self.convention == Convention.ENU else Convention.ENU
        )

        return VehicleState(
            convention=new_convention,
            x=new_x,
            y=new_y,
            z=new_z,
            u=new_u,
            v=new_v,
            w=new_w,
            p=new_p,
            q=new_q,
            r=new_r,
            qr=new_qr,
            qi=new_qi,
            qj=new_qj,
            qk=new_qk,
        )

    def __str__(self) -> str:
        return (
            f"VehicleState [{self.convention.value}]\n"
            f"  Position      : x={self.x:.4f},  y={self.y:.4f},  z={self.z:.4f}\n"
            f"  Lin. velocity : u={self.u:.4f},  v={self.v:.4f},  w={self.w:.4f}\n"
            f"  Fixed velocity: x_dot={self.x_dot:.4f}, y_dot={self.y_dot:.4f}, z_dot={self.z_dot:.4f}\n"
            f"  Ang. velocity : p={self.p:.4f},  q={self.q:.4f},  r={self.r:.4f}\n"
            f"  Quaternion    : qr={self.qr:.4f}, qi={self.qi:.4f}, qj={self.qj:.4f}, qk={self.qk:.4f}\n"
            f"  Euler (rad)   : phi={self.phi:.4f}, theta={self.theta:.4f}, psi={self.psi:.4f}\n"
            f"  Euler (deg)   : phi={self.phi*180/math.pi:.4f}, theta={self.theta*180/math.pi:.4f}, psi={self.psi*180/math.pi:.4f}\n"
        )

    def __eq__(self, other: object) -> bool:
        """Compare two VehicleState instances for equality within a numerical tolerance."""
        if not isinstance(other, VehicleState):
            return NotImplemented
        if self.convention != other.convention:
            return False
        tol = 1e-6
        return all(
            abs(a - b) < tol
            for a, b in [
                (self.x, other.x),
                (self.y, other.y),
                (self.z, other.z),
                (self.u, other.u),
                (self.v, other.v),
                (self.w, other.w),
                (self.p, other.p),
                (self.q, other.q),
                (self.r, other.r),
                (self.qr, other.qr),
                (self.qi, other.qi),
                (self.qj, other.qj),
                (self.qk, other.qk),
            ]
        )


def random_vehicle_state(
    convention: Convention = Convention.ENU, seed: int = None
) -> VehicleState:
    """Generate a random VehicleState with realistic vehicle ranges."""
    if seed is not None:
        random.seed(seed)

    def rand(lo, hi):
        return random.uniform(lo, hi)

    # Random Euler angles then build quaternion via ZYX composition
    phi = rand(-math.pi, math.pi)  # roll   [-180°, 180°]
    theta = rand(-math.pi / 2, math.pi / 2)  # pitch  [ -90°,  90°]
    psi = rand(-math.pi, math.pi)  # yaw    [-180°, 180°]

    q = quat_mult(
        quat_mult(quat_from_rot_z(psi), quat_from_rot_y(theta)), quat_from_rot_x(phi)
    )

    return VehicleState(
        convention=convention,
        x=rand(-1000.0, 1000.0),  # m
        y=rand(-1000.0, 1000.0),  # m
        z=rand(0.0, 1000.0),  # m  (positive up in ENU)
        u=rand(-50.0, 50.0),  # m/s
        v=rand(-10.0, 10.0),  # m/s
        w=rand(-5.0, 5.0),  # m/s
        p=rand(-math.pi, math.pi),  # rad/s
        q=rand(-math.pi, math.pi),  # rad/s
        r=rand(-math.pi, math.pi),  # rad/s
        qr=q[0],
        qi=q[1],
        qj=q[2],
        qk=q[3],
    )


def demo00():
    ned_state = VehicleState(
        convention=Convention.NED,
        x=1.0,
        y=2.0,
        z=3.0,
        u=1.0,
        v=0.5,
        w=0.2,
        p=0.1,
        q=0.05,
        r=0.02,
        qr=1.0,
        qi=0.0,
        qj=0.0,
        qk=0.0,
    )

    enu_state = ned_state.convert()
    ned_back = enu_state.convert()

    print(ned_state)
    print(enu_state)
    print("Round-trip back to NED:")
    print(ned_back)
    print(ned_state == ned_back)


def demo01():
    ned_state = VehicleState(
        convention=Convention.NED,
        x=1.0,
        y=2.0,
        z=3.0,
        u=1.0,
        v=0.5,
        w=0.2,
        p=0.1,
        q=0.05,
        r=0.02,
        qr=math.cos(math.pi / 6),
        qi=0.0,
        qj=0.0,
        qk=math.sin(math.pi / 6),
    )

    enu_state = ned_state.convert()
    ned_back = enu_state.convert()

    print(ned_state)
    print(enu_state)
    print("Round-trip back to NED:")
    print(ned_back)
    print(ned_state == ned_back)


def demo03():
    state_enu = random_vehicle_state(Convention.ENU, seed=42)
    state_ned = state_enu.convert()
    assert abs(state_enu.z - (-state_ned.z)) < 1e-10
    assert abs(state_enu.x - (+state_ned.y)) < 1e-10
    assert abs(state_enu.y - (+state_ned.x)) < 1e-10
    assert abs(state_enu.z_dot - (-state_ned.z_dot)) < 1e-10
    assert abs(state_enu.x_dot - (+state_ned.y_dot)) < 1e-10
    assert abs(state_enu.y_dot - (+state_ned.x_dot)) < 1e-10


if __name__ == "__main__":
    demo00()
    demo01()
    demo03()
    quat_ned2enu = quat_mult(quat_from_rot_z(math.pi / 2), quat_from_rot_x(math.pi))
    print(quat_ned2enu)
