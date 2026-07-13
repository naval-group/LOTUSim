from dataclasses import dataclass
from enum import Enum
import math


class Convention(Enum):
    ENU = "ENU"
    NED = "NED"


def quat_mult(a, b):
    """Hamilton product of two quaternions (wr,wi,wj,wk)."""
    ar, ai, aj, ak = a
    br, bi, bj, bk = b
    return (
        ar*br - ai*bi - aj*bj - ak*bk,
        ar*bi + ai*br + aj*bk - ak*bj,
        ar*bj - ai*bk + aj*br + ak*bi,
        ar*bk + ai*bj - aj*bi + ak*br,
    )


def quat_conj(q):
    qr, qi, qj, qk = q
    return (qr, -qi, -qj, -qk)


def normalize_quat(q):
    qr, qi, qj, qk = q
    norm = math.sqrt(qr**2 + qi**2 + qj**2 + qk**2)
    qr, qi, qj, qk = qr/norm, qi/norm, qj/norm, qk/norm
    if qr < 0:
        qr, qi, qj, qk = -qr, -qi, -qj, -qk
    return qr, qi, qj, qk


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
            1.0 - 2.0 * (self.qi**2 + self.qj**2)
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
            1.0 - 2.0 * (self.qj**2 + self.qk**2)
        )

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

        # Quaternion frame change: q_new = q_R ⊗ q ⊗ q_R*
        q = (self.qr, self.qi, self.qj, self.qk)
        # new_q_tuple = quat_mult(quat_mult(Q_R, q), Q_R_CONJ)
        # new_q_tuple = quat_mult(quat_mult(Q_R, q), Q_FLU_FRD)
        
        new_q_tuple = quat_mult(Q_R, q)
        new_qr, new_qi, new_qj, new_qk = normalize_quat(new_q_tuple)

        new_convention = Convention.NED if self.convention == Convention.ENU else Convention.ENU

        return VehicleState(
            convention=new_convention,
            x=new_x, y=new_y, z=new_z,
            u=new_u, v=new_v, w=new_w,
            p=new_p, q=new_q, r=new_r,
            qr=new_qr, qi=new_qi, qj=new_qj, qk=new_qk,
        )

    def __str__(self) -> str:
        return (
            f"VehicleState [{self.convention.value}]\n"
            f"  Position      : x={self.x:.4f},  y={self.y:.4f},  z={self.z:.4f}\n"
            f"  Lin. velocity : u={self.u:.4f},  v={self.v:.4f},  w={self.w:.4f}\n"
            f"  Ang. velocity : p={self.p:.4f},  q={self.q:.4f},  r={self.r:.4f}\n"
            f"  Quaternion    : qr={self.qr:.4f}, qi={self.qi:.4f}, qj={self.qj:.4f}, qk={self.qk:.4f}\n"
            f"  Euler (rad)   : phi={self.phi:.4f}, theta={self.theta:.4f}, psi={self.psi:.4f}\n"
            f"  Euler (deg)   : phi={self.phi*180/math.pi:.4f}, theta={self.theta*180/math.pi:.4f}, psi={self.psi*180/math.pi:.4f}\n"
        )


if __name__ == "__main__":
    ned_state = VehicleState(
        convention=Convention.NED,
        x=1.0, y=2.0, z=3.0,
        u=1.0, v=0.5, w=0.2,
        p=0.1, q=0.05, r=0.02,
        qr=math.cos(math.pi/6), qi=0.0, qj=0.0, qk=math.sin(math.pi/6),
    )

    enu_state = ned_state.convert()
    ned_back  = enu_state.convert()

    print(ned_state)
    print(enu_state)
    print("Round-trip back to NED:")
    print(ned_back)