from dataclasses import dataclass
import numpy as np
from numpy.linalg import norm

from dataclasses import dataclass
from torch import Tensor
from typing import Union, Sequence
import math

import torch


# --- Helpers ----------------------------------------------------------
def axis_angle_to_R(axis: Union[Tensor, Sequence[float]],
                    angle: Union[float, Tensor]) -> Tensor:
    """
    Rodrigues' rotation formula implemented in pure PyTorch.

    axis  : (3,)  – rotation axis (need not be unit-length)
    angle : scalar (rad)

    Returns
    -------
    R     : (3,3) rotation matrix, same dtype & device as inputs
    """
    axis  = torch.as_tensor(axis, dtype=torch.float32, device='cuda' if torch.cuda.is_available() else 'cpu')
    angle = torch.as_tensor(angle, dtype=axis.dtype, device=axis.device)

    axis = axis / torch.linalg.norm(axis)
    x, y, z = axis
    c, s = torch.cos(angle), torch.sin(angle)
    C = 1.0 - c

    R = torch.stack([
        torch.stack([c + x*x*C,     x*y*C - z*s, x*z*C + y*s]),
        torch.stack([y*x*C + z*s, c + y*y*C,     y*z*C - x*s]),
        torch.stack([z*x*C - y*s, z*y*C + x*s, c + z*z*C    ])
    ])
    return R


# --- Pose class -------------------------------------------------------
@dataclass
class Pose:
    p: Tensor          # (3,) translation
    R: Tensor          # (3,3) rotation matrix

    @property
    def T(self) -> Tensor:
        """4×4 homogeneous transform (same device / dtype)."""
        eye = torch.eye(4, dtype=self.R.dtype, device=self.R.device)
        eye[:3, :3] = self.R
        eye[:3, 3]  = self.p
        return eye

    def transform_point(self, q_local: Tensor) -> Tensor:
        """Transform point from local to world frame."""
        return self.R @ q_local + self.p


# --- Example usage ----------------------------------------------------
device = 'cuda' if torch.cuda.is_available() else 'cpu'
translation = torch.tensor([0.3, 0.1, 0.2], device=device)
rotation    = axis_angle_to_R([0, 0, 1], math.pi / 2)

ee_pose = Pose(translation, rotation)

print("Homogeneous transform T =\n", ee_pose.T)

q_local = torch.tensor([0.05, 0.00, 0.00], device=device)
q_world = ee_pose.transform_point(q_local)
print("Point in world frame:", q_world)


