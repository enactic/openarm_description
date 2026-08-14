# Copyright 2025 SB Intuitions Corp.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from pathlib import Path
from typing import Literal

import numpy as np
import tqdm
import tyro
from plainmp.constraint import LinkPoseCst
from plainmp.ik import solve_ik_srinv
from plainmp.robot_spec import OpenArmV10RarmSpec
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import matrix2quaternion, wxyz2xyzw
from threadpoolctl import threadpool_limits


class OpenArmV12RarmSpec(OpenArmV10RarmSpec):
    def urdf_path_override(self) -> Path:
        urdf_path = Path("~/scripts/openarm.urdf").expanduser()
        return urdf_path

    def _convert_ros_style_to_openarm_style(self, co: Coordinates) -> Coordinates:
        # NOTE: openarm's tcp is different from the ROS-standard definition
        # we takes co (ROS-standard) and convert it to openarm tcp
        co = co.copy_worldcoords()
        co.rotate(+np.pi * 0.5, "x")
        co.rotate(+np.pi * 0.5, "z")
        co.translate([0.0, +0.18, 0.0])
        return co

    def create_tcp_pose_const(self, co: Coordinates) -> LinkPoseCst:
        co = self._convert_ros_style_to_openarm_style(co)
        target = np.hstack([co.worldpos(), wxyz2xyzw(matrix2quaternion(co.worldrot()))])
        return self.create_pose_const(["openarm_right_link7"], [target])


class PlainmpIKSolver:
    def __init__(self, use_v12: bool = False):
        if use_v12:
            self.spec = OpenArmV12RarmSpec()
        else:
            self.spec = OpenArmV10RarmSpec()

    def solve(self, tcp_coords_ros: Coordinates, n_global_iter: int) -> np.ndarray | None:
        cst = self.spec.create_tcp_pose_const(tcp_coords_ros)
        lb, ub = self.spec.angle_bounds()
        ret = solve_ik_srinv(cst, lb, ub, q_seed=None, max_trial=n_global_iter)
        if ret.success:
            return ret.q
        return None


def benchmark(
    z: float,
    use_v12: bool = False,
    direction: Literal["forward1", "forward2", "down1", "down2"] = "forward1",
):
    if direction == "forward1":
        co_base = Coordinates()
    elif direction == "forward2":
        co_base = Coordinates()
        co_base.rotate(-np.pi * 0.5, "x")
    elif direction == "down1":
        co_base = Coordinates()
        co_base.rotate(np.pi * 0.5, "y")
    elif direction == "down2":
        co_base = Coordinates()
        co_base.rotate(np.pi * 0.5, "y")
        co_base.rotate(-np.pi * 0.5, "x")
    else:
        assert False

    if use_v12:
        result_dir = Path(__file__).parent / "result_v12"
    else:
        result_dir = Path(__file__).parent / "result_v10"
    result_dir.mkdir(exist_ok=True)

    xlin = np.linspace(0.0, 0.8, 20)
    ylin = np.linspace(-0.5, 0.5, 20)
    X, Y = np.meshgrid(xlin, ylin)
    points2d = np.stack([X.ravel(), Y.ravel()], axis=1)
    solver = PlainmpIKSolver(use_v12)

    with threadpool_limits(limits=1, user_api="blas"):
        qs_reachable = []
        labels = []
        for point2d in tqdm.tqdm(points2d):
            point3d = np.hstack([point2d, z])
            co = co_base.copy_worldcoords()
            co.translate(point3d, wrt="world")
            ret = solver.solve(co, 30)
            success = ret is not None
            labels.append(success)
            if success:
                qs_reachable.append(ret)
        is_reachables = np.array(labels)
        qs_reachable = np.array(qs_reachable)
        pts_reachable = points2d[is_reachables]

        d = {"z": z, "pts": pts_reachable, "qs": qs_reachable}
        with (result_dir / f"{direction}-z-{round(z, 2)}.npz").open(mode="wb") as f:
            np.savez(f, d)


if __name__ == "__main__":
    tyro.cli(benchmark)
