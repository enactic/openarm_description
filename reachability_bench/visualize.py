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

import time
from pathlib import Path
from typing import Literal

import matplotlib.pyplot as plt
import numpy as np
import tyro
from plainmp.robot_spec import OpenArmV10RarmSpec
from skrobot.model.primitives import Box
from skrobot.viewers import PyrenderViewer

from bench import OpenArmV12RarmSpec


def main(
    htable: float = 0.1,
    hmin: float | None = None,
    hmax: float | None = None,
    animate: bool = False,
    direction: Literal["forward1", "forward2", "down1", "down2"] = "forward1",
    use_v12: bool = False,
):
    feasible_pointss = []
    qss = []
    if hmin is None:
        hmin = -np.inf
    if hmax is None:
        hmax = np.inf
    z_point_max = -np.inf

    if use_v12:
        result_dir = Path("result_v12")
    else:
        result_dir = Path("result_v10")

    for result_path in result_dir.iterdir():
        if not str(result_path.stem).startswith(direction):
            continue
        npz = np.load(result_path, allow_pickle=True)
        dic = npz["arr_0"].item()
        z = dic["z"]
        bools = dic["pts"]
        qs = dic["qs"]
        z_point_max = max(z_point_max, z)
        if z > htable and z > hmin and z < hmax:
            if len(bools) > 0:
                feasible_pointss.append([z, bools])
                qss.append(qs)

    qs = np.vstack(qss)

    v = PyrenderViewer()
    if use_v12:
        spec = OpenArmV12RarmSpec()
    else:
        spec = OpenArmV10RarmSpec()
    model = spec.get_robot_model(with_mesh=True)
    v.add(model)
    table_thickness = 0.03
    table = Box([0.4, 1.0, table_thickness], face_colors=[0.65, 0.45, 0.0, 1.0])
    table.translate([0.3, 0.0, htable - table_thickness * 0.5])
    v.add(table)

    cmap = plt.get_cmap("jet")

    for z, points in feasible_pointss:
        for p_2d in points:
            p_3d = np.hstack([p_2d, z])
            size = 0.01
            s = (z - htable) / (z_point_max - htable)
            color = cmap(s)
            box = Box([size, size, size], face_colors=color)
            box.translate(p_3d)
            v.add(box)
    v.show()

    if animate:
        input("press enter to start animate")
        for q in qs:
            spec.set_skrobot_model_state(model, q)
            v.redraw()
            time.sleep(0.5)
    time.sleep(1000)


if __name__ == "__main__":
    tyro.cli(main)
