#!/bin/bash
#
# Copyright 2026 Enactic, Inc.
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

# Regenerate the checked-in example URDFs from their Xacro sources.
#
# xacro must be able to resolve $(find openarm_description), e.g. by
# sourcing a workspace where this package is built.

set -eu

base_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "${base_dir}/.."

xacro \
  assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro \
  -o assets/robot/openarm_v1.0/urdf/example/v1.urdf
xacro \
  assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro \
  -o assets/robot/openarm_v2.0/urdf/example/v2.urdf
