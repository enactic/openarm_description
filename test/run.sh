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

set -eu

if [ $# -ne 1 ]; then
    echo "Usage: $0 build_dir"
    echo " e.g.: $0 /tmp/openarm_description.build"
    exit 1
fi

source_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$1"

# Build out of the source tree so that build/, install/ and log/
# aren't created in the source tree. pytest also writes
# test/__pycache__/ without PYTHONDONTWRITEBYTECODE.
export PYTHONDONTWRITEBYTECODE=1
mkdir -p "${build_dir}"
cd "${build_dir}"

build_paths=("${source_dir}")
if [ -f "${source_dir}/.ci/zed_description/package.xml" ]; then
    build_paths+=("${source_dir}/.ci/zed_description")
fi
colcon build --paths "${build_paths[@]}"
set +u
source install/setup.bash
set -u
# Run only our pytest tests. ament_lint_auto also registers
# ament_lint_common linters but we use pre-commit for lint.
colcon test \
    --base-paths "${source_dir}" \
    --event-handlers console_direct+ \
    --ctest-args -L pytest
colcon test-result --verbose
