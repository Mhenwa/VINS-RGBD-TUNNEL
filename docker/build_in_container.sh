#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/melodic/setup.bash
set -u

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

workspace_dir="${WORKSPACE_DIR:-${repo_root}/.docker_catkin_ws}"
repo_name="${REPO_NAME:-VINS-RGBD}"
jobs="${JOBS:-$(nproc)}"
package_link="${workspace_dir}/src/${repo_name}"
voxblox_repo="${workspace_dir}/src/voxblox"
voxblox_commit="${VOXBLOX_COMMIT:-c8066b04075d2fee509de295346b1c0b788c4f38}"

mkdir -p "${HOME}/.ros"
mkdir -p "${workspace_dir}/src"

if [[ -L "${package_link}" ]]; then
  ln -sfn "${repo_root}" "${package_link}"
elif [[ ! -e "${package_link}" ]]; then
  ln -s "${repo_root}" "${package_link}"
elif [[ "$(cd "${package_link}" && pwd)" != "${repo_root}" ]]; then
  echo "Workspace package path already exists and does not point to ${repo_root}: ${package_link}" >&2
  exit 1
fi

if [[ ! -d "${voxblox_repo}/.git" ]]; then
  git clone https://github.com/ethz-asl/voxblox.git "${voxblox_repo}"
fi

git -C "${voxblox_repo}" fetch --quiet origin master
git -C "${voxblox_repo}" checkout --quiet "${voxblox_commit}"

pushd "${workspace_dir}/src" >/dev/null
if [[ ! -f .rosinstall ]]; then
  wstool init .
fi
if ! grep -q "local-name: catkin_simple" .rosinstall; then
  wstool merge ./voxblox/voxblox_https.rosinstall
fi
wstool update -j"${jobs}"
popd >/dev/null

pushd "${workspace_dir}" >/dev/null
catkin config --extend /opt/ros/melodic --link-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build voxblox_ros camera_model feature_tracker vins_estimator pose_graph -j"${jobs}"
popd >/dev/null

echo
echo "Build complete."
echo "Use the workspace with:"
echo "source ${workspace_dir}/devel/setup.bash"
