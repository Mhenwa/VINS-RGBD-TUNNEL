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

pushd "${workspace_dir}" >/dev/null
catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build camera_model feature_tracker vins_estimator pose_graph -j"${jobs}"
popd >/dev/null

echo
echo "Build complete."
echo "Use the workspace with:"
echo "source ${workspace_dir}/devel/setup.bash"
