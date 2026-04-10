#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

image_name="${IMAGE_NAME:-vins-rgbd:melodic}"
container_name="${CONTAINER_NAME:-vins-rgbd}"
bag_input="${1:-${BAG_PATH:-/home/mhenwa/slam/bags/Normal.bag}}"
output_dir="${OUTPUT_DIR:-${repo_root}/.docker_output}"
workspace_dir="${WORKSPACE_DIR:-${repo_root}/.docker_catkin_ws}"
host_uid="$(id -u)"
host_gid="$(id -g)"

if [[ -f "${bag_input}" ]]; then
  bag_dir="$(cd "$(dirname "${bag_input}")" && pwd)"
  bag_name="$(basename "${bag_input}")"
elif [[ -d "${bag_input}" ]]; then
  bag_dir="$(cd "${bag_input}" && pwd)"
  bag_name="Normal.bag"
else
  echo "Bag path not found: ${bag_input}" >&2
  exit 1
fi

mkdir -p "${output_dir}"
mkdir -p "${workspace_dir}"

xhost +local: >/dev/null 2>&1 || true

docker_args=(
  --rm
  -it
  --name "${container_name}"
  --network host
  --privileged
  --user "${host_uid}:${host_gid}"
  -e DISPLAY="${DISPLAY:-:0}"
  -e HOME=/tmp/vins-rgbd-home
  -e QT_X11_NO_MITSHM=1
  -e DEFAULT_BAG="/data/${bag_name}"
  -v /tmp/.X11-unix:/tmp/.X11-unix
  -v /dev:/dev
  -v "${repo_root}:/workspace/VINS-RGBD"
  -v "${bag_dir}:/data"
  -v "${output_dir}:/home/shanzy/output"
  -v "${workspace_dir}:/workspace/VINS-RGBD/.docker_catkin_ws"
  -w /workspace/VINS-RGBD
)

if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
  docker_args+=(-e XAUTHORITY="${XAUTHORITY}" -v "${XAUTHORITY}:${XAUTHORITY}")
fi

if [[ -d /dev/dri ]]; then
  docker_args+=(--device /dev/dri:/dev/dri)
fi

echo "Image: ${image_name}"
echo "Container: ${container_name}"
echo "Repo: ${repo_root}"
echo "Bag dir: ${bag_dir}"
echo "Default bag in container: /data/${bag_name}"
echo "Output dir: ${output_dir}"
echo
echo "Container will start in /workspace/VINS-RGBD"

docker run "${docker_args[@]}" "${image_name}" bash
