#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "${script_dir}/../../../.." && pwd)

# WSLg exposes the Windows GPU through Mesa's D3D12 Gallium driver. On this
# machine Mesa otherwise selects llvmpipe, which makes one simulated second
# take several seconds of wall time once the optical-flow and range sensors run.
if [[ -e /dev/dxg && -f /usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so ]]; then
	export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
	export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
fi

export PX4_GZ_WORLD="${PX4_GZ_WORLD:-v7_four_post_canopy}"

printf 'Gazebo graphics: GALLIUM_DRIVER=%s adapter=%s\n' \
	"${GALLIUM_DRIVER:-automatic}" "${MESA_D3D12_DEFAULT_ADAPTER_NAME:-automatic}"

exec make -C "${repo_root}" px4_sitl gz_x500_cart
