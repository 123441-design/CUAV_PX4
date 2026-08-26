#!/usr/bin/env bash

set -uo pipefail

if [[ $# -lt 1 ]]; then
	echo "usage: sitl_run.sh <px4-binary> [arguments...]" >&2
	exit 2
fi

gz_pid_file=$(mktemp "${TMPDIR:-/tmp}/px4-gz-pids.XXXXXX")

cleanup_gazebo()
{
	if [[ -f "${gz_pid_file}" ]]; then
		while IFS= read -r process_id; do
			if [[ ! "${process_id}" =~ ^[0-9]+$ ]] || [[ ! -r "/proc/${process_id}/cmdline" ]]; then
				continue
			fi

			process_command=$(tr '\0' ' ' < "/proc/${process_id}/cmdline")

			if [[ "${process_command}" == *"gz sim"* ]]; then
				kill "${process_id}" 2>/dev/null || true
			fi
		done < "${gz_pid_file}"

		rm -f -- "${gz_pid_file}"
	fi
}

trap cleanup_gazebo EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

export PX4_GZ_PID_FILE="${gz_pid_file}"
"$@"
