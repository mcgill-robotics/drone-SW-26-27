#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
PYTHONPATH="$PWD/src/drone_video${PYTHONPATH:+:$PYTHONPATH}" /usr/bin/python3 -m drone_video.sender --check "$@"
