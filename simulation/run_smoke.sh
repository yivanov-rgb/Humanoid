#!/usr/bin/env bash
set -euo pipefail
report=/workspace/simulation/artifacts/smoke.json
mkdir -p "$(dirname -- "$report")"
python - "$report" <<'PY'
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
Path(sys.argv[1]).write_text(json.dumps({
    'ok': False, 'state': 'starting',
    'started_at_utc': datetime.now(timezone.utc).isoformat(),
    'error': 'Smoke has not completed. Check the command exit status and gazebo.log.'
}, indent=2) + '\n')
PY
log=/workspace/simulation/artifacts/gazebo.log
launch_args=(gui:=false anchored:=true)
if [[ -n "${ROBOTRIO_SERVO_CONFIG:-}" ]]; then
    launch_args+=("servo_config:=$ROBOTRIO_SERVO_CONFIG")
fi
setsid xvfb-run -a -s '-screen 0 1280x720x24' \
    roslaunch robotrio_sim ainex.launch "${launch_args[@]}" > "$log" 2>&1 &
launch_pid=$!
cleanup() {
    # Terminate the simulation process group, not just the xvfb-run wrapper.
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    for attempt in {1..50}; do
        if ! kill -0 "$launch_pid" 2>/dev/null; then break; fi
        sleep 0.1
    done
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
rosrun robotrio_sim smoke.py --output "$report"
