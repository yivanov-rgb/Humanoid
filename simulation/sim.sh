#!/usr/bin/env bash
set -euo pipefail
sim_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_dir=$(dirname -- "$sim_dir")
image=humanoid-ainex:noetic-gazebo
container=humanoid-ainex-sim
command=${1:-help}
if [[ $# -gt 0 ]]; then shift; fi

fail() {
    printf 'simulation: %s\n' "$*" >&2
    exit 1
}

cleanup_auth_dir() {
    local auth_dir=$1
    local suffix=${auth_dir#"$sim_dir/artifacts/.gui-auth."}
    if [[ "$suffix" == "$auth_dir" || -z "$suffix" || "$suffix" == */* || -L "$auth_dir" ]]; then
        return
    fi
    rm -f -- "$auth_dir/source" "$auth_dir/source-c" "$auth_dir/source-l" \
        "$auth_dir/source-n" "$auth_dir/Xauthority" "$auth_dir/Xauthority-c" \
        "$auth_dir/Xauthority-l" "$auth_dir/Xauthority-n"
    rmdir -- "$auth_dir" 2>/dev/null || true
}

pending_auth_dir=
cleanup_pending_auth() {
    if [[ -n "$pending_auth_dir" ]]; then
        cleanup_auth_dir "$pending_auth_dir"
    fi
}

prepare_gui() {
    local display=${DISPLAY:-}
    local host_authority=${XAUTHORITY:-${HOME:-}/.Xauthority}
    local display_number display_screen x11_socket cookie_entries entry dependency
    [[ "$display" =~ ^(:|unix:)([0-9]+)(\.[0-9]+)?$ ]] || \
        fail 'GUI needs a local X11/XWayland DISPLAY such as :0; remote/TCP displays are not supported.'
    display_number=${BASH_REMATCH[2]}
    display_screen=${BASH_REMATCH[3]:-}
    x11_socket=/tmp/.X11-unix/X${display_number}
    [[ -S "$x11_socket" ]] || fail "X11 socket is unavailable: $x11_socket. Run from an active graphical session."
    [[ -f "$host_authority" && -r "$host_authority" ]] || \
        fail "Xauthority is not readable: $host_authority. Set XAUTHORITY to your graphical session's authority file."
    for dependency in xauth xdpyinfo timeout; do
        command -v "$dependency" >/dev/null 2>&1 || \
            fail "GUI diagnostic requires '$dependency' on the host."
    done

    # Work only on a private copy: xauth may lock its input authority file.
    pending_auth_dir=$(umask 077; mktemp -d "$sim_dir/artifacts/.gui-auth.XXXXXX")
    trap cleanup_pending_auth EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    (umask 077; cp -- "$host_authority" "$pending_auth_dir/source")
    chmod 600 "$pending_auth_dir/source"
    cookie_entries=$(xauth -f "$pending_auth_dir/source" nlist "$display") || \
        fail 'Could not read the selected display cookie from the private Xauthority copy.'
    [[ -n "$cookie_entries" ]] || fail "No Xauthority cookie matches DISPLAY=$display."
    (umask 077; : > "$pending_auth_dir/Xauthority")
    # FamilyWild avoids a host/container hostname mismatch; the cookie still
    # authenticates only the selected display. No xhost grants are needed.
    while IFS= read -r entry; do
        printf 'ffff%s\n' "${entry:4}"
    done <<< "$cookie_entries" | xauth -f "$pending_auth_dir/Xauthority" nmerge -
    unset cookie_entries entry
    rm -f -- "$pending_auth_dir/source"
    if ! DISPLAY=":${display_number}${display_screen}" \
        XAUTHORITY="$pending_auth_dir/Xauthority" timeout 5s xdpyinfo >/dev/null 2>&1; then
        fail 'Cannot connect to the local display with its private cookie. Check that your graphical session is active and DISPLAY/XAUTHORITY match.'
    fi
    options+=(--env "DISPLAY=:${display_number}${display_screen}"
        --env XAUTHORITY=/tmp/robotrio.Xauthority
        --mount "type=bind,src=$x11_socket,dst=$x11_socket,readonly"
        --mount "type=bind,src=$pending_auth_dir/Xauthority,dst=/tmp/robotrio.Xauthority,readonly"
        --label "robotrio.xauthority-dir=$pending_auth_dir")
}

case "$command" in
    fetch)
        exec bash "$sim_dir/fetch_vendor.sh"
        ;;
    build)
        bash "$sim_dir/fetch_vendor.sh"
        cd "$project_dir"
        docker build --pull -f simulation/Dockerfile -t "$image" .
        ;;
    smoke|up|gui)
        gui=false
        if [[ "$command" == gui ]]; then gui=true; fi
        anchored=${ROBOTRIO_ANCHORED:-false}
        if [[ "$command" == smoke ]]; then anchored=true; fi
        [[ "$anchored" == true || "$anchored" == false ]] || \
            fail 'ROBOTRIO_ANCHORED must be true or false.'
        actions_dir=${ROBOTRIO_ACTIONS_DIR:-"$sim_dir/actiongroups"}
        while [[ $# -gt 0 ]]; do
            case "$1" in
                --gui)
                    [[ "$command" != smoke ]] || fail 'smoke is headless; use gui or up --gui for Gazebo GUI.'
                    gui=true
                    shift
                    ;;
                --free)
                    [[ "$command" != smoke ]] || fail 'smoke uses the fixed inspection model; --free is available with up or gui.'
                    anchored=false
                    shift
                    ;;
                --anchored)
                    anchored=true
                    shift
                    ;;
                --actions-dir)
                    [[ $# -ge 2 && -n "$2" ]] || fail '--actions-dir needs a directory path.'
                    actions_dir=$2
                    shift 2
                    ;;
                --actions-dir=*)
                    actions_dir=${1#*=}
                    [[ -n "$actions_dir" ]] || fail '--actions-dir needs a directory path.'
                    shift
                    ;;
                *) fail "Unknown startup option: $1. Use --gui, --free, --anchored and/or --actions-dir PATH." ;;
            esac
        done
        if [[ "$actions_dir" == "$sim_dir/actiongroups" ]]; then
            mkdir -p "$actions_dir"
        fi
        [[ -d "$actions_dir" && -r "$actions_dir" && -x "$actions_dir" ]] || \
            fail "ActionGroups directory is not readable: $actions_dir. Mount an existing .d6a directory with --actions-dir PATH."
        actions_dir=$(cd -- "$actions_dir" && pwd -P)
        [[ "$actions_dir" != *,* ]] || fail 'ActionGroups directory paths containing commas are not supported by Docker bind mounts.'
        mkdir -p "$sim_dir/artifacts"
        options=(--network none --shm-size=512m --init
            --mount "type=bind,src=$project_dir,dst=/workspace,readonly"
            --mount "type=bind,src=$sim_dir/ros_ws/src/robotrio_sim,dst=/opt/ainex_ws/src/robotrio_sim,readonly"
            --mount "type=bind,src=$actions_dir,dst=/actions,readonly"
            --mount "type=bind,src=$sim_dir/artifacts,dst=/workspace/simulation/artifacts")
        launch_args=("anchored:=$anchored")
        if [[ "$command" != smoke ]]; then
            printf 'Gazebo mode: %s\n' "$([[ "$anchored" == true ]] && printf 'ANCHORED (body fixed at 0.40 m)' || printf 'FREE (preparing and checking initial stand)')"
        fi
        if [[ -n "${ROBOTRIO_SERVO_CONFIG:-}" ]]; then
            servo_config=$ROBOTRIO_SERVO_CONFIG
            [[ "$servo_config" == /* && -f "$servo_config" && -r "$servo_config" ]] || \
                fail 'ROBOTRIO_SERVO_CONFIG must name an existing readable absolute servo_controller.yaml path.'
            [[ "$servo_config" != *,* ]] || fail 'Servo config paths containing commas are not supported by Docker bind mounts.'
            options+=(--mount "type=bind,src=$servo_config,dst=/config/servo_controller.yaml,readonly"
                --env ROBOTRIO_SERVO_CONFIG=/config/servo_controller.yaml)
            launch_args+=(servo_config:=/config/servo_controller.yaml)
        fi
        if [[ "$command" == smoke ]]; then
            exec docker run --rm --name humanoid-ainex-smoke "${options[@]}" "$image" /opt/ainex-run-smoke.sh
        elif [[ "$gui" == true ]]; then
            prepare_gui
            docker run -d --name "$container" "${options[@]}" "$image" \
                roslaunch robotrio_sim ainex.launch gui:=true "${launch_args[@]}"
            # Keep this cookie until stop: Docker may need its bind source again.
            pending_auth_dir=
        else
            exec docker run -d --name "$container" "${options[@]}" "$image" \
                xvfb-run -a -s '-screen 0 1280x720x24' \
                roslaunch robotrio_sim ainex.launch gui:=false "${launch_args[@]}"
        fi
        ;;
    shell)
        exec docker exec -it "$container" bash
        ;;
    logs)
        exec docker logs --tail 100 "$container"
        ;;
    status)
        exec docker exec "$container" /opt/ainex-entrypoint.sh rosparam get /robotrio_sim
        ;;
    action|list-actions|action-info|stop-action)
        case "$command" in
            action) action_command=play ;;
            list-actions) action_command=list ;;
            action-info) action_command=info ;;
            stop-action) action_command=stop ;;
        esac
        exec docker exec "$container" /opt/ainex-entrypoint.sh \
            rosrun robotrio_sim action_cli.py "$action_command" "$@"
        ;;
    line-follow)
        # A TTY forwards Ctrl+C to this client. Observation is its default.
        terminal_options=()
        if [[ -t 0 && -t 1 ]]; then terminal_options=(-it); fi
        exec docker exec "${terminal_options[@]}" "$container" /opt/ainex-entrypoint.sh \
            python /workspace/line_follow.py --backend sim "$@"
        ;;
    line-stop)
        exec docker exec "$container" /opt/ainex-entrypoint.sh \
            rosservice call /robotrio_line_follow/stop '{}'
        ;;
    line-view)
        [[ $# -le 1 ]] || fail 'line-view accepts at most one image topic.'
        exec docker exec "$container" /opt/ainex-entrypoint.sh \
            rosrun rqt_image_view rqt_image_view "${1:-/line_follow/image}"
        ;;
    stop)
        auth_dir=$(docker inspect --format '{{ index .Config.Labels "robotrio.xauthority-dir" }}' "$container")
        docker stop --time 15 "$container"
        docker rm "$container"
        if [[ -n "$auth_dir" ]]; then cleanup_auth_dir "$auth_dir"; fi
        ;;
    *)
        printf 'Usage: bash simulation/sim.sh {fetch|build|smoke|up|gui|shell|logs|status|action|list-actions|action-info|stop-action|line-follow|line-stop|line-view|stop}\n'
        printf '  up [--gui] [--anchored] [--actions-dir PATH]  Start free Gazebo (headless by default).\n'
        printf '  gui [--anchored] [--actions-dir PATH]        Start free Gazebo with local GUI.\n'
        printf '  ROBOTRIO_ACTIONS_DIR sets the default .d6a directory (otherwise simulation/actiongroups).\n'
        printf '  --anchored explicitly fixes the body above ground; --free explicitly removes the fixture.\n'
        printf '  status prints initial_pose_ready/error; free actions wait for verified stand.\n'
        printf '  ROBOTRIO_SERVO_CONFIG optionally mounts an absolute servo_controller.yaml path read-only.\n'
        printf '  action ARGS... | action-info ARGS... | list-actions | stop-action\n'
        printf '  line-follow ARGS...  Observe camera; --execute --empty-hands --image-orientation standard enables actions.\n'
        printf '  line-stop            Stop the route, including between individual actions.\n'
        printf '  line-view [TOPIC]     Open camera viewer (default /line_follow/image).\n'
        ;;
esac
