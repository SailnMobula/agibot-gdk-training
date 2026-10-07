# Source this file on a laptop to work with the real GDK and a robot.
#
#   source real.sh                   Debug cable. The script finds your 10.42.1.x address.
#   source real.sh 192.168.178.75    Wi-Fi or LAN. Pass the address of the robot.
#
# The address can also come from the variable ROBOT_IP. This file replaces the vendor script
# ~/.cache/agibot/app/env.sh, which works over the cable only. Run `python -m gdk_training.doctor`
# afterwards to check the connection.

_gdk_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GDK_APP="${GDK_APP:-$HOME/.cache/agibot/app}"

if [ ! -d "$GDK_APP/gdk/lib/agibot_gdk" ]; then
    echo "real.sh: The GDK is not installed in $GDK_APP. Run scripts/install_gdk.sh [ROBOT_IP] first." >&2
    return 1
fi

# The robot address comes from the argument, then from ROBOT_IP, then from the Debug cable.
if [ -n "${1:-}" ]; then
    ROBOT_IP="$1"
elif [ -z "${ROBOT_IP:-}" ]; then
    if ip -o -4 addr show 2>/dev/null | grep -q ' 10\.42\.1\.'; then
        ROBOT_IP=10.42.1.101
    else
        echo "real.sh: This laptop has no 10.42.1.x address. Pass the address of the robot, source real.sh <ROBOT_IP>." >&2
        return 1
    fi
fi
export ROBOT_IP

# The address of this laptop on the route to the robot. The GDK announces it as LOCATOR_IP.
_gdk_local_ip="$(ip -4 route get "$ROBOT_IP" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -n 1)"
if [ -z "$_gdk_local_ip" ]; then
    echo "real.sh: There is no route to $ROBOT_IP." >&2
    return 1
fi
export LOCATOR_IP="$_gdk_local_ip"
export DEV_IP="$_gdk_local_ip"
export AORTA_DISCOVERY_URI="http://${ROBOT_IP}:2379"
export AORTA_DISPATCHER_THREAD_NUM=6
export APP_CONF_PATH="$GDK_APP/gdk/config/app_conf.json"
# The GDK writes glog files into the current directory by default. Keep them out of the repository.
export GLOG_log_dir="${GDK_LOG_DIR:-$HOME/.cache/agibot/logs}"
mkdir -p "$GLOG_log_dir"

# The native libraries and the Python binding of the GDK package.
_gdk_libs="$(find "$GDK_APP/lib" -type f -name '*.so*' -printf '%h\n' | sort -u | paste -sd: -)"
case ":${LD_LIBRARY_PATH:-}:" in
    *":${_gdk_libs%%:*}:"*) ;;
    *) export LD_LIBRARY_PATH="${_gdk_libs}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
esac
export PYTHONPATH="$GDK_APP/gdk/lib:$_gdk_root"

# The binding is built for CPython 3.10. The environment of this repository uses that version.
[ -f "$_gdk_root/.venv/bin/activate" ] && source "$_gdk_root/.venv/bin/activate"

echo "GDK:    $(sed -n 1p "$GDK_APP/gdk/version")"
echo "Robot:  $ROBOT_IP   this laptop: $LOCATOR_IP   AORTA: $AORTA_DISCOVERY_URI"
echo "Python: $(python --version 2>&1)"
unset _gdk_root _gdk_local_ip _gdk_libs
