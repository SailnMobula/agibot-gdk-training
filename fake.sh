# Source this file to work without a robot.
# Afterwards agibot_gdk resolves to fake/agibot_gdk.
_gdk_root="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
[ -f "$_gdk_root/.venv/bin/activate" ] && source "$_gdk_root/.venv/bin/activate"
export PYTHONPATH="$_gdk_root:$_gdk_root/fake"
unset _gdk_root
