# Source this file on the robot itself.
# It loads the GDK environment of the robot and adds this repository to the Python path.
_gdk_root="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
source /home/agi/app/env.sh > /dev/null
export PYTHONPATH="$_gdk_root:$PYTHONPATH"
unset _gdk_root
