# Source this file to work with the real robot.
# It loads the GDK environment and adds this folder to the Python path.
# The GDK is in ~/.cache/agibot/app on a development PC and in /home/agi/app on the robot.
# Set GDK_ENV to use an env.sh in another place.
if [ -z "$GDK_ENV" ]; then
  if [ -f "$HOME/.cache/agibot/app/env.sh" ]; then
    GDK_ENV="$HOME/.cache/agibot/app/env.sh"
  else
    GDK_ENV="/home/agi/app/env.sh"
  fi
fi
if [ ! -f "$GDK_ENV" ]; then
  echo "No GDK environment found at $GDK_ENV. Install the GDK first, see README.md."
  return 1
fi
source "$GDK_ENV" > /dev/null
export PYTHONPATH="$(pwd):$PYTHONPATH"
echo "GDK environment loaded from $GDK_ENV"

# The GDK reaches the robot only through the 10.42.1.x network of the Debug port.
if ! ip -o -4 addr list 2> /dev/null | grep -q "10\.42\.1\."; then
  echo "WARNING: This machine has no address in 10.42.1.x. The GDK cannot reach the robot."
  echo "         Connect the Ethernet cable and set a static address, for example 10.42.1.102."
fi

# The prebuilt GDK module works with one Python version only.
gdk_module=$(ls "$(dirname "$GDK_ENV")"/gdk/lib/agibot_gdk/agibot_gdk.cpython-*.so 2> /dev/null | head -n 1)
gdk_python=$(echo "$gdk_module" | sed -n 's/.*cpython-\([0-9]\)\([0-9]*\)-.*/\1.\2/p')
this_python=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
if [ -n "$gdk_python" ] && [ "$gdk_python" != "$this_python" ]; then
  echo "WARNING: The GDK is built for Python $gdk_python, but python3 is Python $this_python."
  echo "         Use a Python $gdk_python interpreter, see README.md."
fi
unset gdk_module gdk_python this_python
