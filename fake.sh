# Source this file on a laptop.
# Afterwards agibot_gdk resolves to fake/agibot_gdk.
[ -f .venv/bin/activate ] && source .venv/bin/activate
export PYTHONPATH="$(pwd):$(pwd)/fake"
