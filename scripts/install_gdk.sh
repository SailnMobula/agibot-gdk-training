#!/usr/bin/env bash
# Installs the AGIBOT GDK package for laptops. The robot serves the package itself.
#
# Usage:  scripts/install_gdk.sh [ROBOT_IP]
#   ROBOT_IP   Address of the robot. The default is 10.42.1.101, the address on the Debug cable.
#   GDK_HOME   Install root. The default is ~/.cache/agibot, and the package goes to $GDK_HOME/app.
#
# The script does the same steps as the vendor command `curl http://10.42.1.101:8849/install.sh | bash`.
# In addition it takes the robot address as a parameter, resumes an interrupted download, checks the
# archive before it replaces anything and keeps the previous install until the new one is ready.
set -euo pipefail

ROBOT_IP="${1:-${ROBOT_IP:-10.42.1.101}}"
GDK_HOME="${GDK_HOME:-$HOME/.cache/agibot}"
BASE="http://${ROBOT_IP}:8849"
ARCHIVE="${GDK_HOME}/server_installer.tar.gz"
STAGE="${GDK_HOME}/stage"

echo "Robot:    ${ROBOT_IP}"
echo "Target:   ${GDK_HOME}/app"

curl -fsS -m 8 -o /dev/null "${BASE}/gdk_ros_domain.xml" \
    || { echo "ERROR: ${BASE} is not reachable. Check the cable address or the Wi-Fi/LAN route." >&2; exit 1; }

mkdir -p "${GDK_HOME}"
echo "Downloading server_installer.tar.gz (about 220 MB, resumes if interrupted) ..."
curl -fL --progress-bar --retry 10 --retry-delay 2 --retry-all-errors -C - -o "${ARCHIVE}" \
    "${BASE}/server_installer.tar.gz"

echo "Verifying archive ..."
gzip -t "${ARCHIVE}"
sha256sum "${ARCHIVE}"

rm -rf "${STAGE}"
mkdir -p "${STAGE}"
tar -xzf "${ARCHIVE}" -C "${STAGE}"
[ -d "${STAGE}/app/gdk/lib/agibot_gdk" ] || { echo "ERROR: archive has no app/gdk/lib/agibot_gdk" >&2; exit 1; }

mkdir -p "${STAGE}/app/conf/dds"
curl -fsS -o "${STAGE}/app/conf/dds/gdk_ros_domain.xml" "${BASE}/gdk_ros_domain.xml"

rm -rf "${GDK_HOME}/app"
mv "${STAGE}/app" "${GDK_HOME}/app"
rmdir "${STAGE}"
rm -f "${ARCHIVE}"

echo
echo "Installed GDK:"
sed -n 1,2p "${GDK_HOME}/app/gdk/version"
ls "${GDK_HOME}/app/gdk/lib/agibot_gdk" | grep '\.so$'
echo "The binding is built for the Python version in its file name. Use that version, see README.md."
