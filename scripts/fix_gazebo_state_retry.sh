#!/usr/bin/env bash
# fix_gazebo_state_retry.sh
#
# Patches coursework_world_spawner.py (both the installed copy and the source
# copy) to add retry logic so that get_model_state_by_name and
# get_model_state_via_gz wait for Gazebo to fully register a freshly-spawned
# entity before giving up.
#
# Run once after cloning / rebuilding:
#   bash ~/ros2_ws/src/comp0250_s26_labs/src/courseworks/cw2_team_20/scripts/fix_gazebo_state_retry.sh
#
# Environment variables (optional overrides):
#   ROS2_WS        — path to the ROS 2 workspace (default: ~/ros2_ws)
#   PYTHON_VERSION — e.g. "3.10" (default: auto-detected)
#
# No colcon rebuild is needed for cw2_world_spawner after running this script
# (it is a pure-Python package).  You DO need to rebuild cw2_team_20 if you
# renamed the package (see README.md).

set -euo pipefail

# --- Configurable paths --------------------------------------------------- #
ROS2_WS="${ROS2_WS:-${HOME}/ros2_ws}"

if [[ -z "${PYTHON_VERSION:-}" ]]; then
    PYTHON_VERSION="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
fi

INSTALL_FILE="${ROS2_WS}/install/cw2_world_spawner/local/lib/python${PYTHON_VERSION}/dist-packages/cw2_world_spawner_lib/coursework_world_spawner.py"
SOURCE_FILE="${ROS2_WS}/src/comp0250_s26_labs/src/courseworks/cw2_world_spawner/src/cw2_world_spawner_lib/coursework_world_spawner.py"
# -------------------------------------------------------------------------- #

patch_file() {
    local TARGET="$1"

    if [[ ! -f "$TARGET" ]]; then
        echo "[SKIP] File not found: $TARGET"
        return
    fi

    python3 << PYEOF
import sys

path = "$TARGET"

with open(path, 'r') as fh:
    src = fh.read()

# ------------------------------------------------------------------ #
# 1. Ensure 'import time' is present (needed for time.sleep calls)   #
# ------------------------------------------------------------------ #
if 'import time' not in src:
    # Insert after the first import line
    first_import = src.find('\nimport ')
    if first_import == -1:
        first_import = src.find('\nfrom ')
    if first_import != -1:
        src = src[:first_import + 1] + 'import time\n' + src[first_import + 1:]
    else:
        src = 'import time\n' + src
    print(f"[OK]   Added 'import time' to {path}")
else:
    print(f"[OK]   'import time' already present in {path}")

# ------------------------------------------------------------------ #
# 2. Patch get_model_state_by_name                                    #
# ------------------------------------------------------------------ #
OLD_BY_NAME = \
  '  def get_model_state_by_name(self, name, relname="world"):\n' \
  '    if self.get_state_client.wait_for_service(timeout_sec=0.2):\n' \
  '      request = GetEntityState.Request()\n' \
  '      request.name = name\n' \
  '      request.reference_frame = relname\n' \
  '      return call_service_sync(self.client_node, self.get_state_client, request, timeout_sec=5.0)\n' \
  '\n' \
  '    # Gazebo Classic on ROS 2 Humble does not always expose get_entity_state\n' \
  '    # reliably. Fall back to \`gz model -i\` so coursework task setup can still\n' \
  '    # query spawned object poses.\n' \
  '    return self.get_model_state_via_gz(name)'

NEW_BY_NAME = \
  '  def get_model_state_by_name(self, name, relname="world"):\n' \
  '    # Wait up to 30s for service availability\n' \
  '    if not self.get_state_client.wait_for_service(timeout_sec=30.0):\n' \
  '      return self.get_model_state_via_gz(name)\n' \
  '\n' \
  '    request = GetEntityState.Request()\n' \
  '    request.name = name\n' \
  '    request.reference_frame = relname\n' \
  '\n' \
  '    # Retry up to 30 times with 1s sleep — entity may not be registered in Gazebo immediately after spawn\n' \
  '    max_attempts = 30\n' \
  '    for attempt in range(max_attempts):\n' \
  "      resp = call_service_sync(self.client_node, self.get_state_client, request, timeout_sec=5.0)\n" \
  "      if resp is not None and getattr(resp, 'success', False):\n" \
  '        return resp\n' \
  '      self.node.get_logger().warn(\n' \
  "        f\"get_entity_state for '{name}' attempt {attempt+1}/{max_attempts} failed, retrying in 1s...\")\n" \
  '      time.sleep(1.0)\n' \
  '\n' \
  '    return self.get_model_state_via_gz(name)'

if OLD_BY_NAME in src:
    src = src.replace(OLD_BY_NAME, NEW_BY_NAME, 1)
    print(f"[OK]   Patched get_model_state_by_name in {path}")
elif NEW_BY_NAME in src:
    print(f"[SKIP] get_model_state_by_name already patched in {path}")
else:
    print(f"[WARN] Could not find get_model_state_by_name pattern in {path} — skipping.")

# ------------------------------------------------------------------ #
# 3. Patch get_model_state_via_gz                                     #
# ------------------------------------------------------------------ #
OLD_VIA_GZ = \
  '  def get_model_state_via_gz(self, name):\n' \
  '    try:\n' \
  '      output = subprocess.check_output(\n' \
  "        ['gz', 'model', '-m', name, '-i'],\n" \
  '        stderr=subprocess.STDOUT,\n' \
  '        text=True,\n' \
  '        timeout=3.0,\n' \
  '      )\n' \
  '    except Exception:\n' \
  '      self.node.get_logger().warn(\n' \
  '        f"Unable to query model state for \'{name}\' via /gazebo/get_entity_state or gz model")\n' \
  '      return None'

NEW_VIA_GZ = \
  '  def get_model_state_via_gz(self, name):\n' \
  '    max_attempts = 30\n' \
  '    output = None\n' \
  '    for attempt in range(max_attempts):\n' \
  '      try:\n' \
  '        output = subprocess.check_output(\n' \
  "          ['gz', 'model', '-m', name, '-i'],\n" \
  '          stderr=subprocess.STDOUT,\n' \
  '          text=True,\n' \
  '          timeout=5.0,\n' \
  '        )\n' \
  '        break\n' \
  '      except Exception:\n' \
  '        if attempt < max_attempts - 1:\n' \
  '          self.node.get_logger().warn(\n' \
  '            f"gz model query for \'{name}\' attempt {attempt+1}/{max_attempts} failed, retrying in 1s...")\n' \
  '          time.sleep(1.0)\n' \
  '        else:\n' \
  '          self.node.get_logger().warn(\n' \
  '            f"Unable to query model state for \'{name}\' via /gazebo/get_entity_state or gz model")\n' \
  '          return None\n' \
  '    if output is None:\n' \
  '      return None'

if OLD_VIA_GZ in src:
    src = src.replace(OLD_VIA_GZ, NEW_VIA_GZ, 1)
    print(f"[OK]   Patched get_model_state_via_gz in {path}")
elif NEW_VIA_GZ in src:
    print(f"[SKIP] get_model_state_via_gz already patched in {path}")
else:
    print(f"[WARN] Could not find get_model_state_via_gz pattern in {path} — skipping.")
    print(f"       You may need to patch this method manually (see README.md).")

with open(path, 'w') as fh:
    fh.write(src)

print(f"[DONE] {path}")
print()
PYEOF
}

echo "=== Patching installed copy ==="
patch_file "$INSTALL_FILE"

echo "=== Patching source copy ==="
patch_file "$SOURCE_FILE"

echo "=== All done ==="
echo ""
echo "No rebuild required for cw2_world_spawner."
echo "If you renamed the package to cw2_team_20, rebuild it with:"
echo "  cd ${ROS2_WS} && colcon build --packages-select cw2_team_20"
