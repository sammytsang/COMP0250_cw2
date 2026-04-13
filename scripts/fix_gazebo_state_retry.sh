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

    # ------------------------------------------------------------------ #
    # 1. Ensure `import time` is present (needed for time.sleep calls)    #
    # ------------------------------------------------------------------ #
    if ! grep -q "^import time" "$TARGET"; then
        # Insert after the first `import` line
        sed -i '0,/^import /s//import time\nimport /' "$TARGET"
        echo "[OK]   Added 'import time' to $TARGET"
    else
        echo "[OK]   'import time' already present in $TARGET"
    fi

    # ------------------------------------------------------------------ #
    # 2. Patch get_model_state_by_name                                    #
    # ------------------------------------------------------------------ #
    # Detect whether the file has already been patched
    if grep -q "max_attempts = 10" "$TARGET"; then
        echo "[SKIP] get_model_state_by_name already patched in $TARGET"
    else
        python3 - "$TARGET" <<'PYEOF'
import sys, re

path = sys.argv[1]
with open(path, 'r') as fh:
    src = fh.read()

OLD = (
    r'def get_model_state_by_name\(self, name, relname="world"\):\n'
    r'        if self\.get_state_client\.wait_for_service\(timeout_sec=0\.2\):\n'
    r'            request = GetEntityState\.Request\(\)\n'
    r'            request\.name = name\n'
    r'            request\.reference_frame = relname\n'
    r'            return call_service_sync\(self\.client_node, self\.get_state_client, request, timeout_sec=5\.0\)\n'
    r'\n'
    r'        return self\.get_model_state_via_gz\(name\)'
)

NEW = (
    'def get_model_state_by_name(self, name, relname="world"):\n'
    '        # Wait longer for the service to be available\n'
    '        if not self.get_state_client.wait_for_service(timeout_sec=10.0):\n'
    '            return self.get_model_state_via_gz(name)\n'
    '\n'
    '        request = GetEntityState.Request()\n'
    '        request.name = name\n'
    '        request.reference_frame = relname\n'
    '\n'
    '        # Retry up to 10 times — entity may not be registered in Gazebo immediately after spawn\n'
    '        max_attempts = 10\n'
    '        for attempt in range(max_attempts):\n'
    '            resp = call_service_sync(self.client_node, self.get_state_client, request, timeout_sec=5.0)\n'
    '            if resp is not None and getattr(resp, \'success\', False):\n'
    '                return resp\n'
    '            time.sleep(1.0)\n'
    '\n'
    '        # Final fallback to gz model CLI\n'
    '        return self.get_model_state_via_gz(name)'
)

new_src, n = re.subn(OLD, NEW, src, flags=re.MULTILINE)
if n == 0:
    print(f"[WARN] Could not find get_model_state_by_name pattern in {path} — skipping.")
else:
    with open(path, 'w') as fh:
        fh.write(new_src)
    print(f"[OK]   Patched get_model_state_by_name in {path}")
PYEOF
    fi

    # ------------------------------------------------------------------ #
    # 3. Patch get_model_state_via_gz                                     #
    # ------------------------------------------------------------------ #
    if grep -q "max_attempts = 5" "$TARGET"; then
        echo "[SKIP] get_model_state_via_gz already patched in $TARGET"
    else
        python3 - "$TARGET" <<'PYEOF'
import sys, re

path = sys.argv[1]
with open(path, 'r') as fh:
    src = fh.read()

# Match the entire existing try/except block inside get_model_state_via_gz.
# The method currently does a single subprocess call followed by a regex parse.
# We wrap both in a retry loop.
OLD = (
    r'(    def get_model_state_via_gz\(self, name\):\n)'
    r'(        try:\n'
    r'            output = subprocess\.check_output\(\n'
    r"                \['gz', 'model', '-m', name, '-i'\],\n"
    r'                stderr=subprocess\.STDOUT,\n'
    r'                text=True,\n'
    r'                timeout=3\.0,\n'
    r'            \)\n'
    r'        except Exception:\n'
    r'            self\.node\.get_logger\(\)\.warn\(\n'
    r'                f"Unable to query model state for \'\{name\}\' via /gazebo/get_entity_state or gz model"\)\n'
    r'            return None\n'
    r'\n'
    r'        match = re\.search\((.*?)\)\n'
    r'\n'
    r'        if match is None:\n'
    r'            return None)'
)

def replacer(m):
    method_def = m.group(1)
    search_args = m.group(3)  # captured by (.*?) inside re\.search\((.*?)\)
    return (
        method_def +
        '        max_attempts = 5\n'
        '        match = None\n'
        '        for attempt in range(max_attempts):\n'
        '            try:\n'
        '                output = subprocess.check_output(\n'
        "                    ['gz', 'model', '-m', name, '-i'],\n"
        '                    stderr=subprocess.STDOUT,\n'
        '                    text=True,\n'
        '                    timeout=3.0,\n'
        '                )\n'
        '            except Exception:\n'
        '                if attempt < max_attempts - 1:\n'
        '                    time.sleep(1.0)\n'
        '                    continue\n'
        '                self.node.get_logger().warn(\n'
        '                    f"Unable to query model state for \'{name}\' via /gazebo/get_entity_state or gz model")\n'
        '                return None\n'
        '\n'
        f'            match = re.search({search_args})\n'
        '\n'
        '            if match is not None:\n'
        '                break\n'
        '            time.sleep(1.0)\n'
        '\n'
        '        if match is None:\n'
        '            return None'
    )

new_src, n = re.subn(OLD, replacer, src, flags=re.DOTALL)
if n == 0:
    print(f"[WARN] Could not find get_model_state_via_gz pattern in {path} — skipping.")
    print("       You may need to patch this method manually (see README.md).")
else:
    with open(path, 'w') as fh:
        fh.write(new_src)
    print(f"[OK]   Patched get_model_state_via_gz in {path}")
PYEOF
    fi

    echo "[DONE] $TARGET"
    echo ""
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
