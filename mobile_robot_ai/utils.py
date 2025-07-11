from ament_index_python.packages import get_package_share_directory
import os
import json
import sys

try:
    # Gets the path to your package's share directory (installed via CMakeLists.txt)
    package_share_dir = get_package_share_directory("mobile_robot_ai")
    config_path = os.path.join(package_share_dir, "config.json")

    with open(config_path, "r") as f:
        config = json.load(f)

except FileNotFoundError:
    print(f"[ERROR] config.json not found at path: {config_path}", file=sys.stderr)
    sys.exit(1)

except json.JSONDecodeError as e:
    print(f"[ERROR] config.json contains invalid JSON: {e}", file=sys.stderr)
    sys.exit(1)

# Retrieve a configuration parameter
def get_config_param(parent, child):
    config_parent = config.get(parent)
    if config_parent is None:
        print(f"[WARNING] Parent key '{parent}' not found in config.")
        return None
    return config_parent.get(child)
