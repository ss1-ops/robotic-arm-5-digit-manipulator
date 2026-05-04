#!/bin/bash

# Finder double-click launcher for the Moveo Simple Controller.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

bash "$SCRIPT_DIR/launch_simple_controller.sh"
STATUS=$?

echo ""
if [ "$STATUS" -ne 0 ]; then
  echo "Launcher exited with status $STATUS"
fi
read -r -p "Press Enter to close this window..." _

exit "$STATUS"
