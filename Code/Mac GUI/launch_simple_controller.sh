#!/bin/bash
# Moveo Simple Controller - Quick Start Launcher

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_NAME="Moveo Simple Controller"

echo "╔══════════════════════════════════════════╗"
echo "║  Moveo Simple Controller                 ║"
echo "║  Direct ROS2 Joint Control               ║"
echo "╚══════════════════════════════════════════╝"
echo ""

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "✓ Python version: $PYTHON_VERSION"

# Check dependencies
echo ""
echo "📦 Checking dependencies..."

MISSING_PACKAGES=()
for package in PyQt5 paramiko; do
    if ! python3 -c "import ${package}" 2>/dev/null; then
        MISSING_PACKAGES+=("$package")
    else
        echo "  ✓ $package installed"
    fi
done

if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    echo ""
    echo "⚠️  Missing packages: ${MISSING_PACKAGES[*]}"
    echo "Installing..."
    python3 -m pip install --upgrade pip --break-system-packages > /dev/null
    python3 -m pip install PyQt5 paramiko --break-system-packages
    echo "✓ Dependencies installed"
fi

echo ""
echo "🚀 Starting $APP_NAME..."
echo ""
cd "$SCRIPT_DIR"
python3 moveo_simple_controller.py "$@"
