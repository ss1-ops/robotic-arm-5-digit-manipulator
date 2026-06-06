#!/bin/bash
# Moveo Joint Controller - Quick Start Launcher
# macOS launcher script with automatic dependency check

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_NAME="Moveo Joint Controller"
PYTHON_MIN_VERSION="3.8"

echo "╔══════════════════════════════════════════╗"
echo "║  $APP_NAME            ║"
echo "║  Joint Testing Console                   ║"
echo "╚══════════════════════════════════════════╝"
echo ""

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "✓ Python version: $PYTHON_VERSION"

# Check if requirements are installed
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
    echo "Installing via pip..."
    python3 -m pip install --upgrade pip --break-system-packages > /dev/null
    python3 -m pip install -r "$SCRIPT_DIR/requirements.txt" --break-system-packages
    echo "✓ Dependencies installed"
fi

echo ""
echo "🚀 Starting $APP_NAME..."
echo ""
cd "$SCRIPT_DIR"
python3 moveo_joint_controller.py "$@"
