#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# macOS Dependency Installer for Moveo Joint Controller

echo "╔═══════════════════════════════════════════════════╗"
echo "║  Installing Moveo Joint Controller Dependencies  ║"
echo "╚═══════════════════════════════════════════════════╝"
echo ""

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "Python version: $PYTHON_VERSION"

# Upgrade pip
echo ""
echo "Upgrading pip..."
python3 -m pip install --upgrade pip --break-system-packages -q

# Install requirements
echo "Installing dependencies..."
pip3 install -r requirements.txt --break-system-packages -q

# Verify installation
echo ""
echo "Verifying installation..."
python3 -c "import PyQt5; import paramiko; print('✓ PyQt5 installed'); print('✓ paramiko version:', paramiko.__version__)"

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Installation successful!"
    echo ""
    echo "You can now run:"
    echo "   python3 moveo_joint_controller.py"
    echo ""
    echo "Or use the launcher:"
    echo "   bash launch_joint_controller.sh"
else
    echo ""
    echo "❌ Installation failed. Please check the error above."
    exit 1
fi
