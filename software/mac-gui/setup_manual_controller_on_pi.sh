#!/bin/bash
# Setup script for Moveo Joint Controller testing environment
# Run this on armpi to set up the helper node

echo "=== Setting up Moveo Manual Joint Controller ==="
echo ""

# Copy helper script to Pi
HELPER_SCRIPT="$HOME/ros_nodes/manual_joint_controller_node.py"
echo "📦 Setting up helper ROS node..."

# Create ros_nodes directory if it doesn't exist
mkdir -p "$HOME/ros_nodes"

# Create systemd service for the helper node
echo "🔧 Creating systemd service..."

sudo tee /etc/systemd/system/manual-joint-controller.service > /dev/null <<EOF
[Unit]
Description=Moveo Manual Joint Controller Node
After=network.target
Requires=moveo_stack.service

[Service]
Type=simple
User=armpi
WorkingDirectory=/home/armpi
Environment="PATH=/home/armpi/.local/bin:/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
Environment="PYTHONUNBUFFERED=1"
ExecStart=/opt/ros/jazzy/bin/python3 /home/armpi/ros_nodes/manual_joint_controller_node.py
Restart=on-failure
RestartSec=5s
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

echo "✓ Systemd service created"
echo ""

echo "📋 Next steps:"
echo "1. Copy manual_joint_controller_node.py to armpi:/home/armpi/ros_nodes/"
echo "2. Run: scp manual_joint_controller_node.py armpi@armpi.local:~/ros_nodes/"
echo ""
echo "3. On armpi, enable the service:"
echo "   ssh armpi@armpi.local 'sudo systemctl daemon-reload && sudo systemctl enable manual-joint-controller.service && sudo systemctl start manual-joint-controller.service'"
echo ""
echo "4. Verify it's running:"
echo "   ssh armpi@armpi.local 'sudo systemctl status manual-joint-controller.service'"
echo ""
echo "5. Run the GUI app on your Mac:"
echo "   python3 moveo_joint_controller.py"
echo ""
