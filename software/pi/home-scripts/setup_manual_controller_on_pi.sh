#!/bin/bash
# Run this on armpi: bash ~/setup_manual_controller_on_pi.sh

set -e

echo "╔════════════════════════════════════════════════╗"
echo "║  Moveo Manual Joint Controller Setup           ║"
echo "║  Running on armpi - Follow prompts             ║"
echo "╚════════════════════════════════════════════════╝"
echo ""

# Check if helper node exists
if [ ! -f ~/ros_nodes/manual_joint_controller_node.py ]; then
    echo "❌ ERROR: ~/ros_nodes/manual_joint_controller_node.py not found"
    echo "Copy the file first: scp manual_joint_controller_node.py armpi@armpi.local:~/ros_nodes/"
    exit 1
fi

echo "✓ Helper node found at ~/ros_nodes/manual_joint_controller_node.py"
echo ""

# Create systemd service
echo "Creating systemd service..."
sudo tee /etc/systemd/system/manual-joint-controller.service > /dev/null <<'SERVICEEOF'
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
SERVICEEOF

echo "✓ Service file created"
echo ""

echo "Enabling and starting service..."
sudo systemctl daemon-reload
sudo systemctl enable manual-joint-controller.service
sudo systemctl start manual-joint-controller.service

echo "✓ Service enabled and started"
echo ""

echo "Verifying setup..."
sleep 2

echo ""
echo "=== Service Status ==="
sudo systemctl status manual-joint-controller.service --no-pager | head -10

echo ""
echo "=== ROS Nodes ==="
source /opt/ros/jazzy/setup.bash
source ~/moveo_ws/install/local_setup.bash
ros2 node list 2>/dev/null | grep -i manual || echo "⏳ Waiting for node to initialize..."

echo ""
echo "=== Topics ==="
ros2 topic list 2>/dev/null | grep -i manual || echo "⏳ Topics not visible yet..."

echo ""
echo "✓ Setup complete!"
echo ""
echo "Next steps:"
echo "1. On your Mac, run: python3 ~/Desktop/moveo_joint_controller.py"
echo "2. Click 'Connect SSH'"
echo "3. Move sliders to control joints"
echo ""
