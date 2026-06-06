#!/bin/bash
# Run once on the Pi to install Arduino CLI and the ESP32S3 core.
# Usage: bash setup_arduino_cli_on_pi.sh
set -e

echo "=== Installing Arduino CLI ==="
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
export PATH="$HOME/.local/bin:$PATH"

echo "=== Initialising config ==="
arduino-cli config init --overwrite
arduino-cli config add board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

echo "=== Updating index ==="
arduino-cli core update-index

echo "=== Installing ESP32 core (this takes a few minutes) ==="
arduino-cli core install esp32:esp32

echo "=== Installing standard libraries used by the sketch ==="
# Note: WebServer and ESPmDNS are bundled inside the ESP32 core — do NOT install separately.
arduino-cli lib install "AsyncTCP"

echo ""
echo "Done. Add ~/.local/bin to PATH permanently:"
echo "  echo 'export PATH=\"\$HOME/.local/bin:\$PATH\"' >> ~/.bashrc && source ~/.bashrc"
echo ""
echo "Then flash the sketch with:"
echo "  bash ~/flash_esp32.sh"
