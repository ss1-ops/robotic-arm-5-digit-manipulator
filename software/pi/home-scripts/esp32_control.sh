#!/bin/bash
# Control ESP32-S3 reset and bootloader mode via Pi GPIO.
#
# WIRING (use 330Ω resistors in series):
#   Pi GPIO17 (pin 11) ---[330Ω]--- ESP32 EN (RST)
#   Pi GPIO27 (pin 13) ---[330Ω]--- ESP32 IO0 (BOOT)
#
# Usage:
#   ~/esp32_control.sh reset       — hard reset ESP32 (runs normally)
#   ~/esp32_control.sh bootloader  — reset into download mode (for flashing)

CHIP="gpiochip0"
EN_PIN=17    # BCM17 → ESP32 EN (active LOW = reset)
BOOT_PIN=27  # BCM27 → ESP32 IO0 (active LOW = bootloader mode)

reset_esp32() {
  echo "Resetting ESP32..."
  gpioset --mode=time --sec=0 --usec=200000 "$CHIP" ${EN_PIN}=0
  gpioset "$CHIP" ${EN_PIN}=1
  echo "Reset done."
}

enter_bootloader() {
  echo "Entering ESP32 bootloader mode..."
  # Hold IO0 low, pulse EN low, release IO0
  gpioset "$CHIP" ${BOOT_PIN}=0
  sleep 0.05
  gpioset --mode=time --sec=0 --usec=200000 "$CHIP" ${EN_PIN}=0
  gpioset "$CHIP" ${EN_PIN}=1
  sleep 0.05
  gpioset "$CHIP" ${BOOT_PIN}=1
  echo "ESP32 is now in bootloader mode."
}

case "${1:-}" in
  reset)      reset_esp32 ;;
  bootloader) enter_bootloader ;;
  *)
    echo "Usage: $0 reset | bootloader"
    exit 1
    ;;
esac
