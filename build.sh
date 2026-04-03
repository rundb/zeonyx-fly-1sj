#!/usr/bin/env bash
# Build helper for 1sj_shell Zephyr project
# Target: STM32L072CZ, UART shell on PA2 (TX) / PA3 (RX) at 115200 baud
#
# Usage:
#   ./build.sh           — configure + build
#   ./build.sh pristine  — clean + configure + build
#   ./build.sh flash     — build + flash via J-Link
#   ./build.sh menuconfig — open interactive Kconfig menu

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NCS_ROOT=/home/roman/ncs_v3.1.1

export ZEPHYR_BASE="${NCS_ROOT}/zephyr"
export ZEPHYR_SDK_INSTALL_DIR=/home/roman/programming/zephyr-sdk-0.16.5

EXTRA_CMAKE=(
    -DBOARD_ROOT="${SCRIPT_DIR}"
    -DZEPHYR_EXTRA_MODULES="${NCS_ROOT}/modules/hal/stm32"
)

case "${1:-build}" in
    pristine)
        west build --pristine -b 1sj_module "${SCRIPT_DIR}" -- "${EXTRA_CMAKE[@]}"
        ;;
    flash)
        west build -b 1sj_module "${SCRIPT_DIR}" -- "${EXTRA_CMAKE[@]}"
        west flash
        ;;
    menuconfig)
        west build -b 1sj_module "${SCRIPT_DIR}" -- "${EXTRA_CMAKE[@]}"
        west build -t menuconfig
        ;;
    build|*)
        west build -b 1sj_module "${SCRIPT_DIR}" -- "${EXTRA_CMAKE[@]}"
        ;;
esac
