#!/usr/bin/env bash
set -euo pipefail

# Pfad relativ zum MCU-Projekt
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ELF und BIN Pfade aus PlatformIO build
ELF=".pio/build/genericSTM32F411RE/firmware.elf"
BIN=".pio/build/genericSTM32F411RE/firmware.bin"

# Activate pyenv environment that contains PlatformIO / toolchain
# This ensures the correct Python and tools (platformio, arm-none-eabi-objcopy, st-flash)
# are on PATH when the script runs. If pyenv is not available, this will silently continue.
if command -v pyenv >/dev/null 2>&1; then
	# Use the exact environment name used interactively
	PYENV_ENV="3.13.1/envs/FOC"
	echo "Activating pyenv environment: $PYENV_ENV"
	# `pyenv shell` sets the shell-local pyenv version
	pyenv shell "$PYENV_ENV" || true
fi

# Erzeuge BIN aus ELF
echo "Running PlatformIO build for genericSTM32F411RE"
platformio run -e genericSTM32F411RE

# Erzeuge BIN aus ELF (produced by PlatformIO build)
arm-none-eabi-objcopy -O binary "$ELF" "$BIN"

# Schreibe per st-flash (connect under reset)
st-flash --connect-under-reset write "$BIN" 0x8000000