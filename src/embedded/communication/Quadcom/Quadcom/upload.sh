#!/usr/bin/env bash
set -euo pipefail

# Upload helper for the Quadcom ESP32 PlatformIO project
# This script activates the pyenv environment (if available) and then
# builds and uploads the `esp32dev` environment using PlatformIO.

# Pfad relativ zum Projekt
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# PlatformIO environment name (matches platformio.ini)
ENV_NAME="esp32dev"

# Activate pyenv environment that contains PlatformIO if available
if command -v pyenv >/dev/null 2>&1; then
  PYENV_ENV="3.13.1/envs/FOC"
  echo "Activating pyenv environment: $PYENV_ENV"
  pyenv shell "$PYENV_ENV" || true
fi

echo "Changing to project dir: $PROJECT_DIR"
cd "$PROJECT_DIR"

echo "Building project (env: $ENV_NAME)"
platformio run -e "$ENV_NAME"

echo "Uploading firmware to device (env: $ENV_NAME)"
platformio run -e "$ENV_NAME" -t upload -v

echo "Upload finished."
