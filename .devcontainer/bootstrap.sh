#!/usr/bin/env bash
# Запускается один раз после создания контейнера (postCreateCommand).
set -euo pipefail

cd "$(dirname "$0")/.."

echo "==> installing conan profile"
mkdir -p ~/.conan2/profiles
cp .conan/default ~/.conan2/profiles/default

echo "==> conan install: debug"
conan install . --build=missing -s build_type=Debug

echo "==> conan install: release"
conan install . --build=missing -s build_type=RelWithDebInfo

echo "==> done. Use 'cmake --preset debug' or open in VSCode."