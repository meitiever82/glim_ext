#!/usr/bin/env bash
# Build rtklibexplorer demo5 rtkrcv into tools/bin/rtkrcv (host or ARM64).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/third_party/RTKLIB"
if [ ! -d "$SRC" ]; then
    git clone --depth 1 --branch demo5 https://github.com/rtklibexplorer/RTKLIB.git "$SRC"
fi
make -C "$SRC/app/consapp/rtkrcv/gcc" -j"$(nproc)"
mkdir -p "$ROOT/tools/bin"
cp "$SRC/app/consapp/rtkrcv/gcc/rtkrcv" "$ROOT/tools/bin/rtkrcv"
echo "built: $ROOT/tools/bin/rtkrcv"
