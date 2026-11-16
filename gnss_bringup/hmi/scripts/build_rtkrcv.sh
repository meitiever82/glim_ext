#!/usr/bin/env bash
# 把 RTKLIB-EX v2.5.1 的 rtkrcv 编到 tools/bin/rtkrcv(主机或 ARM64 目标机均可)。
# 只编 app/consapp/rtkrcv,不产生 librtklib.so,因此不需要 sudo / ldconfig;
# 产出的二进制可直接用 rtkrcv_node 的 binary 参数,或把 tools/bin 加进 PATH。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/third_party/RTKLIB"
# 钉 tag,不跟 demo5 分支头:分支会移动,而 rtkrcv_node 依赖 v2.5.1 的 -nc 选项
# (见 gnss_bringup README「安装 RTKLIB-EX 2.5.1」)。需要别的版本时用 RTKLIB_REF 覆盖。
RTKLIB_REF="${RTKLIB_REF:-v2.5.1}"

if [ ! -d "$SRC/.git" ]; then
    git clone --depth 1 --branch "$RTKLIB_REF" \
        https://github.com/rtklibexplorer/RTKLIB.git "$SRC"
fi

HEAD_SHA="$(git -C "$SRC" rev-parse HEAD)"
VER_RTKLIB="$(sed -n 's/^#define VER_RTKLIB *"\([^"]*\)".*/\1/p' "$SRC/src/rtklib.h")"
PATCH_LEVEL="$(sed -n 's/^#define PATCH_LEVEL *"\([^"]*\)".*/\1/p' "$SRC/src/rtklib.h")"

make -C "$SRC/app/consapp/rtkrcv/gcc" -j"$(nproc)"
mkdir -p "$ROOT/tools/bin"
cp "$SRC/app/consapp/rtkrcv/gcc/rtkrcv" "$ROOT/tools/bin/rtkrcv"

# 版本必须显式打印出来:源码里 PATCH_LEVEL 才是真正的版本号(VER_RTKLIB 只有 "EX"),
# 复用了旧 clone 时一眼能看出编的是哪个版本。
echo "built: $ROOT/tools/bin/rtkrcv (RTKLIB $VER_RTKLIB $PATCH_LEVEL, $RTKLIB_REF @ ${HEAD_SHA:0:12})"
echo "验证: $ROOT/tools/bin/rtkrcv -h"
