#!/bin/bash
# 一键编译脚本 - 只编译, 不烧录
# 用法: ./build.sh
# 依赖:
#   - cmake / ninja / arm-none-eabi-gcc (STM32CubeCLT, 已在 PATH)
#   - cube-cmake (STM32CubeIDE build-cmake 扩展提供, 缓存里 CMAKE_COMMAND 指向它,
#                  ninja 再生成 CMakeLists 时需要能在 PATH 里找到)
set -e
cd "$(dirname "$0")"

# STM32CubeCLT 工具链路径（若存在则加入 PATH，用于编译）
BUNDLE="$HOME/.local/share/stm32cube/bundles"
GCCBIN=$(find "$BUNDLE/gnu-tools-for-stm32" -maxdepth 2 -name bin -type d 2>/dev/null | head -1)
NINJABIN=$(find "$BUNDLE/ninja" -maxdepth 2 -name bin -type d 2>/dev/null | head -1)
[ -n "$GCCBIN" ]   && export PATH="$GCCBIN:$PATH"
[ -n "$NINJABIN" ] && export PATH="$NINJABIN:$PATH"

# cube-cmake (STM32CubeIDE build-cmake 扩展)
CUBE_CMAKE=$(ls -d "$HOME/.vscode/extensions"/stmicroelectronics.stm32cube-ide-build-cmake-*/resources/cube-cmake/win32/x86_64 2>/dev/null | head -1)
[ -n "$CUBE_CMAKE" ] && export PATH="$CUBE_CMAKE:$PATH"

echo "==> 编译..."
cmake --build build/Debug -j
echo "==> 编译完成!"
