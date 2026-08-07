#!/bin/bash
# 一键烧录脚本 - 编译并烧录到 STM32F407
# 用法: ./flash.sh
# 依赖: ~/.local/openocd/bin/openocd (OpenOCD 0.12, 已针对 CMSIS-DAP 配置)
#        STM32CubeCLI bundle 的 ARM 工具链 (~/.local/share/stm32cube/bundles)
set -e
cd "$(dirname "$0")"

# STM32CubeCLI bundle 工具链路径（若存在则加入 PATH，用于编译）
BUNDLE="$HOME/.local/share/stm32cube/bundles"
GCCBIN=$(find "$BUNDLE/gnu-tools-for-stm32" -maxdepth 2 -name bin -type d 2>/dev/null | head -1)
NINJABIN=$(find "$BUNDLE/ninja" -maxdepth 2 -name bin -type d 2>/dev/null | head -1)
[ -n "$GCCBIN" ]   && export PATH="$GCCBIN:$PATH"
[ -n "$NINJABIN" ] && export PATH="$NINJABIN:$PATH"

# 1. 编译（工具链可用 且 构建缓存不是 Windows 生成时才重新编译）
CAN_BUILD=no
if command -v arm-none-eabi-gcc >/dev/null 2>&1 && [ -f build/Debug/build.ninja ] \
   && ! grep -q 'd:/' build/Debug/CMakeCache.txt 2>/dev/null; then
    CAN_BUILD=yes
fi

if [ "$CAN_BUILD" = "yes" ]; then
    echo "==> 编译..."
    cmake --build build/Debug -j
else
    echo "==> 跳过编译（无 ARM 工具链或构建缓存来自 Windows），直接烧录现有 elf"
fi

# 2. 烧录 (烧完自动校验并复位运行)
echo "==> 烧录..."
~/.local/openocd/bin/openocd -f flash.cfg -c "program build/Debug/NatonalCarSet.elf verify reset exit"

echo "==> 烧录完成!"
