#!/bin/bash
# 一键烧录脚本 - 编译并烧录到 STM32F407
# 用法: ./flash.sh
# 依赖: ~/.local/openocd/bin/openocd (OpenOCD 0.12, 已针对 CMSIS-DAP 配置)
set -e
cd "$(dirname "$0")"

# 1. 编译
./build.sh

# 2. 烧录 (烧完自动校验并复位运行)
echo "==> 烧录..."
~/.local/openocd/bin/openocd -f flash.cfg -c "program build/Debug/NatonalCarSet.elf verify reset exit"

echo "==> 烧录完成!"
