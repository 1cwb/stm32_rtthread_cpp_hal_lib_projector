#!/bin/sh
make clean

# 将所有脚本参数 ($@) 正确地传递给 make 命令
make BOOT=1 -j32 "$@" && make -j32 "$@" 2>&1 | tee output/build.log