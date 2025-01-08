#!/bin/sh
make clean
make BOOT=1 -j32
make -j32