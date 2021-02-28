#!/bin/bash
rm -rf "bin"

cd "/root/sdir/ctrl/bin"

cmake ..
make
#ping 127.0.0.1
./sdir_ctrl2020
