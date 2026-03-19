#! /bin/bash
# Copyright (c) 2026 Mario T (MuffinMario)
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT
set -e pipefail 

# install required packages (Windows users can use e.g. vcpkg to install hidapi)
./packages.sh


./uninstall.sh || true 

# cmake build the project
cmake -B build -S . \
-DCMAKE_BUILD_TYPE=Release \
-DCMAKE_INSTALL_PREFIX=/usr \
-DUSE_SYSTEMD=ON \
-DENABLE_CLANG_TIDY=OFF 

cmake --build build

sudo cmake --install build --prefix /usr