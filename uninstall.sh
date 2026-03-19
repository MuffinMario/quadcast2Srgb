#! /bin/bash
# Copyright (c) 2026 Mario T (MuffinMario)
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT
set -e pipefail 

# install required packages (Windows users can use e.g. vcpkg to install hidapi)
sudo cmake --build build --target uninstall