#! /bin/bash

if [ -f "/etc/arch-release" ]; then
    echo "Assuming Arch Linux based on /etc/arch-release file."
    pacman -S --needed --noconfirm base-devel cmake gcc pkgconf hidapi libsystemd tomlplusplus portaudio fftw
elif [ -f "/etc/debian_version" ]; then
    echo "Assuming Debian based on /etc/debian_version file."
    apt-get update
    apt-get install -y cmake build-essential gcc pkg-config libhidapi-dev libsystemd-dev libtomlplusplus-dev libportaudio-dev libfftw3-dev
else
    echo "Unknown distro"
    exit 1
fi
exit 0