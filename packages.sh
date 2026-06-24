#! /bin/bash

if [ -f "/etc/arch-release" ]; then
    echo "Assuming Arch Linux based on /etc/arch-release file."
    pacman -S --needed --noconfirm base-devel cmake gcc pkgconf hidapi libsystemd tomlplusplus portaudio fftw sdl2
elif [ -f "/etc/debian_version" ]; then
    echo "Assuming Debian based on /etc/debian_version file."
    apt-get update
    apt-get install -y cmake build-essential gcc pkg-config libhidapi-dev libsystemd-dev libtomlplusplus-dev portaudio19-dev libfftw3-dev libsdl2-dev
else
    echo "Unknown distro"
    exit 1
fi
exit 0