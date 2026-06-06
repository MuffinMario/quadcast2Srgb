# Build stage - produces the .deb inside the container.
# The docker-build.sh script copies the result out to ./packages/deb/.
FROM debian:bookworm-slim AS builder

# Install build dependencies (mirrors debian/control Build-Depends)
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        debhelper \
        cmake \
        gcc \
        g++ \
        pkg-config \
        libhidapi-dev \
        libsystemd-dev \
        libtomlplusplus-dev \
        libegl1-mesa-dev \
        libgles2-mesa-dev \
        portaudio19-dev \
        libfftw3-dev \
        rsync \
        dpkg-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src

# Copy the entire repo source into the image
COPY . .

# Place the debian/ packaging directory at the repo root (where dpkg-buildpackage expects it)
RUN cp -r resources/deb/debian debian

# Build the package - -b: binary only, -us/-uc: no signing required
RUN dpkg-buildpackage -b -us -uc

# Collect the .deb from the parent directory dpkg-buildpackage writes to
RUN mkdir -p /output && find /tmp /src .. -maxdepth 1 -name "quadcast2srgb_*.deb" -exec cp {} /output/ \; 2>/dev/null; \
    ls /output/*.deb 2>/dev/null || (find / -maxdepth 3 -name "quadcast2srgb_*.deb" -exec cp {} /output/ \; 2>/dev/null)

# ---- minimal final stage so the caller can docker cp from a clean image ----
FROM scratch AS output
COPY --from=builder /output /
