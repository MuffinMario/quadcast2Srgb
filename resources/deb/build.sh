#!/usr/bin/env bash
# Build the quadcast2srgb .deb package.
# Run this script from anywhere; it resolves paths relative to itself.
#
# Requirements: debhelper (>= 13), cmake, gcc, pkg-config,
#               libhidapi-dev, libsystemd-dev
#
# Usage (native, requires build deps installed on the host):
#   cd resources/deb && ./build.sh
#
# For a fully isolated build inside Docker (recommended for CI or cross-distro use),
# use the Docker wrapper instead — output goes to ./packages/deb/ at the repo root:
#   ./resources/deb/docker-build.sh
#
# The resulting .deb will appear in resources/deb/ (native) or packages/deb/ (docker).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Source root is two levels up from resources/deb/
SOURCE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "==> Source root: $SOURCE_ROOT"
echo "==> Build dir:   $SCRIPT_DIR"

# dpkg-buildpackage expects to be run from the source tree root that
# contains debian/ — so we stage a symlink/copy approach or use
# dpkg-buildpackage -b from the source root after pointing it at our debian/.
#
# Simplest portable approach: copy debian/ into a temp source tree and build.
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

echo "==> Staging source in $WORK_DIR ..."
rsync -a --exclude='resources/deb/debian/build' "$SOURCE_ROOT/" "$WORK_DIR/"
cp -r "$SCRIPT_DIR/debian" "$WORK_DIR/debian"

echo "==> Running dpkg-buildpackage ..."
cd "$WORK_DIR"
dpkg-buildpackage -b -us -uc

# Collect the produced .deb back to the deb folder
echo "==> Collecting output ..."
find "$(dirname "$WORK_DIR")" -maxdepth 1 -name "quadcast2srgb_*.deb" \
    -exec cp {} "$SCRIPT_DIR/" \;

echo ""
echo "==> Done. Package(s) written to: $SCRIPT_DIR/"
ls -lh "$SCRIPT_DIR/"*.deb 2>/dev/null || echo "(no .deb found — check dpkg-buildpackage output above)"
