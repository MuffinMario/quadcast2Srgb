#!/usr/bin/env bash
# Build the quadcast2srgb .deb package inside a Debian Docker container
# and copy the result to ./packages/deb/ at the repository root.
#
# Requirements: docker (running and accessible by current user)
#
# Usage (from anywhere in the repo):
#   ./resources/deb/docker-build.sh
#
# The .deb will be written to <repo-root>/packages/deb/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUTPUT_DIR="$REPO_ROOT/packages/deb"
IMAGE_TAG="quadcast2srgb-deb-builder"
CONTAINER_NAME="quadcast2srgb-deb-build-$$"

echo "==> Repository root : $REPO_ROOT"
echo "==> Output directory: $OUTPUT_DIR"
echo ""

# Ensure the output directory exists
mkdir -p "$OUTPUT_DIR"

# Always remove the build container on exit, even on error
cleanup() {
    docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "==> Building Docker image ($IMAGE_TAG) ..."
docker build \
    --target builder \
    -t "$IMAGE_TAG" \
    -f "$SCRIPT_DIR/Dockerfile" \
    "$REPO_ROOT"

echo ""
echo "==> Creating container to extract .deb ..."
docker create --name "$CONTAINER_NAME" "$IMAGE_TAG" /bin/true

echo "==> Copying .deb to $OUTPUT_DIR ..."
# dpkg-buildpackage writes the .deb one level above the build source root (/src),
# which inside the container is /
docker cp "$CONTAINER_NAME:/output/." "$OUTPUT_DIR/"

echo ""
echo "==> Done. Package(s) in $OUTPUT_DIR/:"
ls -lh "$OUTPUT_DIR/"*.deb 2>/dev/null \
    || echo "  (no .deb found - check docker build output above)"
