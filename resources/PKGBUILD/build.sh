#!/usr/bin/env bash
# Build the Arch source tarball and produce a ready-to-use PKGBUILD.
#
# Modes:
#   ./build.sh [--reinstall]              Local build: creates tarball next to this
#                                         script, patches PKGBUILD in-place with a
#                                         local source reference, and runs makepkg.
#
#   ./build.sh --package <outdir> <url>   CI/release mode: writes the tarball and a
#                                         finalised PKGBUILD into <outdir> using <url>
#                                         as the download URL. Does NOT touch the repo's
#                                         PKGBUILD and does NOT run makepkg.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PKGBUILD_FILE="$SCRIPT_DIR/PKGBUILD"

# ── Argument parsing ──────────────────────────────────────────────────────────
MODE="local"
REINSTALL=0
OUT_DIR=""
SOURCE_URL=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --reinstall) REINSTALL=1; shift ;;
        --package)
            MODE="package"
            OUT_DIR="$2"
            SOURCE_URL="$3"
            shift 3
            ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

# ── Version from CMakeLists.txt ───────────────────────────────────────────────
VERSION="$(awk '/project\(qc2srgb/{p=1} p && /VERSION/{match($0,/[0-9]+\.[0-9]+\.[0-9]+/); print substr($0,RSTART,RLENGTH); exit}' "$PROJECT_ROOT/CMakeLists.txt")"
PKGNAME="quadcast2srgb"
INNER_DIR="quadcast2Srgb-${VERSION}"   # must match cmake -S "quadcast2Srgb-$pkgver" in PKGBUILD
TARNAME="${PKGNAME}-${VERSION}.tar.gz"

echo "==> Packaging ${PKGNAME} v${VERSION}"

# ── Stage required source files ───────────────────────────────────────────────
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

STAGE="$TMP/$INNER_DIR"
mkdir -p "$STAGE/resources"

cp    "$PROJECT_ROOT/CMakeLists.txt"          "$STAGE/"
cp    "$PROJECT_ROOT/LICENSE"                 "$STAGE/"
cp -r "$PROJECT_ROOT/src"                     "$STAGE/src"
cp -r "$PROJECT_ROOT/cmake"                   "$STAGE/cmake"
cp -r "$PROJECT_ROOT/resources/config"        "$STAGE/resources/config"
cp -r "$PROJECT_ROOT/resources/systemd"       "$STAGE/resources/systemd"
cp -r "$PROJECT_ROOT/resources/udev"          "$STAGE/resources/udev"
[[ -d "$PROJECT_ROOT/resources/shader" ]] && cp -r "$PROJECT_ROOT/resources/shader" "$STAGE/resources/shader"

# ── CI / release mode ─────────────────────────────────────────────────────────
if [[ "$MODE" == "package" ]]; then
    mkdir -p "$OUT_DIR"
    TAROUT="$OUT_DIR/$TARNAME"

    tar -czf "$TAROUT" -C "$TMP" "$INNER_DIR"
    echo "==> Created $TAROUT"

    SHA256="$(sha256sum "$TAROUT" | awk '{print $1}')"
    echo "==> sha256: $SHA256"

    # Write a finalised PKGBUILD into <outdir>; the repo template is not touched.
    sed \
        -e "s/^pkgver=.*/pkgver=${VERSION}/" \
        -e "s/^pkgrel=.*/pkgrel=1/" \
        -e "s|^source=.*|source=(\"${TARNAME}::${SOURCE_URL}\")|" \
        -e "s/^sha256sums=.*/sha256sums=('${SHA256}')/" \
        "$PKGBUILD_FILE" > "$OUT_DIR/PKGBUILD"
    echo "==> $OUT_DIR/PKGBUILD written"

# ── Local build mode ──────────────────────────────────────────────────────────
else
    tar -czf "$SCRIPT_DIR/$TARNAME" -C "$TMP" "$INNER_DIR"
    echo "==> Created $SCRIPT_DIR/$TARNAME"

    SHA256="$(sha256sum "$SCRIPT_DIR/$TARNAME" | awk '{print $1}')"
    echo "==> sha256: $SHA256"

    # Patch the PKGBUILD in-place with a local source reference and real hash.
    sed -i \
        -e "s/^pkgver=.*/pkgver=${VERSION}/" \
        -e "s/^pkgrel=.*/pkgrel=1/" \
        -e "s|^source=.*|source=(\"${TARNAME}\")|" \
        -e "s/^sha256sums=.*/sha256sums=('${SHA256}')/" \
        "$PKGBUILD_FILE"
    echo "==> PKGBUILD patched for local build"

    cd "$SCRIPT_DIR"
    rm -rf src pkg

    if [[ "$REINSTALL" -eq 1 ]]; then
        sudo pacman -R "$PKGNAME" --noconfirm 2>/dev/null || true
        makepkg -fsi --noconfirm
    else
        makepkg -f
    fi
fi

