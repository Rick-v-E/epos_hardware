#!/usr/bin/env bash
#
# Installs the PROPRIETARY maxon EPOS Command Library (libEposCmd) and the
# FTDI D2XX library (libftd2xx) system-wide, using maxon's own installer.
#
# These libraries are NOT bundled in this repository for license reasons.
# Obtain the "EPOS Command Library" for Linux from maxon:
#   https://www.maxongroup.com  (Downloads -> Software/Firmware -> EPOS Command Library)
#
# Usage:
#   scripts/install_maxon_epos_library.sh /path/to/EPOS-Linux-Library-En.zip
#   scripts/install_maxon_epos_library.sh /path/to/EPOS_Linux_Library   (extracted dir)
#
set -euo pipefail

SRC="${1:-}"
if [[ -z "$SRC" ]]; then
  echo "Usage: $0 <EPOS-Linux-Library ZIP | extracted directory>" >&2
  exit 1
fi

WORK="$(mktemp -d)"
cleanup() { rm -rf "$WORK"; }
trap cleanup EXIT

if [[ "$SRC" == *.zip ]]; then
  echo "Extracting $SRC ..."
  unzip -q "$SRC" -d "$WORK"
  DIR="$(find "$WORK" -maxdepth 3 -name install.sh -printf '%h\n' | head -1)"
else
  DIR="$SRC"
fi

if [[ -z "${DIR:-}" || ! -f "$DIR/install.sh" ]]; then
  echo "ERROR: could not find install.sh under '$SRC'." >&2
  exit 1
fi

echo "Running the official maxon installer from: $DIR"
# Two things matter in this one line.
#
# The cd: maxon's install.sh copies with relative paths - ./include,
# ./lib/arm/v8, ./misc/99-ftdi.rules - so it must run with its own directory as
# the working directory. Without that every cp fails, but `ln -sf` still
# succeeds, because it happily points a symlink at something that does not
# exist. The installer then prints "EPOS Command Library installed" and leaves
# /usr/lib/libEposCmd.so dangling. Seen on a Raspberry Pi 4 on 2026-11-24: an
# installation that looks complete and is not.
#
# The subshell keeps the cd out of the caller's shell, and bash <file> is used
# because /tmp is often mounted noexec.
( cd "$DIR" && sudo bash ./install.sh )

echo
echo "Verifying system integration:"
# -e follows the symlink, so a dangling one fails here instead of passing as a
# tidy-looking ls line. This is the check that would have caught the above.
broken=0
for lib in /usr/lib/libEposCmd.so /usr/lib/libftd2xx.so; do
  if [[ -e "$lib" ]]; then
    printf '  %-24s -> %s
' "$lib" "$(readlink -f "$lib")"
  else
    printf '  %-24s BROKEN: target missing (%s)
'       "$lib" "$(readlink "$lib" 2>/dev/null || echo 'not a symlink')" >&2
    broken=1
  fi
done
if [[ "$broken" -ne 0 ]]; then
  echo "ERROR: the library is not usable - see above." >&2
  exit 1
fi
echo "Done. You may need to re-plug the USB device (udev) and re-login (group access)."
