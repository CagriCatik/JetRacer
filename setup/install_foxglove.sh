#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./common.sh
source "$SCRIPT_DIR/common.sh"

original_args=("$@")
package_path=''

while [[ $# -gt 0 ]]; do
  case "$1" in
    --package)
      [[ $# -ge 2 ]] || fail '--package requires a path.'
      package_path=$2
      shift 2
      ;;
    *)
      fail "Unknown argument: $1"
      ;;
  esac
done

require_root "${original_args[@]}"
detect_ubuntu

if [[ -z "$package_path" ]]; then
  package_path=$(
    find . -maxdepth 1 -type f -name 'foxglove-studio-*.deb' -printf '%T@ %p\n' \
      | sort -nr | head -n 1 | cut -d' ' -f2-
  )
fi

if [[ -z "$package_path" ]]; then
  fail "No package found matching 'foxglove-studio-*.deb' in the current directory. Download the .deb first or pass --package <path>."
fi

[[ -f "$package_path" ]] || fail "Package not found: $package_path"
[[ "$package_path" == *.deb ]] || fail "Expected a .deb package, got: $package_path"

log "Installing Foxglove Studio from: $package_path"
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y "$package_path"

log 'Foxglove Studio installation completed.'
log 'To install future updates, run: sudo apt update && sudo apt install foxglove-studio'
