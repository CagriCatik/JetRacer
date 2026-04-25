#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./common.sh
source "$SCRIPT_DIR/common.sh"

original_args=("$@")
version='latest'

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      [[ $# -ge 2 ]] || fail '--version requires a value.'
      version=$2
      shift 2
      ;;
    *)
      fail "Unknown argument: $1"
      ;;
  esac
done

require_root "${original_args[@]}"
detect_ubuntu

case "$(dpkg --print-architecture)" in
  amd64)
    code_arch='x64'
    ;;
  arm64)
    code_arch='arm64'
    ;;
  *)
    fail 'Visual Studio Code is only handled here for amd64 and arm64.'
    ;;
esac

tmp_deb=$(mktemp /tmp/vscode.XXXXXX.deb)
trap 'rm -f "$tmp_deb"' EXIT

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y ca-certificates curl

log "Downloading Visual Studio Code ($code_arch, version: $version)."
curl -fsSL -o "$tmp_deb" "https://update.code.visualstudio.com/$version/linux-deb-$code_arch/stable"

apt-get install -y "$tmp_deb"

log 'Visual Studio Code installation completed.'
