#!/usr/bin/env bash
set -eu
app_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
cd -- "$app_dir"
exec python3 -m im6014_bench.dashboard "$@"
