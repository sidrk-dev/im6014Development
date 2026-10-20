#!/usr/bin/env bash
set -eu
repo_dir="$(cd -- "$(dirname -- "$0")/.." && pwd)"
cd -- "$repo_dir"
python -m tests.bench.test_motor
QT_QPA_PLATFORM=offscreen python -m tests.bench.test_dashboard
python -m im6014_bench.motor --help >/dev/null
python scripts/check_repository.py
