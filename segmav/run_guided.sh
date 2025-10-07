#!/usr/bin/env bash
cd "$(dirname "$0")"
source venv/bin/activate 2>/dev/null || true
python3 guided_wp_segnet_preview.py "$@"
