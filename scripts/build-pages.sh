#!/usr/bin/env bash
set -euo pipefail

rm -rf pages-dist
mkdir -p pages-dist
cp -R site/. pages-dist/
test -s pages-dist/index.html
