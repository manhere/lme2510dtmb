#!/bin/bash
set -e
cd "$(dirname "$0")"
git add -A
git commit -m "ci: package .deb (module+firmware+blacklist) and publish to GitHub Release (rolling 'latest' on main push, versioned on v* tags)" 2>&1 | tail -1
git -c http.proxy=http://127.0.0.1:7890 push 2>&1 | tail -2
