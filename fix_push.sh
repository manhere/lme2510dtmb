#!/bin/bash
set -e
cd "$(dirname "$0")"
rm -f push_repo.sh
git config user.email "manhere@users.noreply.github.com"
git add -A
git commit --amend --reset-author --no-edit
git -c http.proxy=http://127.0.0.1:7890 push -u origin main 2>&1 | tail -4
echo ---
git log --format='%h %an <%ae> %s' -1
git ls-files | grep -c . 
