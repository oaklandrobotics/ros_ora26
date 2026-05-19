#!/usr/bin/env bash
set -u

REPO="/home/ora/ros_ora26"

cd "$REPO" || exit 0

timeout 20 git fetch origin main || exit 0
git merge --ff-only "origin/main" || exit 0
