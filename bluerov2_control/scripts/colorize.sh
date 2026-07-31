#!/usr/bin/env bash
# Usage: colorize.sh <ansi_color_code> <tag> -- <command...>
COLOR="$1"
TAG="$2"
shift 2
if [ "$1" = "--" ]; then shift; fi

ESC=$'\033'
PREFIX="${ESC}[${COLOR}m[${TAG}]${ESC}[0m"

stdbuf -oL -eL "$@" 2>&1 | while IFS= read -r line; do
  printf '%s %s\n' "$PREFIX" "$line"
done
