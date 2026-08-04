#!/usr/bin/env bash
# Build Doxygen API docs for multiple versions of Flexiv TDK.
#
# Output layout (under docs/api/doxygen/, which is git-ignored):
#   docs/api/doxygen/           latest release (backward-compatible entry point)
#   docs/api/doxygen/<tag>/     each release tag that ships docs/doxygen/Doxyfile.in
#
# Usage: bash docs/doxygen/build_api_docs.sh
# Requires: git, doxygen, graphviz. All release tags must be fetched (git fetch --tags).

set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
OUT_BASE="${REPO_ROOT}/docs/api/doxygen"
WORKTREE_BASE="$(mktemp -d /tmp/tdk_api_docs.XXXXXX)"

cleanup() {
  git -C "$REPO_ROOT" worktree list --porcelain | awk '/^worktree /{print $2}' | \
    while read -r wt; do
      case "$wt" in
        "${WORKTREE_BASE}"/*) git -C "$REPO_ROOT" worktree remove --force "$wt" ;;
      esac
    done
  rm -rf "$WORKTREE_BASE"
}
trap cleanup EXIT

# Release tags that ship a Doxyfile, oldest to newest
TAGS="$(git -C "$REPO_ROOT" tag | while read -r t; do
  git -C "$REPO_ROOT" cat-file -e "${t}:docs/doxygen/Doxyfile.in" 2>/dev/null && echo "$t"
done | sort -V)"

if [ -z "$TAGS" ]; then
  echo "No release tag with docs/doxygen/Doxyfile.in found; run 'git fetch --tags' first" >&2
  exit 1
fi

LATEST="$(echo "$TAGS" | tail -1)"
echo "Release tags with Doxyfile: $(echo $TAGS | tr '\n' ' ') (latest: ${LATEST})"

build_doxygen() {  # <src_dir> <out_dir>
  local src_dir="$1" out_dir="$2"
  mkdir -p "$out_dir"
  (cd "$src_dir" && sed "s|^OUTPUT_DIRECTORY\s*=.*|OUTPUT_DIRECTORY=${out_dir}|" docs/doxygen/Doxyfile.in | doxygen -)
}

rm -rf "$OUT_BASE"

for t in $TAGS; do
  echo "==> Building ${t}"
  wt="${WORKTREE_BASE}/${t}"
  git -C "$REPO_ROOT" worktree add --detach -q "$wt" "$t"
  build_doxygen "$wt" "${OUT_BASE}/${t}"
done

echo "==> Copying latest release (${LATEST}) to root for backward compatibility"
cp -a "${OUT_BASE}/${LATEST}/." "${OUT_BASE}/"

echo "Done. Versions available: $(echo $TAGS | tr '\n' ' ')"
echo "Root of docs/api/doxygen/ now mirrors ${LATEST}."
