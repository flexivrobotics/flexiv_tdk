#!/usr/bin/env bash
# Publish the built MkDocs site/ directory onto the orphan gh-pages branch.
# Only the requested directory (v1 or v2) is replaced. The root index.html,
# when absent, redirects visitors to v2/.
#
# Usage: bash docs/publish_gh_pages.sh v2
# Runs from GitHub Actions, which provides GITHUB_TOKEN and GITHUB_REPOSITORY.

set -euo pipefail

VERSION_DIR="${1:?usage: publish_gh_pages.sh v1|v2}"
case "$VERSION_DIR" in
  v1|v2) ;;
  *)
    echo "version directory must be v1 or v2" >&2
    exit 1
    ;;
esac

if [ -z "${GITHUB_TOKEN:-}" ] || [ -z "${GITHUB_REPOSITORY:-}" ]; then
  echo "This publisher runs in GitHub Actions, where GITHUB_TOKEN and GITHUB_REPOSITORY are set." >&2
  exit 1
fi

REPO_ROOT="$(git rev-parse --show-toplevel)"
SITE="${REPO_ROOT}/site"
if [ ! -d "$SITE" ]; then
  echo "Missing ${SITE}. Run mkdocs build first." >&2
  exit 1
fi

SERVER="${GITHUB_SERVER_URL:-https://github.com}"
SERVER="${SERVER#https://}"
SERVER="${SERVER#http://}"
# Username only. The token stays in the environment and is supplied when Git asks for a password.
REMOTE="https://x-access-token@${SERVER}/${GITHUB_REPOSITORY}.git"
PUBLISH="$(mktemp -d)"
ASKPASS="$(mktemp)"
cleanup() {
  rm -rf "$PUBLISH"
  rm -f "$ASKPASS"
}
trap cleanup EXIT

cat > "$ASKPASS" << 'EOF'
#!/bin/sh
printf '%s\n' "$GITHUB_TOKEN"
EOF
chmod 700 "$ASKPASS"
export GIT_ASKPASS="$ASKPASS"
export GIT_TERMINAL_PROMPT=0

git_auth() {
  # Drop the runner credential helper. It is configured for the source checkout, not this clone.
  git -c credential.helper= "$@"
}

if git_auth ls-remote --exit-code --heads "$REMOTE" gh-pages >/dev/null 2>&1; then
  git_auth clone --branch gh-pages --depth 1 "$REMOTE" "$PUBLISH"
else
  git init -b gh-pages "$PUBLISH"
  git -C "$PUBLISH" remote add origin "$REMOTE"
fi

# Identity applies only to the temporary publish checkout.
git -C "$PUBLISH" config user.email "github-actions[bot]@users.noreply.github.com"
git -C "$PUBLISH" config user.name "github-actions[bot]"

if [ ! -f "${PUBLISH}/index.html" ]; then
  cat > "${PUBLISH}/index.html" <<'EOF'
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Flexiv TDK Documentation</title>
  <meta http-equiv="refresh" content="0; url=v2/">
  <link rel="canonical" href="https://flexivrobotics.github.io/flexiv_tdk/v2/">
  <script>location.replace("v2/");</script>
</head>
<body>
  <p><a href="v2/">Flexiv TDK documentation</a></p>
</body>
</html>
EOF
fi
touch "${PUBLISH}/.nojekyll"

rm -rf "${PUBLISH:?}/${VERSION_DIR}"
mkdir -p "${PUBLISH}/${VERSION_DIR}"
cp -a "${SITE}/." "${PUBLISH}/${VERSION_DIR}/"

git -C "$PUBLISH" add -A -- "$VERSION_DIR" index.html .nojekyll
if git -C "$PUBLISH" diff --cached --quiet; then
  echo "No documentation changes to publish."
  exit 0
fi

git -C "$PUBLISH" commit -m "Publish ${VERSION_DIR} documentation"
git_auth -C "$PUBLISH" push origin HEAD:gh-pages
echo "Published ${VERSION_DIR} to gh-pages."
