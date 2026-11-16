#!/usr/bin/env bash
# Publish the built MkDocs site/ directory onto the orphan gh-pages branch.
# Only the requested directory (v1 or v2) is replaced. When absent, the root
# index.html redirects visitors to v2/, and the root 404.html sends links from
# the old unversioned site to the same page under v2/.
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
WORK="$(mktemp -d)"
ASKPASS="$(mktemp)"
cleanup() {
  rm -rf "$WORK"
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

# Sets RESULT to pushed, unchanged, or rejected. Any other failure exits the script.
publish_once() {
  local publish="$1" status=0
  RESULT=""

  # Exit code 2 means the branch does not exist yet. Anything else non-zero is an access or network failure.
  git_auth ls-remote --exit-code --heads "$REMOTE" gh-pages >/dev/null || status=$?
  case "$status" in
    0) git_auth clone --quiet --branch gh-pages --depth 1 "$REMOTE" "$publish" ;;
    2)
      git init -q -b gh-pages "$publish"
      git -C "$publish" remote add origin "$REMOTE"
      ;;
    *)
      echo "Cannot read gh-pages from ${GITHUB_REPOSITORY} (git ls-remote exit ${status})." >&2
      exit 1
      ;;
  esac

  # Identity applies only to the temporary publish checkout.
  git -C "$publish" config user.email "github-actions[bot]@users.noreply.github.com"
  git -C "$publish" config user.name "github-actions[bot]"

  if [ ! -f "${publish}/index.html" ]; then
    cat > "${publish}/index.html" <<'EOF'
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
  if [ ! -f "${publish}/404.html" ]; then
    cat > "${publish}/404.html" <<'EOF'
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Page not found - Flexiv TDK Documentation</title>
  <script>
    (function () {
      var siteRoot = "/flexiv_tdk/";
      var path = location.pathname;
      if (path.indexOf(siteRoot) !== 0) return;
      var rest = path.slice(siteRoot.length);
      if (/^(v1|v2)(\/|$)/.test(rest)) return;
      location.replace(siteRoot + "v2/" + rest + location.search + location.hash);
    })();
  </script>
</head>
<body>
  <p>Page not found. Open the <a href="/flexiv_tdk/v2/">v2</a> or <a href="/flexiv_tdk/v1/">v1</a> documentation.</p>
</body>
</html>
EOF
  fi
  touch "${publish}/.nojekyll"

  rm -rf "${publish:?}/${VERSION_DIR}"
  mkdir -p "${publish}/${VERSION_DIR}"
  cp -a "${SITE}/." "${publish}/${VERSION_DIR}/"

  git -C "$publish" add -A -- "$VERSION_DIR" index.html 404.html .nojekyll
  if git -C "$publish" diff --cached --quiet; then
    RESULT=unchanged
    return
  fi

  git -C "$publish" commit -q -m "Publish ${VERSION_DIR} documentation"
  if git_auth -C "$publish" push origin HEAD:gh-pages; then
    RESULT=pushed
  else
    RESULT=rejected
  fi
}

# The v1 and v2 workflows publish independently, so a push can lose a race to the other line.
ATTEMPTS=3
for attempt in $(seq 1 "$ATTEMPTS"); do
  rm -rf "${WORK:?}/publish"
  publish_once "${WORK}/publish"
  case "$RESULT" in
    pushed)
      echo "Published ${VERSION_DIR} to gh-pages."
      exit 0
      ;;
    unchanged)
      echo "No documentation changes to publish."
      exit 0
      ;;
  esac
  echo "Push to gh-pages was rejected (attempt ${attempt}/${ATTEMPTS})." >&2
  if [ "$attempt" -lt "$ATTEMPTS" ]; then
    sleep $((attempt * 5))
  fi
done

echo "Could not publish ${VERSION_DIR} to gh-pages after ${ATTEMPTS} attempts." >&2
exit 1
