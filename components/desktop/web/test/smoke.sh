#!/usr/bin/env bash
# Headless-Chrome smoke test for components/desktop/web/desktop.html:
#   1. the page runs to the end of its script from file:// with and without
#      ?autoconnect=1 (no exception before the "ready" log line);
#   2. the ?selftest=1 hook (a fake transport answering discovery /
#      GET_DESKTOP / LAUNCH_APP / CLOSE_WINDOW, the d2h golden vectors from
#      test/desktop_vectors.txt fed through the parser, synthetic drag /
#      resize / minimise / dialog answers) ends with the title SELFTEST PASS.
#
#   components/desktop/web/test/smoke.sh            (needs Google Chrome)
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
page="$here/../desktop.html"
vectors="$here/../../test/desktop_vectors.txt"
chrome="${CHROME:-/Applications/Google Chrome.app/Contents/MacOS/Google Chrome}"
if [ ! -x "$chrome" ]; then
  for c in google-chrome chromium chromium-browser; do
    if command -v "$c" >/dev/null 2>&1; then chrome="$(command -v "$c")"; break; fi
  done
fi
[ -x "$chrome" ] || { echo "smoke: no Chrome found (set CHROME=/path/to/chrome)"; exit 2; }
[ -f "$page" ] && [ -f "$vectors" ] || { echo "smoke: page or vectors missing"; exit 2; }

dump() { # url -> DOM on stdout
  "$chrome" --headless=new --disable-gpu \
    --window-size=1280,900 --virtual-time-budget=5000 --dump-dom "$1" 2>/dev/null
}
fail=0
dom="$(mktemp)"
trap 'rm -f "$dom"' EXIT
for q in "" "?autoconnect=1"; do
  # the DOM is captured to a file first (a grep -q closing Chrome's pipe early
  # would abort the dump under pipefail), then the RENDERED state is checked:
  # the script's last statement stamps data-ready on <body>, which a throw
  # anywhere before it leaves out (the script's own literals are no proof)
  dump "file://$page$q" > "$dom" || true
  if grep -q '<body data-ready="desktop"' "$dom"; then
    echo "PASS page runs to the end of its script (file://desktop.html$q)"
  else
    echo "FAIL page did not run to the end of its script (file://desktop.html$q)"; fail=1
  fi
done
# the vectors travel in the URL hash (fetch() of a file:// sibling is blocked)
v="$(base64 < "$vectors" | tr -d '\n' | tr '+/' '-_' | tr -d '=')"
dump "file://$page?selftest=1#v=$v" > "$dom" || true
title="$(sed -n 's/.*<title>\([^<]*\)<\/title>.*/\1/p' "$dom" | head -1)"
if [ "$title" = "SELFTEST PASS" ]; then
  echo "PASS selftest: $title"
else
  echo "FAIL selftest: ${title:-no title}"
  sed '/<script>/,$d' "$dom" | grep -o 'SELFTEST FAIL[^<]*' | head -3 || true  # rendered log only, not the script literals
  fail=1
fi
exit $fail
