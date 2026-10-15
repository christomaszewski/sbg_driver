#!/usr/bin/env bash
# check_dashboard_health.sh [dashboard-checkout]
#
# Cross-repo contract check: run this driver's serialized /diagnostics reports
# (src/sbg_driver/test/fixtures/health_diagnostics.json — real CDR, pinned by the
# test_health_publisher gtest) through the DASHBOARD's own decoder, health store,
# verdict/issue logic and Home health widget.
#
# The test is copied into the dashboard checkout for the run and removed again, so
# the dashboard repository is left untouched. Needs `npm install` done in <checkout>/app.
#
#   tools/check_dashboard_health.sh ~/ws/dashboard        # or DASHBOARD_DIR=...
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
DASHBOARD="${1:-${DASHBOARD_DIR:-$REPO/../dashboard}}"
APP="$DASHBOARD/app"
FIXTURE="$REPO/src/sbg_driver/test/fixtures/health_diagnostics.json"

[ -f "$APP/src/health/discovery.ts" ] || {
  echo "check_dashboard_health: no dashboard health sources under $APP (need the feat/health tree)" >&2
  exit 2
}
[ -x "$APP/node_modules/.bin/vitest" ] || {
  echo "check_dashboard_health: run 'npm install' in $APP first" >&2
  exit 2
}

STAGE="$APP/src/health/__sbg_compat__"
[ ! -e "$STAGE" ] || { echo "check_dashboard_health: $STAGE already exists — remove it first" >&2; exit 2; }
trap 'rm -rf "$STAGE"' EXIT
mkdir "$STAGE"
cp "$FIXTURE" "$STAGE/health_diagnostics.json"
cp "$REPO/tools/dashboard_compat/sbg_health.compat.test.tsx" "$STAGE/"

echo "check_dashboard_health: dashboard $(git -C "$DASHBOARD" rev-parse --short HEAD 2>/dev/null || echo '?')" \
     "($(git -C "$DASHBOARD" branch --show-current 2>/dev/null || echo '?'))" >&2
cd "$APP"
exec_status=0
node_modules/.bin/vitest run src/health/__sbg_compat__ || exec_status=$?
exit "$exec_status"
