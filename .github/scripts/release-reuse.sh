#!/usr/bin/env bash
# Which test suites can a release reuse from an earlier green run? (test.yml, `changes` job,
# called by release-stm32.yml with release: true.)
#
# A suite is reused when a Tests run on main (a push, the nightly or a manual run, never a pull
# request) passed every job of that suite on a commit C, and nothing that suite reads changed
# between C and the commit being released. Otherwise the suite runs again, as before. The
# release-only checks (check-release, gen_features/gen_errors --check-release, the signing) are
# in release-stm32.yml and always run.
#
# usage: release-reuse.sh <owner/repo> <release sha>
#   prints "<suite>=true|false" lines (true: run it) for $GITHUB_OUTPUT, notes on stderr.
#   needs gh with a token that can read actions (GH_TOKEN), and git in a checkout of the repo.
# Any error leaves every suite at true: a failure here costs time, never a check.
set -uo pipefail

repo=$1
head=$2

# suite name, the files it reads (the same expressions as the pull request filter in test.yml),
# and the jobs that must all have passed
suites=(ice40 stm32 bridge)
declare -A paths jobs
paths[ice40]='^(ice40/|tools/(gen_protocol|ice40_release|test_ice40_release)\.py$|stm32h563/src/(cmd_opcodes|fpga_config)\.h$|\.github/workflows/test\.yml$)'
paths[stm32]='^(stm32h563/|errors/|features/|tools/gen_(errors|features)\.py$|esp32-hosted-slave/|wifi-bridge/components/|\.gitmodules$|\.github/workflows/test\.yml$)'
paths[bridge]='^(wifi-bridge/|stm32h563/test/vectors/fwsign|stm32h563/keys/|tools/fwsign\.py$|\.github/workflows/test\.yml$)'
jobs[ice40]='ice40-sim'
jobs[stm32]='stm32-host'
jobs[bridge]='wifi-bridge-host wifi-bridge-build'

declare -A run
for s in "${suites[@]}"; do run[$s]=true; done

finish() {
  for s in "${suites[@]}"; do echo "$s=${run[$s]}"; done
  exit 0
}

# the newest 50 completed Tests runs on main, not from pull requests
if ! runs=$(gh api "repos/$repo/actions/workflows/test.yml/runs?branch=main&status=completed&per_page=50" \
    --jq '.workflow_runs[] | select(.event == "push" or .event == "schedule" or .event == "workflow_dispatch") | "\(.id) \(.head_sha) \(.html_url)"'); then
  echo "could not list the Tests runs: running every suite" >&2
  finish
fi

left=${#suites[@]}
while read -r id sha url; do
  [ -n "$id" ] || continue
  [ "$left" -gt 0 ] || break
  if ! git cat-file -e "$sha^{commit}" 2>/dev/null &&
     ! git fetch -q --depth=1 --filter=blob:none origin "$sha" 2>/dev/null; then
    continue
  fi
  if ! changed=$(git diff --name-only --no-renames "$sha" "$head"); then continue; fi
  if ! passed=$(gh api "repos/$repo/actions/runs/$id/jobs?per_page=100" \
      --jq '.jobs[] | select(.conclusion == "success") | .name'); then
    continue
  fi
  for s in "${suites[@]}"; do
    [ "${run[$s]}" = true ] || continue
    if grep -Eq "${paths[$s]}" <<< "$changed"; then continue; fi
    ok=true
    for j in ${jobs[$s]}; do grep -qx "$j" <<< "$passed" || ok=false; done
    if [ "$ok" = true ]; then
      run[$s]=false
      left=$((left - 1))
      echo "$s: reusing ${jobs[$s]} of $url (commit ${sha:0:12}; nothing it reads changed since)" >&2
    fi
  done
done <<< "$runs"

for s in "${suites[@]}"; do
  [ "${run[$s]}" = true ] && echo "$s: no green run covers $head: running it" >&2
done
finish
