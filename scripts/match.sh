#!/bin/bash
# Launch a full 6-robot match with per-robot auto strategies (issue: lineup
# picker v1). Usage: scripts/match.sh [strategy_dir]
# robot_<n>.json drives robot n; falls back to example.json.
set -e
cd "$(dirname "$0")/.."
STRAT_DIR="${1:-strategies}"
BIN=./target/debug
[ -x ./target/release/server ] && BIN=./target/release

$BIN/server --seed 7 &
SERVER_PID=$!
sleep 1
AGENT_PIDS=()
for i in 0 1 2 3 4 5; do
  f="$STRAT_DIR/robot_$i.json"
  [ -f "$f" ] || f="$STRAT_DIR/example.json"
  $BIN/strategy_agent --robot-id "$i" --strategy "$f" &
  AGENT_PIDS+=($!)
done
trap "kill $SERVER_PID ${AGENT_PIDS[*]} 2>/dev/null" EXIT
echo "match running — server + ${#AGENT_PIDS[@]} strategy agents"
wait
