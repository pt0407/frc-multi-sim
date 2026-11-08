#!/bin/bash
# FRC sim launcher: ensures the physics server (9771) and vite dev server
# (5173) are running, then opens the viz. Extra args pass to `server`,
# e.g. "FRC Sim.app lineup" -> default 6-robot field.
REPO="/Users/pranav/Desktop/DevinProjects/frc-sim-m0"
cd "$REPO" || exit 1

# vite dev server (web viz)
if ! curl -sf -o /dev/null http://localhost:5173/ ; then
  (cd web && nohup npm run dev >/tmp/frc-vite.log 2>&1 &)
  sleep 2
fi

# physics/spectator server
if ! lsof -iTCP:9771 -sTCP:LISTEN >/dev/null 2>&1; then
  if [ "$#" -eq 0 ]; then
    set -- --num-robots 1 --archetype 2910
  fi
  nohup ./target/release/server "$@" >/tmp/frc-server.log 2>&1 &
  sleep 2
fi

open "http://localhost:5173"
