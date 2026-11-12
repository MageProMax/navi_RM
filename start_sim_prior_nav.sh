#!/usr/bin/env bash
# Replica of start_real_prior_nav.sh using the same prior map and navigation core.
set -eo pipefail
WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$WORKSPACE_DIR/src/rm_bringup/sim_bringup/scripts/start_sim_prior_nav.sh" "$@"
