#!/bin/sh
# Generate both configuration variants:
#   1. Distributed — per-robot controllers with directional collision avoidance
#   2. Centralized — request-grant architecture with Arbiter
#
# Usage: ./generate_all.sh [output_root]
#   Default output_root: out/

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OUTPUT_ROOT="${1:-${SCRIPT_DIR}/out}"

NUM_ROBOTS=2
GRID_SIZE=3
OBSTACLES="1,1"
COMMON="--grid-size $GRID_SIZE --num-robots $NUM_ROBOTS --obstacles $OBSTACLES --variable-targets --variable-obstacles 1"

echo "Generating 2 variants into ${OUTPUT_ROOT}/"
echo ""

# 1. Distributed
python3 "${SCRIPT_DIR}/generate_gridworld.py" $COMMON \
    --control-mode distributed \
    --output-dir "${OUTPUT_ROOT}/distributed/"
echo ""

# 2. Centralized (request-grant)
python3 "${SCRIPT_DIR}/generate_gridworld.py" $COMMON \
    --control-mode centralized \
    --output-dir "${OUTPUT_ROOT}/central/"
echo ""

echo "Done. Generated variants:"
ls -d "${OUTPUT_ROOT}"/*/
