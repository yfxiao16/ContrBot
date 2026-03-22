#!/bin/sh

# GridWorld Robot Controller Synthesis Demo
# Usage: ./run_demo.sh <working_directory>
#
# Grid Layout (3x3 with obstacle at (1,1)):
#
#   (0,0) R1 Depot  |  (1,0) Free     |  (2,0) R2 Depot
#   (0,1) Free      |  (1,1) OBSTACLE |  (2,1) Free
#   (0,2) R2 Stn    |  (1,2) Free     |  (2,2) R1 Stn
#
# Actions: Stay=0, North=1, South=2, East=3, West=4
# Robot1: depot=(0,0), station=(2,2)
# Robot2: depot=(2,0), station=(0,2)

WD=$1

if test "$#" -ne 1; then
    echo "Usage: ./run_demo.sh <working_directory>"
    exit 1
fi

mkdir -p "$WD"

# Tool paths -- override via environment variables or edit these defaults.
# Default layout assumes this demo lives inside the CHASE project tree:
#   project_root/
#     chase/bin/logics_tool
#     slugs/src/slugs
#     slugs/tools/StructuredSlugsParser/compiler.py
#     third_party/antlr4/lib/
#     demos/GridWorld/   <-- this directory
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOGICS_TOOL="${LOGICS_TOOL:-${SCRIPT_DIR}/../../chase/bin/logics_tool}"
SLUGS="${SLUGS:-${SCRIPT_DIR}/../../slugs/src/slugs}"
SLUGSCOMPILER="${SLUGSCOMPILER:-${SCRIPT_DIR}/../../slugs/tools/StructuredSlugsParser/compiler.py}"
ANTLR4_LIB="${ANTLR4_LIB:-${SCRIPT_DIR}/../../third_party/antlr4/lib}"

export DYLD_LIBRARY_PATH="${ANTLR4_LIB}:${DYLD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${ANTLR4_LIB}:${LD_LIBRARY_PATH}"

echo "============================================"
echo " GridWorld Robot Controller Synthesis Demo"
echo "============================================"
echo ""

# -----------------------------------------------
# Part 1: Single Robot Synthesis
# -----------------------------------------------
echo ">>> PART 1: Single Robot Direct Synthesis"
echo "    Grid: 3x3 with obstacle at (1,1)"
echo "    Depot: (0,0), Station: (2,2)"
echo ""

echo "--- Generating structuredSlugs specification ---"
"$LOGICS_TOOL" -i single_robot.ltl -o "$WD" -c single_synthesis.chase
echo ""

if [ -f "$WD/robot.structuredSlugs" ]; then
    echo "--- Compiling to Slugs input format ---"
    python3 "$SLUGSCOMPILER" "$WD/robot.structuredSlugs" > "$WD/robot.slugsin"

    echo "--- Running Slugs reactive synthesis ---"
    "$SLUGS" --explicitStrategy --jsonOutput "$WD/robot.slugsin" > "$WD/robot.json" 2>&1

    if [ $? -eq 0 ]; then
        echo "    Single robot FSM synthesized: $WD/robot.json"
    else
        echo "    Synthesis result saved to: $WD/robot.json"
    fi
else
    echo "    [SKIP] structuredSlugs file not generated"
fi
echo ""

# -----------------------------------------------
# Part 2: Multi-Robot Composition & Verification
# -----------------------------------------------
echo ">>> PART 2: Multi-Robot Composition & Refinement Check"
echo "    Robot1: depot=(0,0), station=(2,2)"
echo "    Robot2: depot=(2,0), station=(0,2)"
echo "    Collision avoidance + obstacle at (1,1)"
echo ""

echo "--- Composing Robot1 || Robot2 and generating refinement SMV ---"
"$LOGICS_TOOL" -i specs.ltl -o "$WD" -c verification.chase -V
echo ""

if [ -f "$WD/refinement.smv" ]; then
    echo "--- Checking refinement with NuXMV ---"
    if command -v nuxmv >/dev/null 2>&1; then
        ref=$(nuxmv "$WD/refinement.smv" | grep -c "is true")
        if [ "$ref" -ge 2 ]; then
            echo "    Refinement Check: PASSED"
        else
            echo "    Refinement Check: FAILED (check $WD/refinement.smv)"
        fi
    else
        echo "    [SKIP] nuxmv not found. Install NuXMV to verify refinement."
        echo "    Generated SMV file: $WD/refinement.smv"
    fi
else
    echo "    [SKIP] refinement.smv not generated"
fi
echo ""

# -----------------------------------------------
# Part 3: Per-Robot Controller Synthesis
# -----------------------------------------------
echo ">>> PART 3: Per-Robot Controller Synthesis"
echo ""

echo "--- Generating per-robot structuredSlugs specifications ---"
"$LOGICS_TOOL" -i specs.ltl -o "$WD" -c synthesis.chase
echo ""

for robot in robot1 robot2; do
    if [ -f "$WD/${robot}.structuredSlugs" ]; then
        echo "--- Synthesizing ${robot} controller ---"
        python3 "$SLUGSCOMPILER" "$WD/${robot}.structuredSlugs" > "$WD/${robot}.slugsin"
        "$SLUGS" --explicitStrategy --jsonOutput "$WD/${robot}.slugsin" > "$WD/${robot}.json" 2>&1

        if [ $? -eq 0 ]; then
            echo "    ${robot} FSM synthesized: $WD/${robot}.json"
        else
            echo "    ${robot} synthesis result: $WD/${robot}.json"
        fi
    else
        echo "    [SKIP] ${robot}.structuredSlugs not generated"
    fi
done
echo ""

echo "============================================"
echo " Demo Complete"
echo "============================================"
echo ""
echo "Generated files in $WD/:"
ls -la "$WD/" 2>/dev/null
