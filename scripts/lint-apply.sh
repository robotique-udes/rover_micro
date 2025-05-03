#!/bin/bash

# Enable strict error handling
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR="$SCRIPT_DIR/.."

# Skip these directories (space-separated)
SKIP_DIRS=".platformio .github"

# Build the find command components
PRUNE_CMD=()
for dir in $SKIP_DIRS; do
    PRUNE_CMD+=(-o -path "$ROOT_DIR/$dir" -prune)
done

# Remove the first "-o" as it's not needed for the first condition
PRUNE_CMD=("${PRUNE_CMD[@]:1}")

# Finding the .clang-format file
echo "=== Finding .clang-format file... ==="
if [ -z "$(find "$ROOT_DIR" -name '.clang-format' -print -quit)" ]; then
    echo "[FAILED] .clang-format file not found in the repository!"
    exit 1
else
    echo -e "\e[0;32m[OK]\e[0m .clang-format file found"
fi

# Applying clang-format with directory skipping
echo "=== Applying lint (skipping $SKIP_DIRS)... ==="
find "$ROOT_DIR" \
    \( "${PRUNE_CMD[@]}" \) -o \
    \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) -print -exec clang-format -style=file -i {} \;

echo -e "\e[0;32m[SUCCESS]\e[0m Linting applied successfully (skipped $SKIP_DIRS)!"
