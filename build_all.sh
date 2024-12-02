#!/bin/bash

set -e

echo -e "\e[0;34m=== Downloading PlatformIO and dependencies ... ===\e[0m"
python3 -m pip install --upgrade pip
pip install platformio
pip install --upgrade requests urllib3 chardet
echo -e "\e[0;32m[OK]\e[0m"
echo ""

find . -name platformio.ini | while read platformio_file; do
project_dir=$(dirname "$platformio_file")
project_name=$(basename "$project_dir")
echo -e "\e[0;34m=== Building project: $project_name ... ===\e[0m"
if (cd "$project_dir" && pio run --target clean && PLATFORMIO_BUILD_FLAGS="-Werror" pio run); then
    echo ""
else
    exit 1  # Exit immediately on failure
fi
done
