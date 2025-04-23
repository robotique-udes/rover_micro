#!/bin/bash

set -e

echo -e "\e[0;34m=== Downloading PlatformIO and dependencies ... ===\e[0m"
python3 -m pip install --upgrade pip
pip install platformio
pip install --upgrade requests urllib3 chardet
echo -e "\e[0;32m[OK]\e[0m"
echo ""

find ../. -name platformio.ini | while read platformio_file; do
  project_dir=$(dirname "$platformio_file")
  project_name=$(basename "$project_dir")

  # --- Build Phase ---
  echo -e "\e[0;34m=== Building project: $project_name ... ===\e[0m"
  if (cd "$project_dir" && pio run --target clean && PLATFORMIO_BUILD_FLAGS="-Werror" pio run); then
    echo -e "\e[0;32m[BUILD PASSED]\e[0m"
  else
    echo -e "\e[0;31m[BUILD FAILED]\e[0m"
    exit 1
  fi

  # --- Test Phase ---
  if (cd "$project_dir" && pio project config | grep -q -E "test_|_test"); then
    echo -e "\e[0;34m=== Running tests for $project_name ... ===\e[0m"
    if (cd "$project_dir" && pio test -e test_off_device); then  # Replace with your test env name
      echo -e "\e[0;32m[TEST PASSED]\e[0m"
    else
      echo -e "\e[0;31m[TEST FAILED]\e[0m"
      exit 1
    fi
  else
    echo -e "\e[0;33m[NO TESTS FOUND - SKIPPING]\e[0m"
  fi
  
  echo ""
done
