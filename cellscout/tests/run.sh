#!/usr/bin/env bash
# Build the DSP for this PC and run the synthetic tests.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build data
for variant in "run_dsp" "run_trace -DLCS_TRACE"; do
    read -r name flags <<<"$variant"
    g++ -O2 -std=c++17 -Wall -Wextra ${flags:-} -I../firmware/baseband -I../firmware/common \
        tests/run_dsp.cpp ../firmware/baseband/ltecellscout_dsp.cpp -o "build/$name"
done
g++ -O1 -std=c++17 -Wall -Wextra -I../firmware/application/external/ltecellscout \
    tests/test_logic.cpp -o build/test_logic
build/test_logic ../sdcard/CELLSCOUT/UK.TXT ../sdcard/CELLSCOUT/IT.TXT
g++ -O2 -std=c++17 -Wall -Wextra -I../firmware/baseband -I../firmware/common \
    tests/test_decimator.cpp ../firmware/baseband/ltecellscout_dsp.cpp -o build/test_decimator
build/test_decimator
python3 tests/test_dsp.py
