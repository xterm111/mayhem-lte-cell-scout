# LTE Cell Scout

A receive-only LTE downlink measuring app for HackRF One + PortaPack running
**official Mayhem v2.4.0**. It runs from the SD card.

- Finds cells on European bands: EARFCN, PCI, bandwidth, RSRP, SINR
- Monitors one channel: the cells on it, RSRP/RSRQ/SINR/RSSI, antenna ports and
  a 24 MHz spectrum
- Shows operator names from a preset file; they are not read off the air

## Build

You need podman. Clone with `--recurse-submodules`, take `firmware_hackrf.bin`
from the official v2.4.0 release, then from the top folder:

    podman build -t portapack-dev -f dockerfile-nogit .
    mkdir -p cellscout/reference
    head -c 691564 firmware_hackrf.bin > cellscout/reference/application.bin
    cellscout/tools/build.sh

The app calls into the firmware at fixed addresses, so the build checks that the
firmware image still matches the official one.

## Install

Copy the contents of `cellscout/dist/` to an SD card that holds the v2.4.0 SD
content. Open **Receive > LTE Cell Scout**.

## Use

| Control | Meaning |
|---|---|
| LNA, VGA, AMP | receiver gains |
| CAL | dB added to power readings |
| List | preset: none, or a file from `CELLSCOUT/` |
| Band | search one band, or all |
| Search | find cells; select a result to monitor it |
| middle number | EARFCN to monitor (Select switches the dial step, 1 or 100) |
| Monitor | measure that EARFCN repeatedly; select in its list to go back |

| On screen | Meaning |
|---|---|
| `PLMN?` | taken from the preset |
| `?` after a PCI | weak evidence |
| `~` after a PCI | not seen in the last capture |
| `*` after RSRP | CAL is 0 |
| `CLIP` | input too strong: lower the gains |
| `DROP` | samples were lost: a bug, please report it |
| `ms` | processing time of the last cell search |

## Good to know

- Where a list names channels in a band, only those are searched. Set List to
  none to search blind. Blind search can miss weak carriers.
- A weak cell may show up in one search and not the next. A second cell on the
  same channel is usually seen if 3 dB weaker, rarely if 6 dB weaker.
- Power readings are relative. A single CAL value cannot cover how this radio
  varies with frequency and gain, so compare within one band, with the same
  antenna and gains. RSRQ and SINR do not depend on CAL.
- Tried on hardware below 1 GHz only. TDD, extended cyclic prefix and 4 antenna
  ports are tested on synthetic signals only.
- No NR, no PLMN decoding. Up to 40 results and 8 monitored cells.
- The UK and Italy presets come from public sources and are not verified.

## Tests

    cellscout/tests/run.sh      # needs g++ and python3 with numpy

The synthetic signals only check the decoder against our own reading of the
standard. For a check against real ones, put recordings
(`hackrf_transfer -s 3840000 -b 1750000`) in `cellscout/data/field/` and list
the expected cells in `expected.json` there; the format is in `test_dsp.py`.
