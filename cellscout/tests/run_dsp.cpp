// Runs the baseband DSP on a PC.
//   run_dsp search FILE [cfo_center_hz [cfo_steps [offset]]]
//   run_dsp spectrum FILE
// FILE holds signed 8 bit I/Q at 3.84 MS/s, as the radio delivers it.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include "ltecellscout_dsp.hpp"

int main(int argc, char** argv) {
    if (argc < 3) return 2;
    FILE* f = std::fopen(argv[2], "rb");
    if (!f) return 2;
    std::vector<lcs::Sample> raw;
    lcs::Sample block[2048];
    size_t n;
    while ((n = std::fread(block, sizeof(lcs::Sample), 2048, f)) > 0) raw.insert(raw.end(), block, block + n);
    std::fclose(f);

    const long offset = argc > 5 ? std::atol(argv[5]) : 0;
    if (raw.size() < static_cast<size_t>(offset) + 2 * lcs::capture_length) {
        std::fprintf(stderr, "file too short\n");
        return 2;
    }
    // decimate in DMA sized pieces, as the baseband does
    std::vector<lcs::Sample> capture(lcs::capture_length);
    lcs::Decimator decimator;
    for (int done = 0; done < lcs::capture_length; done += 1024) {
        const int take = lcs::capture_length - done < 1024 ? lcs::capture_length - done : 1024;
        decimator.process(&raw[offset + 2 * done], 2 * take, &capture[done]);
    }
    const bool wants_spectrum = !std::strcmp(argv[1], "spectrum");
    lcs::remove_dc(capture.data(), wants_spectrum ? lcs::spectrum_length : lcs::capture_length);

    auto work = std::make_unique<lcs::Workspace>();
    lcs::init(*work);

    if (wants_spectrum) {
        uint8_t bins[128];
        lcs::spectrum(*work, capture.data(), bins);
        for (int k = 0; k < 128; k++) std::printf("%d%c", bins[k], k == 127 ? '\n' : ' ');
        return 0;
    }

    lcs_link::Cell cells[lcs_link::max_cells];
    const auto start = std::chrono::steady_clock::now();
    const int found = lcs::search(*work, capture.data(), argc > 3 ? std::atoi(argv[3]) : 0, argc > 4 ? std::atoi(argv[4]) : 7, cells, lcs_link::max_cells);
    const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    for (int i = 0; i < found; i++) {
        const auto& c = cells[i];
        std::printf("cell pci=%d tdd=%d extcp=%d mib=%d n_rb=%d ports=%d rsrp=%.1f rssi=%.1f rsrq=%.1f sinr=%.1f cfo=%d quality=%d tentative=%d\n",
                    c.pci, !!(c.flags & lcs_link::flag_tdd), !!(c.flags & lcs_link::flag_extended_cp), !!(c.flags & lcs_link::flag_mib),
                    c.n_rb, c.ports, c.rsrp_x10 / 10.0, c.rssi_x10 / 10.0, c.rsrq_x10 / 10.0, c.sinr_x10 / 10.0, c.cfo_hz, c.quality, !!(c.flags & lcs_link::flag_tentative));
    }
    std::fprintf(stderr, "found %d in %.0f ms; workspace %zu bytes\n", found, ms, sizeof(lcs::Workspace));
    return 0;
}
