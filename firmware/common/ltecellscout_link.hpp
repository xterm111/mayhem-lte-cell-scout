/*
 * LTE Cell Scout: the data exchanged between the application core (M0) and the
 * baseband core (M4). It lives in shared_memory.bb_data, so neither the shared
 * memory layout nor the message list of the stock firmware changes.
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#ifndef __LTECELLSCOUT_LINK_H__
#define __LTECELLSCOUT_LINK_H__

#include <stdint.h>

namespace lcs_link {

constexpr uint32_t adc_rate = 3'840'000;  // the baseband decimates this by two
constexpr uint32_t filter_bandwidth = 1'750'000;
constexpr int32_t cfo_step_hz = 7'500;
constexpr int max_cells = 6;
constexpr int spectrum_bins = 128;  // 15 kHz each, lowest frequency first

enum class Op : uint8_t {
    Spectrum = 1,
    Search = 2,
};

enum CellFlags : uint8_t {
    flag_tdd = 1,
    flag_extended_cp = 2,
    flag_mib = 4,  // n_rb and ports are valid
    flag_tentative = 8,  // weak evidence: believe it only if it shows up again
};

struct Command {
    volatile uint32_t seq;  // written last by the M0
    Op op;
    uint8_t cfo_steps;      // search cfo_center_hz +- cfo_steps * cfo_step_hz
    uint16_t reserved;
    int32_t cfo_center_hz;
};

struct Cell {
    uint16_t pci;
    uint8_t flags;
    uint8_t n_rb;   // 6, 15, 25, 50, 75 or 100
    uint8_t ports;  // CRS antenna ports: 1, 2 or 4
    uint8_t quality;  // coherence of the reference signals, 0..100
    int16_t rsrp_x10;  // dB relative to full scale, per resource element
    int16_t rssi_x10;  // dB relative to full scale, central 6 resource blocks
    int16_t rsrq_x10;  // dB
    int16_t sinr_x10;  // dB
    int16_t reserved;
    int32_t cfo_hz;  // carrier frequency error seen by this receiver
};

enum Trouble : uint8_t {
    trouble_clipped = 1,  // the ADC reached full scale
    trouble_dropped = 2,  // sample blocks were lost: results are worthless
};

struct Response {
    volatile uint32_t seq;  // equals Command::seq when the answer is complete
    uint8_t cell_count;
    uint8_t trouble;  // bits of Trouble
    uint16_t millis;  // processing time of this answer on the M4
    union {
        Cell cells[max_cells];
        uint8_t spectrum[spectrum_bins];  // dB above an arbitrary floor, 0.5 dB steps
    };
};

struct Link {
    Command command;
    Response response;
};

static_assert(sizeof(Link) <= 512, "does not fit in shared_memory.bb_data");

}  // namespace lcs_link

#endif
