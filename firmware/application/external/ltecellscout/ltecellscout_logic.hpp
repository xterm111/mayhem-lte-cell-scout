/*
 * LTE Cell Scout: bands, preset files and the walk of a blind search.
 *
 * No firmware dependencies, so that the tests can run this on a PC. Everything
 * stays in the app's namespace, which keeps it out of the firmware image.
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#ifndef __LTECELLSCOUT_LOGIC_H__
#define __LTECELLSCOUT_LOGIC_H__

#include <stdint.h>

namespace ui::external_app::ltecellscout {

constexpr int max_operators = 8;
constexpr int max_carriers = 64;
constexpr int max_blocks = 16;
constexpr int max_channels = 1000;  // 100 kHz steps in the widest band (B40)

/* European LTE bands. Frequencies are in units of 100 kHz, the EARFCN step. */
struct Band {
    uint8_t number;
    uint16_t low;  // downlink frequency of the first EARFCN
    uint16_t first, last;
};
constexpr int band_count = 9;
static constexpr Band bands[band_count] = {
    {1, 21100, 0, 599},
    {3, 18050, 1200, 1949},
    {7, 26200, 2750, 3449},
    {8, 9250, 3450, 3799},
    {20, 7910, 6150, 6449},
    {28, 7580, 9210, 9659},
    {32, 14520, 9920, 10359},
    {38, 25700, 37750, 38249},
    {40, 23000, 38650, 39649}};

inline int band_index(uint32_t number) {
    for (int i = 0; i < band_count; i++)
        if (bands[i].number == number) return i;
    return -1;
}

inline int band_of(uint32_t earfcn) {
    for (int i = 0; i < band_count; i++)
        if (earfcn >= bands[i].first && earfcn <= bands[i].last) return i;
    return -1;
}

inline uint32_t channel_of(int band, uint32_t earfcn) {
    return bands[band].low + earfcn - bands[band].first;
}

/* Decimal number; with scale 10, one decimal is read and the result is tenths. */
inline uint32_t parse_number(const char* s, int scale) {
    uint32_t value = 0;
    while (*s >= '0' && *s <= '9') value = value * 10 + (*s++ - '0');
    value *= scale;
    if (scale == 10 && *s == '.' && s[1] >= '0' && s[1] <= '9') value += s[1] - '0';
    return value;
}

inline void copy_text(char* to, const char* from, int size) {
    int i = 0;
    for (; i < size - 1 && from[i]; i++) to[i] = from[i];
    to[i] = 0;
}

/* What a preset file says. */
struct Preset {
    struct Operator {
        char plmn[8];
        char name[10];
    } operators[max_operators];
    struct Carrier {
        uint16_t earfcn;
        uint8_t mask;  // one bit per operator
    } carriers[max_carriers];
    struct Block {
        uint16_t low, high;  // 100 kHz units
        uint8_t mask;
    } blocks[max_blocks];
    uint8_t operator_count, carrier_count, block_count;
    uint16_t sweep_mask;  // one bit per entry of the band table

    /* "234-10+234-20" to a bit per known operator */
    uint8_t mask_of(const char* list) const {
        uint8_t mask = 0;
        while (*list) {
            for (int i = 0; i < operator_count; i++) {
                const char* code = operators[i].plmn;
                int n = 0;
                while (code[n] && code[n] == list[n]) n++;
                if (!code[n] && (!list[n] || list[n] == '+')) mask |= 1 << i;
            }
            while (*list && *list != '+') list++;
            if (*list) list++;
        }
        return mask;
    }

    /* OP,plmn,name   CARRIER,earfcn,width,plmns,...   BLOCK,low,high,plmns
     * SWEEP,band     Anything else is ignored. The line is modified. */
    void parse_line(char* line) {
        const char* end = line;
        while (*end) end++;
        const char* fields[4];
        for (auto& field : fields) field = end;  // missing fields are empty
        const char kind = line[0];
        int count = 0;
        for (char* p = line; p < end; p++) {
            if (*p != ',') continue;
            *p = 0;
            if (count < 4) fields[count++] = p + 1;
        }

        if (kind == 'O' && operator_count < max_operators) {
            auto& op = operators[operator_count++];
            copy_text(op.plmn, fields[0], sizeof(op.plmn));
            copy_text(op.name, fields[1], sizeof(op.name));
        } else if (kind == 'C' && carrier_count < max_carriers) {
            auto& carrier = carriers[carrier_count++];
            carrier.earfcn = parse_number(fields[0], 1);
            carrier.mask = mask_of(fields[2]);
            const int band = band_of(carrier.earfcn);
            if (band >= 0) sweep_mask |= 1 << band;
        } else if (kind == 'B' && block_count < max_blocks) {
            auto& block = blocks[block_count++];
            block.low = parse_number(fields[0], 10);
            block.high = parse_number(fields[1], 10);
            block.mask = mask_of(fields[2]);
        } else if (kind == 'S') {
            const int band = band_index(parse_number(fields[0], 1));
            if (band >= 0) sweep_mask |= 1 << band;
        }
    }

    /* Operators using an EARFCN according to the file. */
    uint8_t operators_at(uint16_t earfcn) const {
        for (int i = 0; i < carrier_count; i++)
            if (carriers[i].earfcn == earfcn && carriers[i].mask) return carriers[i].mask;
        const int band = band_of(earfcn);
        if (band < 0) return 0;
        const uint32_t channel = channel_of(band, earfcn);
        for (int i = 0; i < block_count; i++)
            if (channel > blocks[i].low && channel < blocks[i].high) return blocks[i].mask;
        return 0;
    }
};

/* Where the centre of a carrier is likely to be above the lower edge of the
 * occupied spectrum: 20, 10, 15 and 5 MHz wide. */
static constexpr uint8_t guesses[4] = {90, 45, 67, 22};

/* Without a list: visits the occupied parts of a band. Positions are 100 kHz
 * steps from the bottom of the band. Each visit must cover 200 kHz either side. */
struct Walker {
    int16_t cursor;
    int16_t edge;  // start of the occupied stretch being examined, or -1
    int8_t guess;

    void reset() {
        cursor = 0;
        edge = -1;
        guess = 0;
    }

    bool next(const uint8_t* levels, int width, int threshold, int& position) {
        auto occupied = [&](int at) { return at < width && levels[at] >= threshold; };
        while (true) {
            if (edge < 0) {
                // three samples across 1 MHz, so that a narrow spur does not count
                while (cursor < width && !(occupied(cursor) && occupied(cursor + 5) && occupied(cursor + 10))) cursor++;
                if (cursor >= width) return false;
                edge = cursor;
                guess = 0;
                cursor += 6;
            }
            if (guess < 4) {
                position = edge + guesses[guess++];
                if (occupied(position)) return true;
                continue;
            }
            if (!occupied(cursor)) {
                edge = -1;
                cursor++;
                continue;
            }
            position = cursor;
            cursor += 4;
            return true;
        }
    }

    /* A carrier of n_rb resource blocks (0 if unknown) was found at position. */
    void found(int position, int n_rb) {
        // half the width of a carrier, in 100 kHz, equals n_rb from 15 up
        const int half = n_rb > 7 ? n_rb : 7;
        // Skip it only if it starts about where the occupied stretch does.
        // Otherwise something below it has not been found yet.
        if (edge >= 0 && position - half > edge + 12) return;
        if (position + half + 1 > cursor) cursor = position + half + 1;
        edge = -1;
    }
};

}  // namespace ui::external_app::ltecellscout

#endif
