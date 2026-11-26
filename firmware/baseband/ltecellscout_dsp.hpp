/*
 * LTE Cell Scout: LTE downlink cell search and measurement.
 *
 * Portable code with no firmware or math library dependencies, so the same
 * source runs on the Cortex-M4 and in the tests on a PC. Section numbers refer
 * to 3GPP TS 36.211 and 36.212.
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#ifndef __LTECELLSCOUT_DSP_H__
#define __LTECELLSCOUT_DSP_H__

#include <stdint.h>
#include "ltecellscout_link.hpp"

namespace lcs {

constexpr int sample_rate = 1'920'000;
constexpr int half_frame = 9600;      // samples in 5 ms
constexpr int capture_length = 21504;  // 11.2 ms: a whole frame wherever it starts
constexpr int spectrum_length = 8192;  // all that spectrum() reads

struct Sample {
    int8_t i, q;
};

struct cf {
    float re, im;
};

struct ViterbiWork {
    float metric[2][64];
    uint32_t decision[120][2];
};

/* All working memory, supplied by the caller. The unions share buffers that are
 * never in use at the same time. */
struct Workspace {
    cf twiddle[256];
    union {
        cf spectrum[512];
        ViterbiWork viterbi;
    };
    union {
        cf scratch[512];
        float llr[480];
    };
    union {
        float energy[385];
        uint16_t positions[240];
    };
    int16_t pss_ref[3][512][2];  // conjugate spectrum of each PSS
    cf pbch_y[4][72];
    cf channel[4][72];
    float pss_ref_energy[3];
};

/* 2:1 half-band decimator from the ADC rate down to 1.92 MHz. It runs on every
 * block while capturing: if it takes longer than a block lasts, blocks are lost. */
class Decimator {
   public:
    /* Consumes count (even) input samples, writes count / 2. */
    void process(const Sample* in, int count, Sample* out);

   private:
    // the last three odd and the last even input sample, as packed I/Q
    int32_t odd_[3]{};
    int32_t even_{0};
};

void init(Workspace& w);
void remove_dc(Sample* capture, int length);

/* Finds and measures the cells in a capture_length capture centred on a carrier.
 * Returns the number of cells written, strongest first. */
int search(Workspace& w, const Sample* capture, int32_t cfo_center_hz, int cfo_steps, lcs_link::Cell* cells, int max_cells);

/* Average power in 128 bins of 15 kHz, lowest frequency first, 0.5 dB steps. */
void spectrum(Workspace& w, const Sample* capture, uint8_t* bins);

}  // namespace lcs

#endif
