/*
 * LTE Cell Scout: LTE downlink cell search and measurement.
 *
 * Steps: PSS correlation (timing, N_ID2, coarse frequency), SSS detection
 * (N_ID1, duplex mode, cyclic prefix, frame timing), reference signal
 * measurement (RSRP, RSSI, RSRQ, SINR) and PBCH decoding (bandwidth, ports).
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include "ltecellscout_dsp.hpp"

#ifdef LCS_TRACE  // PC tests only
#include <cstdio>
#define TRACE(...) std::fprintf(stderr, __VA_ARGS__)
#else
#define TRACE(...)
#endif

namespace lcs {
namespace {

constexpr int fft_advance = 2;  // start FFT windows slightly inside the prefix
constexpr int block_lags = 512 - 128 + 1;
constexpr int first_lag = 480;  // room for a TDD SSS ahead of the first PSS
constexpr int max_candidates = 16;
constexpr float pi_f = 3.14159265f;

/* --- small maths --------------------------------------------------------- */

inline cf mul(cf a, cf b) {
    return {a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re};
}
inline cf mul_conj(cf a, cf b) {  // a * conj(b)
    return {a.re * b.re + a.im * b.im, a.im * b.re - a.re * b.im};
}
inline float norm(cf a) {
    return a.re * a.re + a.im * a.im;
}
inline float root(float x) {
    return __builtin_sqrtf(x);
}

/* e^(j 2 pi turns) */
cf phasor(float turns) {
    turns -= static_cast<float>(static_cast<int32_t>(turns));
    if (turns > 0.5f) turns -= 1.0f;
    if (turns < -0.5f) turns += 1.0f;
    float sign = 1.0f;
    if (turns > 0.25f) {
        turns -= 0.5f;
        sign = -1.0f;
    } else if (turns < -0.25f) {
        turns += 0.5f;
        sign = -1.0f;
    }
    const float x = turns * 2.0f * pi_f;
    const float x2 = x * x;
    const float s = x * (1.0f + x2 * (-1.0f / 6 + x2 * (1.0f / 120 + x2 * (-1.0f / 5040 + x2 * (1.0f / 362880)))));
    const float c = 1.0f + x2 * (-0.5f + x2 * (1.0f / 24 + x2 * (-1.0f / 720 + x2 * (1.0f / 40320 + x2 * (-1.0f / 3628800)))));
    return {sign * c, sign * s};
}

/* argument of z in turns, -0.5 .. 0.5 */
float turns_of(cf z) {
    const float ax = z.re < 0 ? -z.re : z.re;
    const float ay = z.im < 0 ? -z.im : z.im;
    if (ax + ay == 0.0f) return 0.0f;
    const float r = (ax > ay) ? ay / ax : ax / ay;
    const float r2 = r * r;
    float a = r * (0.99866f + r2 * (-0.32993f + r2 * (0.18027f + r2 * (-0.08517f + r2 * 0.02084f))));
    if (ay > ax) a = pi_f / 2 - a;
    if (z.re < 0) a = pi_f - a;
    if (z.im < 0) a = -a;
    return a / (2 * pi_f);
}

/* 10 * log10(x) * 10, for decibel results with one decimal */
int16_t decibel_x10(float x) {
    if (x < 1e-12f) return -1200;
    union {
        float f;
        uint32_t u;
    } v{x};
    const int exponent = static_cast<int>(v.u >> 23) - 127;
    v.u = (v.u & 0x007fffff) | 0x3f800000;
    const float m = v.f;  // 1 .. 2
    const float log2m = -2.153620718f + m * (3.047884161f + m * (-1.051875031f + m * 0.15824871f));
    const float db10 = 30.1029996f * (static_cast<float>(exponent) + log2m);
    return static_cast<int16_t>(db10 + (db10 < 0 ? -0.5f : 0.5f));
}

/* In place radix 2 FFT, n a power of two up to 512. */
__attribute__((optimize("O3"))) void fft(const Workspace& w, cf* x, int n) {
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j |= bit;
        if (i < j) {
            const cf t = x[i];
            x[i] = x[j];
            x[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        const int half = len >> 1;
        const int stride = 512 / len;
        for (int k = 0; k < half; k++) {
            const cf t = w.twiddle[k * stride];
            for (int i = k; i < n; i += len) {
                const cf v = mul(x[i + half], t);
                const cf u = x[i];
                x[i] = {u.re + v.re, u.im + v.im};
                x[i + half] = {u.re - v.re, u.im - v.im};
            }
        }
    }
}

/* --- sequences ----------------------------------------------------------- */

constexpr uint8_t pss_root[3] = {25, 29, 34};

/* PSS in the 128 point frequency grid (36.211 6.11.1). */
void pss_grid(int nid2, cf* grid) {
    for (int k = 0; k < 128; k++) grid[k] = {0, 0};
    const uint32_t u = pss_root[nid2];
    for (uint32_t n = 0; n < 62; n++) {
        // exp(-j pi u m (m + 1) / 63), with m = n for the lower half and n + 1 above
        const uint32_t m = (n < 31) ? n * (n + 1) : (n + 1) * (n + 2);
        const cf p = phasor(static_cast<float>((u * m) % 126) / 126.0f);
        const int bin = (n < 31) ? 128 - 31 + n : n - 30;
        grid[bin] = {p.re, -p.im};
    }
}

/* Time domain PSS: 128 samples, written to out. */
void pss_time(const Workspace& w, int nid2, cf* out) {
    pss_grid(nid2, out);
    for (int k = 0; k < 128; k++) out[k].im = -out[k].im;
    fft(w, out, 128);
    for (int k = 0; k < 128; k++) out[k] = {out[k].re / 128, -out[k].im / 128};
}

/* One of the length 31 m-sequences of 6.11.2.1 as a bit mask. */
uint32_t m_sequence(uint32_t taps) {
    uint32_t bits = 1u << 4;
    for (int i = 0; i < 26; i++) {
        const uint32_t next = __builtin_parity((bits >> i) & taps);
        bits |= next << (i + 5);
    }
    return bits;
}

struct SssTables {
    uint32_t s, c, z;
};

/* Fills d with the 62 values (+1 / -1) of the secondary sync signal. */
void sss_sequence(const SssTables& t, int nid1, int nid2, int subframe5, int8_t* d) {
    const int qp = nid1 / 30;
    const int q = (nid1 + qp * (qp + 1) / 2) / 30;
    const int mp = nid1 + q * (q + 1) / 2;
    const int m0 = mp % 31;
    const int m1 = (m0 + mp / 31 + 1) % 31;
    const int first = subframe5 ? m1 : m0;
    const int second = subframe5 ? m0 : m1;
    const int scramble = first % 8;  // z takes only the low three bits (6.11.2.1)
    for (int n = 0; n < 31; n++) {
        const uint32_t even = (t.s >> ((n + first) % 31)) ^ (t.c >> ((n + nid2) % 31));
        const uint32_t odd = (t.s >> ((n + second) % 31)) ^ (t.c >> ((n + nid2 + 3) % 31)) ^ (t.z >> ((n + scramble) % 31));
        d[2 * n] = (even & 1) ? -1 : 1;
        d[2 * n + 1] = (odd & 1) ? -1 : 1;
    }
}

/* Length 31 Gold sequence (7.2). */
class Gold {
   public:
    Gold(uint32_t c_init, int offset)
        : x1_(1), x2_(c_init) {
        for (int i = 0; i < 1600 + offset; i++) next();
    }
    uint32_t next() {
        const uint32_t out = (x1_ ^ x2_) & 1;
        const uint32_t n1 = ((x1_ >> 3) ^ x1_) & 1;
        const uint32_t n2 = ((x2_ >> 3) ^ (x2_ >> 2) ^ (x2_ >> 1) ^ x2_) & 1;
        x1_ = (x1_ >> 1) | (n1 << 30);
        x2_ = (x2_ >> 1) | (n2 << 30);
        return out;
    }

   private:
    uint32_t x1_, x2_;
};

/* --- samples ------------------------------------------------------------- */

/* Copies count samples from start, removing a frequency error of cfo Hz. */
void load(const Sample* capture, int start, int count, float cfo, cf* out) {
    const float turns_per_sample = -cfo / sample_rate;
    cf rotation = phasor(turns_per_sample * start);
    const cf step = phasor(turns_per_sample);
    for (int n = 0; n < count; n++) {
        const cf x{static_cast<float>(capture[start + n].i), static_cast<float>(capture[start + n].q)};
        out[n] = mul(x, rotation);
        rotation = mul(rotation, step);
    }
}

/* Spectrum of the OFDM symbol whose useful part starts at start. */
void symbol_fft(Workspace& w, const Sample* capture, int start, float cfo, cf* out) {
    load(capture, start - fft_advance, 128, cfo, out);
    fft(w, out, 128);
    for (int k = 0; k < 128; k++) out[k] = {out[k].re / 128, out[k].im / 128};
}

/* The 72 subcarriers around DC, lowest first. */
inline int central_bin(int k) {
    return (k < 36) ? 128 - 36 + k : k - 35;
}

/* Position of the useful part of symbol l within a slot. */
inline int symbol_offset(bool extended, int l) {
    return extended ? 32 + 160 * l : 10 + 137 * l;
}

/* --- PSS search ---------------------------------------------------------- */

struct Candidate {
    float metric;
    int lag;
    int shift;  // frequency error in FFT bins of 3.75 kHz
    int nid2;
};

void add_candidate(Candidate* list, int& count, const Candidate& c) {
    int weakest = 0;
    for (int i = 0; i < count; i++) {
        const int distance = list[i].lag - c.lag;
        const int offset = list[i].shift - c.shift;
        if (list[i].nid2 == c.nid2 && distance > -8 && distance < 8 && offset >= -4 && offset <= 4) {
            if (c.metric > list[i].metric) list[i] = c;
            return;
        }
        if (list[i].metric < list[weakest].metric) weakest = i;
    }
    if (count < max_candidates)
        list[count++] = c;
    else if (c.metric > list[weakest].metric)
        list[weakest] = c;
}

float candidate_floor(const Candidate* list, int count) {
    if (count < max_candidates) return 0.04f;
    float lowest = list[0].metric;
    for (int i = 1; i < count; i++)
        if (list[i].metric < lowest) lowest = list[i].metric;
    return lowest;
}

constexpr float ref_scale = 400.0f;  // keeps the PSS spectrum within int16

__attribute__((optimize("O3"))) void correlate_block(Workspace& w, int lag0, int nid2, int shift, Candidate* list, int& count) {
    for (int k = 0; k < 512; k++) {
        const cf x = w.spectrum[(k + shift) & 511];
        const float rr = w.pss_ref[nid2][k][0];
        const float ri = w.pss_ref[nid2][k][1];
        // conj(x * ref): a forward FFT then gives the conjugate of the inverse
        w.scratch[k] = {x.re * rr - x.im * ri, -(x.re * ri + x.im * rr)};
    }
    fft(w, w.scratch, 512);

    const float scale = 1.0f / (512.0f * ref_scale);
    const float gain = scale * scale / w.pss_ref_energy[nid2];
    float floor = candidate_floor(list, count);
    const int remaining = first_lag + half_frame - lag0;
    const int lags = remaining < block_lags ? remaining : block_lags;
    for (int lag = 0; lag < lags; lag++) {
        const float power = norm(w.scratch[lag]) * gain;
        if (power > floor * w.energy[lag]) {
            add_candidate(list, count, {power / w.energy[lag], lag0 + lag, shift, nid2});
            floor = candidate_floor(list, count);
        }
    }
}

int pss_search(Workspace& w, const Sample* capture, int32_t cfo_center_hz, int cfo_steps, Candidate* list) {
    int count = 0;
    const int center = (cfo_center_hz + (cfo_center_hz < 0 ? -1875 : 1875)) / 3750;

    for (int lag0 = first_lag; lag0 < first_lag + half_frame; lag0 += block_lags) {
        load(capture, lag0, 512, 0, w.spectrum);

        float running = 0;
        for (int n = 0; n < 128; n++) running += norm(w.spectrum[n]);
        for (int lag = 0; lag < block_lags; lag++) {
            w.energy[lag] = running > 128 ? running : 128;  // ignore silence
            if (lag + 128 < 512) running += norm(w.spectrum[lag + 128]) - norm(w.spectrum[lag]);
        }

        fft(w, w.spectrum, 512);
        for (int nid2 = 0; nid2 < 3; nid2++)
            for (int step = -cfo_steps; step <= cfo_steps; step++)
                correlate_block(w, lag0, nid2, center + 2 * step, list, count);
    }
    return count;
}

/* Adds the correlation seen one half frame later and sorts, best first. */
void confirm_candidates(Workspace& w, const Sample* capture, Candidate* list, int count) {
    for (int i = 0; i < count; i++) {
        Candidate& c = list[i];
        pss_time(w, c.nid2, w.spectrum);
        float best = 0;
        for (int delta = -1; delta <= 1; delta++) {
            const int at = c.lag + half_frame + delta;
            load(capture, at, 128, c.shift * 3750.0f, w.scratch);
            cf sum{0, 0};
            float energy = 128;
            for (int n = 0; n < 128; n++) {
                const cf p = mul_conj(w.scratch[n], w.spectrum[n]);
                sum.re += p.re;
                sum.im += p.im;
                energy += norm(w.scratch[n]);
            }
            const float m = norm(sum) / (energy * w.pss_ref_energy[c.nid2]);
            if (m > best) best = m;
        }
        c.metric += best;
    }
    for (int i = 1; i < count; i++) {
        const Candidate c = list[i];
        int j = i;
        for (; j > 0 && list[j - 1].metric < c.metric; j--) list[j] = list[j - 1];
        list[j] = c;
    }
}

/* --- SSS ----------------------------------------------------------------- */

struct Sync {
    float metric;
    int nid1;
    int frame_start;  // sample at which subframe 0 starts, may be negative
    uint8_t flags;
};

/* Correlates one SSS symbol against all 336 sequences, adding into scores
 * (index nid1 * 2 + subframe of the FIRST half frame). */
void sss_correlate(Workspace& w, const SssTables& tables, const Sample* capture, int pss_at, int sss_at, int nid2, float cfo, bool coherent, int half, float* scores) {
    cf z[62];
    cf reference[62];
    pss_grid(nid2, w.scratch);
    for (int n = 0; n < 62; n++) reference[n] = w.scratch[(n < 31) ? 128 - 31 + n : n - 30];
    symbol_fft(w, capture, pss_at, cfo, w.spectrum);
    symbol_fft(w, capture, sss_at, cfo, w.scratch);
    cf h[62];
    for (int n = 0; n < 62; n++) {
        const int bin = (n < 31) ? 128 - 31 + n : n - 30;
        h[n] = mul_conj(w.spectrum[bin], reference[n]);
    }
    float energy = 1e-9f;
    for (int n = 0; n < 62; n++) {
        const int bin = (n < 31) ? 128 - 31 + n : n - 30;
        // neighbouring subcarriers see nearly the same channel: average five
        cf smooth{0, 0};
        for (int k = (n < 2 ? 0 : n - 2); k <= (n > 59 ? 61 : n + 2); k++) {
            smooth.re += h[k].re;
            smooth.im += h[k].im;
        }
        z[n] = mul_conj(w.scratch[bin], smooth);
        energy += norm(z[n]);
    }
    const float scale = 1.0f / root(energy * 62.0f);

    // The frequency error not yet removed turns every subcarrier of the SSS by
    // the same angle against the PSS, so only the magnitude of the sum counts.
    int8_t d[62];
    for (int nid1 = 0; nid1 < 168; nid1++) {
        for (int sf = 0; sf < 2; sf++) {
            sss_sequence(tables, nid1, nid2, sf ^ half, d);
            cf sum{0, 0};
            for (int n = 0; n < 62; n++) {
                sum.re += z[n].re * d[n];
                sum.im += z[n].im * d[n];
            }
            scores[nid1 * 2 + sf] += (coherent ? sum.re : root(norm(sum))) * scale;
        }
    }
}

Sync sss_detect(Workspace& w, const SssTables& tables, const Sample* capture, const Candidate& c, const float* anchor) {
    // PSS to SSS distance and the cell type it implies (FDD / TDD, CP length)
    static const int distance[4] = {137, 160, 412, 480};
    static const uint8_t type[4] = {0, lcs_link::flag_extended_cp, lcs_link::flag_tdd, lcs_link::flag_tdd | lcs_link::flag_extended_cp};

    Sync best{0, 0, 0, 0};
    float* scores = w.energy;  // 336 of the 385 entries
    const float cfo = anchor ? *anchor : c.shift * 3750.0f;
    for (int position = 0; position < 4; position++) {
        for (int i = 0; i < 336; i++) scores[i] = 0;
        int halves = 0;
        for (int half = 0; half < 2; half++) {
            const int pss_at = c.lag + half * half_frame;
            const int sss_at = pss_at - distance[position];
            if (sss_at - fft_advance < 0 || pss_at + 128 > capture_length) continue;
            sss_correlate(w, tables, capture, pss_at, sss_at, c.nid2, cfo, anchor != nullptr, half, scores);
            halves++;
        }
        for (int i = 0; i < 336; i++) {
            const float metric = scores[i] / halves;
            if (metric > best.metric) {
                const bool extended = type[position] & lcs_link::flag_extended_cp;
                const bool tdd = type[position] & lcs_link::flag_tdd;
                // PSS: last symbol of slot 0 (FDD) or third symbol of subframe 1 (TDD)
                const int in_frame = tdd ? 1920 + symbol_offset(extended, 2) : symbol_offset(extended, extended ? 5 : 6);
                best = {metric, i / 2, c.lag - in_frame - (i & 1) * half_frame, type[position]};
            }
        }
    }
    return best;
}

/* --- frequency and reference signals ------------------------------------- */

/* Subframes 0 and 5 are always downlink and carry the normal reference signals. */
template <typename F>
void for_each_sync_subframe(int frame_start, F&& use) {
    int start = frame_start;
    int number = 0;
    while (start >= half_frame) {
        start -= half_frame;
        number ^= 5;
    }
    while (start < 0) {
        start += half_frame;
        number ^= 5;
    }
    for (; start + 1920 <= capture_length; start += half_frame, number ^= 5)
        use(start, number);
}

/* Frequency error from the cyclic prefixes, ambiguous by multiples of 15 kHz. */
float prefix_frequency(const Sample* capture, int frame_start, bool extended, float coarse) {
    cf sum{0, 0};
    const int symbols = extended ? 6 : 7;
    for_each_sync_subframe(frame_start, [&](int start, int) {
        for (int slot = 0; slot < 2; slot++) {
            for (int l = 0; l < symbols; l++) {
                const int useful = start + slot * 960 + symbol_offset(extended, l);
                const int prefix = extended ? 32 : (l == 0 ? 10 : 9);
                for (int i = 2; i < prefix - 1; i++) {
                    const Sample a = capture[useful - prefix + i];
                    const Sample b = capture[useful + 128 - prefix + i];
                    sum.re += b.i * a.i + b.q * a.q;
                    sum.im += b.q * a.i - b.i * a.q;
                }
            }
        }
    });
    float error = turns_of(sum) * 15000.0f - coarse;
    while (error > 7500.0f) error -= 15000.0f;
    while (error < -7500.0f) error += 15000.0f;
    return coarse + error;
}

/* Pilot values of one antenna port in one symbol: 12 estimates of the channel
 * at central subcarriers first, first + 6, ... Returns first. */
int pilots(Workspace& w, const Sample* capture, int subframe_start, int subframe, int slot, int l, int port, int pci, bool extended, float cfo, cf* out) {
    const int ns = subframe * 2 + slot;
    int v;
    if (port < 2) {
        v = ((l == 0) == (port == 0)) ? 0 : 3;
    } else {
        v = 3 * (ns & 1) + (port == 3 ? 3 : 0);
    }
    const int first = (v + pci % 6) % 6;

    symbol_fft(w, capture, subframe_start + slot * 960 + symbol_offset(extended, l), cfo, w.scratch);
    const uint32_t c_init = 1024u * (7 * (ns + 1) + l + 1) * (2 * pci + 1) + 2 * pci + (extended ? 0 : 1);
    Gold gold(c_init, 2 * 104);  // the central 6 resource blocks use m' = 104 .. 115
    constexpr float a = 0.70710678f;
    for (int m = 0; m < 12; m++) {
        const cf r{gold.next() ? -a : a, 0};
        const cf reference{r.re, gold.next() ? -a : a};
        out[m] = mul_conj(w.scratch[central_bin(first + 6 * m)], reference);
    }
    return first;
}

struct Measurement {
    float rsrp, noise, rssi, coherence;
    float cfo_residual;
};

Measurement measure(Workspace& w, const Sample* capture, int frame_start, int pci, bool extended, float cfo) {
    cf cross{0, 0};
    float p1 = 0, p2 = 0, rssi = 0;
    int pairs = 0, symbols = 0;
    const int second_symbol = extended ? 3 : 4;

    for_each_sync_subframe(frame_start, [&](int start, int subframe) {
        for (int which = 0; which < 2; which++) {
            const int l = which ? second_symbol : 0;
            cf h[2][12];
            for (int slot = 0; slot < 2; slot++) {
                pilots(w, capture, start, subframe, slot, l, 0, pci, extended, cfo, h[slot]);
                for (int k = 0; k < 72; k++) rssi += norm(w.scratch[central_bin(k)]);
                symbols++;
            }
            for (int m = 0; m < 12; m++) {
                const cf product = mul_conj(h[1][m], h[0][m]);
                cross.re += product.re;
                cross.im += product.im;
                p1 += norm(h[0][m]);
                p2 += norm(h[1][m]);
                pairs++;
            }
        }
    });

    // The same subcarriers one slot apart: what correlates is signal, the rest noise.
    const float magnitude = root(norm(cross));
    Measurement result{};
    result.rsrp = magnitude / pairs;
    result.noise = (p1 + p2) / (2 * pairs) - result.rsrp;
    if (result.noise < result.rsrp * 1e-4f) result.noise = result.rsrp * 1e-4f;
    result.rssi = rssi / symbols;
    result.coherence = 2 * magnitude / (p1 + p2 + 1e-9f);
    result.cfo_residual = turns_of(cross) * 2000.0f;  // slots are 0.5 ms apart
    return result;
}

/* --- PBCH ---------------------------------------------------------------- */

/* Least squares estimates of one port on all 72 subcarriers around subframe 0. */
void estimate_channel(Workspace& w, const Sample* capture, int start, int port, int pci, bool extended, float cfo) {
    cf sum[72];
    uint8_t seen[72];
    for (int k = 0; k < 72; k++) {
        sum[k] = {0, 0};
        seen[k] = 0;
    }
    const int second_symbol = extended ? 3 : 4;
    struct Source {
        int8_t slot, l;
    };
    const Source normal_ports[3] = {{0, static_cast<int8_t>(second_symbol)}, {1, 0}, {1, static_cast<int8_t>(second_symbol)}};
    const Source extra_ports[1] = {{1, 1}};
    const Source* sources = (port < 2) ? normal_ports : extra_ports;
    const int source_count = (port < 2) ? 3 : 1;

    for (int s = 0; s < source_count; s++) {
        cf h[12];
        const int first = pilots(w, capture, start, 0, sources[s].slot, sources[s].l, port, pci, extended, cfo, h);
        for (int m = 0; m < 12; m++) {
            const int k = first + 6 * m;
            sum[k].re += h[m].re;
            sum[k].im += h[m].im;
            seen[k]++;
        }
    }

    // average the repeats, then interpolate linearly between pilot subcarriers
    int8_t known[72];
    int n = 0;
    for (int k = 0; k < 72; k++) {
        if (!seen[k]) continue;
        sum[k] = {sum[k].re / seen[k], sum[k].im / seen[k]};
        known[n++] = k;
    }
    int j = 0;
    for (int k = 0; k < 72; k++) {
        while (j < n - 2 && k > known[j + 1]) j++;
        const int a = known[j], b = known[j + 1];
        const float t = static_cast<float>(k - a) / (b - a);
        w.channel[port][k] = {sum[a].re + t * (sum[b].re - sum[a].re), sum[a].im + t * (sum[b].im - sum[a].im)};
    }
}

/* Tail biting Viterbi decoder for the rate 1/3, K = 7 code (36.212 5.1.3.1).
 * soft[3 * bit + stream], positive meaning 0. */
void viterbi(const float* soft, uint8_t* bits, ViterbiWork& work) {
    static const uint8_t generator[3] = {0133, 0171, 0165};
    uint8_t output[128];  // coded bits for every (input << 6 | state)
    for (int r = 0; r < 128; r++) {
        output[r] = __builtin_parity(r & generator[0]) | (__builtin_parity(r & generator[1]) << 1) | (__builtin_parity(r & generator[2]) << 2);
    }
    auto& metric = work.metric;
    auto& decision = work.decision;
    for (int s = 0; s < 64; s++) metric[0][s] = 0;

    // three passes over the block stand in for the circular trellis
    for (int step = 0; step < 120; step++) {
        const float* in = &soft[3 * (step % 40)];
        float branch[8];
        for (int c = 0; c < 8; c++)
            branch[c] = ((c & 1) ? -in[0] : in[0]) + ((c & 2) ? -in[1] : in[1]) + ((c & 4) ? -in[2] : in[2]);
        const float* old = metric[step & 1];
        float* next = metric[(step & 1) ^ 1];
        decision[step][0] = decision[step][1] = 0;
        for (int s = 0; s < 64; s++) {
            const int input = s >> 5;
            const int from = (s & 31) << 1;
            const float m0 = old[from] + branch[output[(input << 6) | from]];
            const float m1 = old[from + 1] + branch[output[(input << 6) | (from + 1)]];
            if (m1 > m0) {
                next[s] = m1;
                decision[step][s >> 5] |= 1u << (s & 31);
            } else {
                next[s] = m0;
            }
        }
    }

    int state = 0;
    for (int s = 1; s < 64; s++)
        if (metric[0][s] > metric[0][state]) state = s;
    for (int step = 119; step >= 0; step--) {
        if (step >= 40 && step < 80) bits[step - 40] = state >> 5;
        const int bit = (decision[step][state >> 5] >> (state & 31)) & 1;
        state = ((state & 31) << 1) | bit;
    }
}

uint32_t crc16(const uint8_t* bits, int count) {
    uint32_t crc = 0;
    for (int i = 0; i < count; i++) {
        const uint32_t feedback = ((crc >> 15) ^ bits[i]) & 1;
        crc = (crc << 1) & 0xffff;
        if (feedback) crc ^= 0x1021;
    }
    return crc;
}

/* Position in the 120 coded bits of the circular buffer entry i (36.212 5.1.4.2). */
void rate_matching_order(uint8_t* order) {
    static const uint8_t column[32] = {1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31,
                                       0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30};
    int n = 0;
    for (int stream = 0; stream < 3; stream++) {
        for (int c = 0; c < 32; c++) {
            for (int row = 0; row < 2; row++) {
                const int index = row * 32 + column[c] - 24;  // 24 dummy bits lead
                if (index >= 0) order[n++] = 3 * index + stream;
            }
        }
    }
}

/* Undoes transmit diversity into llr, two values per QPSK symbol. */
int equalise(const Workspace& w, int ports, int pci, bool extended, uint16_t* re, float* llr) {
    // re: positions of the PBCH resource elements, symbol * 72 + subcarrier
    int count = 0;
    for (int l = 0; l < 4; l++) {
        const bool has_pilots = l < 2 || (extended && l == 3);
        for (int k = 0; k < 72; k++)
            if (!has_pilots || k % 3 != pci % 3) re[count++] = l * 72 + k;
    }
    const cf* y = &w.pbch_y[0][0];
    auto h = [&](int port, int at) { return w.channel[port][at % 72]; };

    int out = 0;
    if (ports == 1) {
        for (int i = 0; i < count; i++) {
            const cf d = mul_conj(y[re[i]], h(0, re[i]));
            llr[out++] = d.re;
            llr[out++] = d.im;
        }
        return out;
    }
    for (int i = 0; i + 1 < count; i += 2) {
        // two ports use (0, 1); four ports alternate between (0, 2) and (1, 3)
        const int a = (ports == 2) ? 0 : ((i >> 1) & 1);
        const int b = (ports == 2) ? 1 : a + 2;
        const int p = re[i], q = re[i + 1];
        // d0 = h_a*(p) y(p) + h_b(q) y*(q),  d1 = h_a*(q) y(q) - h_b(p) y*(p)
        const cf s0 = mul_conj(y[p], h(a, p));
        const cf t0 = mul_conj(h(b, q), y[q]);
        const cf s1 = mul_conj(y[q], h(a, q));
        const cf t1 = mul_conj(h(b, p), y[p]);
        llr[out++] = s0.re + t0.re;
        llr[out++] = s0.im + t0.im;
        llr[out++] = s1.re - t1.re;
        llr[out++] = s1.im - t1.im;
    }
    return out;
}

bool decode_pbch(Workspace& w, const Sample* capture, int subframe_start, int pci, bool extended, float cfo, uint8_t& n_rb, uint8_t& ports) {
    for (int l = 0; l < 4; l++) {
        symbol_fft(w, capture, subframe_start + 960 + symbol_offset(extended, l), cfo, w.scratch);
        for (int k = 0; k < 72; k++) w.pbch_y[l][k] = w.scratch[central_bin(k)];
    }
    for (int port = 0; port < 4; port++) estimate_channel(w, capture, subframe_start, port, pci, extended, cfo);

    uint8_t order[120];
    rate_matching_order(order);

    // the larger arrays live in buffers that are idle at this point
    float* llr = w.llr;
    ViterbiWork& work = w.viterbi;
    uint16_t* positions = w.positions;
    for (int antenna_ports = 1; antenna_ports <= 4; antenna_ports *= 2) {
        const int bits_per_frame = equalise(w, antenna_ports, pci, extended, positions, llr);
        for (int quarter = 0; quarter < 4; quarter++) {
            float soft[120];
            for (int i = 0; i < 120; i++) soft[i] = 0;
            Gold scramble(pci, quarter * bits_per_frame);
            for (int i = 0; i < bits_per_frame; i++)
                soft[order[(quarter * bits_per_frame + i) % 120]] += scramble.next() ? -llr[i] : llr[i];

            uint8_t bits[40];
            viterbi(soft, bits, work);
            uint32_t received = 0;
            for (int i = 24; i < 40; i++) received = (received << 1) | bits[i];
            const uint32_t mask = (antenna_ports == 1) ? 0x0000 : (antenna_ports == 2) ? 0xffff
                                                                                       : 0x5555;
            if ((crc16(bits, 24) ^ received) != mask) continue;
            const int code = bits[0] * 4 + bits[1] * 2 + bits[2];
            static const uint8_t resource_blocks[6] = {6, 15, 25, 50, 75, 100};
            if (code > 5) continue;
            n_rb = resource_blocks[code];
            ports = antenna_ports;
            return true;
        }
    }
    return false;
}

/* Looks at one PSS candidate. Returns false if it is not a cell. anchor is the
 * frequency error already established by a strong cell in this capture, or null:
 * every cell on a carrier shows the same one, that of this receiver. */
bool identify(Workspace& w, const SssTables& tables, const Sample* capture, const Candidate& c, const float* anchor, lcs_link::Cell& cell) {
    if (anchor) {
        const float difference = c.shift * 3750.0f - *anchor;
        if (difference > 8000.0f || difference < -8000.0f) return false;  // an echo of another peak
    }
    const Sync sync = sss_detect(w, tables, capture, c, anchor);
    TRACE("cand nid2=%d lag=%d cfo=%d pss=%.3f sss=%.3f nid1=%d type=%d frame=%d\n", c.nid2, c.lag, c.shift * 3750, c.metric, sync.metric, sync.nid1, sync.flags, sync.frame_start);
    if (sync.metric < 0.30f) return false;

    const bool extended = sync.flags & lcs_link::flag_extended_cp;
    const int pci = 3 * sync.nid1 + c.nid2;
    float cfo = anchor ? *anchor : prefix_frequency(capture, sync.frame_start, extended, c.shift * 3750.0f);
    Measurement m = measure(w, capture, sync.frame_start, pci, extended, cfo);
    TRACE("  pci=%d cfo=%.0f coherence=%.3f residual=%.0f\n", pci, cfo, m.coherence, m.cfo_residual);
    if (m.coherence < 0.22f) return false;
    if (!anchor && m.cfo_residual > -900.0f && m.cfo_residual < 900.0f) {
        cfo += m.cfo_residual;
        m = measure(w, capture, sync.frame_start, pci, extended, cfo);
    }

    uint8_t n_rb = 0, ports = 0;
    bool mib = false;
    for_each_sync_subframe(sync.frame_start, [&](int start, int subframe) {
        if (subframe == 0 && !mib) mib = decode_pbch(w, capture, start, pci, extended, cfo, n_rb, ports);
    });

    // A good checksum or clear signals settle it; anything less is tentative.
    const bool certain = mib || (sync.metric > 0.50f && m.coherence > 0.45f);

    cell = {};
    cell.pci = pci;
    cell.flags = sync.flags | (mib ? lcs_link::flag_mib : 0) | (certain ? 0 : lcs_link::flag_tentative);
    cell.n_rb = n_rb;
    cell.ports = ports;
    cell.quality = static_cast<uint8_t>(m.coherence >= 1.0f ? 100 : m.coherence * 100);
    constexpr float full_scale = 1.0f / (128.0f * 128.0f);
    cell.rsrp_x10 = decibel_x10(m.rsrp * full_scale);
    cell.rssi_x10 = decibel_x10(m.rssi * full_scale);
    cell.rsrq_x10 = decibel_x10(6 * m.rsrp / (m.rssi + 1e-9f));
    cell.sinr_x10 = decibel_x10(m.rsrp / (m.noise + 1e-12f));
    cell.cfo_hz = static_cast<int32_t>(cfo);
    return true;
}

}  // namespace

/* --- public -------------------------------------------------------------- */

__attribute__((optimize("O3"))) void Decimator::process(const Sample* in, int count, Sample* out) {
    // taps -1 0 9 16 9 0 -1, over 32. I and Q share one integer, I + 65536 Q,
    // which the filter, being linear, treats in a single pass.
    auto pack = [](Sample x) -> int32_t { return x.i + x.q * 65536; };
    auto clamp = [](int v) -> int8_t { return v > 127 ? 127 : (v < -128 ? -128 : v); };
    int32_t a = odd_[0], b = odd_[1], c = odd_[2], d = even_;
    for (int n = 0; n < count; n += 2) {
        const int32_t e = pack(in[n + 1]);
        const int32_t sum = 16 * d + 9 * (b + c) - (a + e) + 16 * 65536 + 16;  // rounded
        const int i = static_cast<int16_t>(sum);
        const int q = (sum - i) >> 16;
        out[n >> 1] = {clamp(i >> 5), clamp(q >> 5)};
        a = b;
        b = c;
        c = e;
        d = pack(in[n]);
    }
    odd_[0] = a;
    odd_[1] = b;
    odd_[2] = c;
    even_ = d;
}

void init(Workspace& w) {
    cf t{1, 0};
    const cf step{0.9999247018391445f, -0.012271538285719925f};  // e^(-j 2 pi / 512)
    for (int k = 0; k < 256; k++) {
        // exact values at the quarter points stop rounding errors building up
        if (k == 128) t = {0, -1};
        w.twiddle[k] = t;
        t = mul(t, step);
    }
    for (int nid2 = 0; nid2 < 3; nid2++) {
        pss_time(w, nid2, w.scratch);
        float energy = 0;
        for (int n = 0; n < 128; n++) energy += norm(w.scratch[n]);
        w.pss_ref_energy[nid2] = energy;
        for (int n = 128; n < 512; n++) w.scratch[n] = {0, 0};
        fft(w, w.scratch, 512);
        for (int k = 0; k < 512; k++) {
            w.pss_ref[nid2][k][0] = static_cast<int16_t>(w.scratch[k].re * ref_scale);
            w.pss_ref[nid2][k][1] = static_cast<int16_t>(-w.scratch[k].im * ref_scale);
        }
    }
}

void remove_dc(Sample* capture, int length) {
    int32_t i = 0, q = 0;
    for (int n = 0; n < length; n++) {
        i += capture[n].i;
        q += capture[n].q;
    }
    const int di = (i + (i < 0 ? -length / 2 : length / 2)) / length;
    const int dq = (q + (q < 0 ? -length / 2 : length / 2)) / length;
    if (di == 0 && dq == 0) return;
    for (int n = 0; n < length; n++) {
        const int a = capture[n].i - di, b = capture[n].q - dq;
        capture[n] = {static_cast<int8_t>(a > 127 ? 127 : (a < -128 ? -128 : a)),
                      static_cast<int8_t>(b > 127 ? 127 : (b < -128 ? -128 : b))};
    }
}

int search(Workspace& w, const Sample* capture, int32_t cfo_center_hz, int cfo_steps, lcs_link::Cell* cells, int max_cells) {
    const SssTables tables{m_sequence(0b00101), m_sequence(0b01001), m_sequence(0b10111)};
    Candidate candidates[max_candidates];
    int count = pss_search(w, capture, cfo_center_hz, cfo_steps, candidates);
    confirm_candidates(w, capture, candidates, count);

    // The first certain cell, in order of PSS strength, fixes the frequency.
    float anchor = 0;
    int first = 0;
    bool anchored = false;
    for (; first < count && !anchored; first++) {
        lcs_link::Cell cell;
        if (identify(w, tables, capture, candidates[first], nullptr, cell) && !(cell.flags & lcs_link::flag_tentative)) {
            anchor = cell.cfo_hz;
            anchored = true;
        }
    }

    // Look again at that frequency only, so that other timings are not crowded
    // out by copies of the strongest peak at neighbouring frequencies.
    if (anchored) {
        count = pss_search(w, capture, static_cast<int32_t>(anchor), 0, candidates);
        confirm_candidates(w, capture, candidates, count);
    }

    int found = 0;
    for (int i = 0; i < count; i++) {
        lcs_link::Cell cell;
        if (!identify(w, tables, capture, candidates[i], anchored ? &anchor : nullptr, cell)) continue;

        // keep one entry per PCI, the one with the better reference signals
        int slot = 0;
        while (slot < found && cells[slot].pci != cell.pci) slot++;
        if (slot < found) {
            const bool better_kind = (cells[slot].flags & lcs_link::flag_tentative) && !(cell.flags & lcs_link::flag_tentative);
            const bool same_kind = (cells[slot].flags ^ cell.flags) & lcs_link::flag_tentative ? false : true;
            if (!better_kind && !(same_kind && cell.quality > cells[slot].quality)) continue;
        } else if (found < max_cells) {
            found++;
        } else if (!(cell.flags & lcs_link::flag_tentative)) {
            // a certain cell may take the place of a tentative one
            for (slot = 0; slot < found && !(cells[slot].flags & lcs_link::flag_tentative);) slot++;
            if (slot == found) continue;
        } else {
            continue;
        }
        cells[slot] = cell;
    }
    for (int i = 1; i < found; i++) {
        const lcs_link::Cell c = cells[i];
        int j = i;
        for (; j > 0 && cells[j - 1].rsrp_x10 < c.rsrp_x10; j--) cells[j] = cells[j - 1];
        cells[j] = c;
    }
    return found;
}

void spectrum(Workspace& w, const Sample* capture, uint8_t* bins) {
    constexpr int blocks = spectrum_length / 128;
    float* power = w.energy;
    for (int k = 0; k < 128; k++) power[k] = 0;
    for (int b = 0; b < blocks; b++) {
        load(capture, b * 128, 128, 0, w.scratch);
        fft(w, w.scratch, 128);
        for (int k = 0; k < 128; k++) power[k] += norm(w.scratch[(k + 64) & 127]);
    }
    for (int k = 0; k < 128; k++) {
        // 0 is 30 dB below one least significant bit per sample
        const int db10 = decibel_x10(power[k] / (128.0f * 128.0f * blocks)) + 300;
        const int half_db = db10 / 5;
        bins[k] = half_db < 0 ? 0 : (half_db > 255 ? 255 : half_db);
    }
    bins[64] = (bins[63] + bins[65]) / 2;  // the receiver's own carrier sits here
}

}  // namespace lcs
