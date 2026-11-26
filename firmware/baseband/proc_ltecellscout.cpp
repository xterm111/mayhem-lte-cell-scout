/*
 * LTE Cell Scout: baseband image.
 *
 * The baseband thread only fills a capture buffer. A low priority thread does
 * the processing, so messages from the application core are still answered.
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include "event_m4.hpp"
#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "message.hpp"
#include "portapack_shared_memory.hpp"

#include "ltecellscout_dsp.hpp"

namespace {

constexpr int settle_buffers = 8;  // about 4 ms for the synthesiser after a retune

enum class State : uint8_t {
    Idle,
    Settling,
    Capturing,
    Ready,
};

lcs::Workspace workspace;
lcs::Sample capture[lcs::capture_length];
lcs::Decimator decimator;

volatile State state = State::Idle;
volatile uint8_t trouble = 0;
uint16_t missed_before = 0;  // running count kept by the DMA code
int settle = 0;
int position = 0;
volatile int length = lcs::capture_length;  // samples wanted

lcs_link::Link& link() {
    return *reinterpret_cast<lcs_link::Link*>(&shared_memory.bb_data.data[0]);
}

WORKING_AREA(worker_area, 4096);  // the search needs about half of this

msg_t worker(void*) {
    lcs::init(workspace);
    uint32_t done = 0;
    lcs_link::Command command{};

    while (!chThdShouldTerminate()) {
        if (state == State::Idle && link().command.seq != done) {
            done = link().command.seq;
            __DMB();
            command.op = link().command.op;
            command.cfo_steps = link().command.cfo_steps;
            command.cfo_center_hz = link().command.cfo_center_hz;
            trouble = 0;
            settle = settle_buffers;
            position = 0;
            length = command.op == lcs_link::Op::Spectrum ? lcs::spectrum_length : lcs::capture_length;
            __DMB();
            state = State::Settling;
        } else if (state == State::Ready) {
            auto& response = link().response;
            const systime_t started = chTimeNow();
            lcs::remove_dc(capture, length);
            response.trouble = trouble;
            response.cell_count = 0;
            if (command.op == lcs_link::Op::Spectrum) {
                lcs::spectrum(workspace, capture, response.spectrum);
            } else {
                response.cell_count = lcs::search(workspace, capture, command.cfo_center_hz, command.cfo_steps,
                                                  response.cells, lcs_link::max_cells);
            }
            response.millis = static_cast<uint32_t>(chTimeNow() - started) * 1000 / CH_FREQUENCY;
            __DMB();
            response.seq = done;
            state = State::Idle;
        }
        chThdSleepMilliseconds(2);
    }
    return 0;
}

class CellScoutProcessor : public BasebandProcessor {
   public:
    CellScoutProcessor() {
        thread = chThdCreateStatic(worker_area, sizeof(worker_area), NORMALPRIO - 10, worker, nullptr);
        baseband_thread.start();
    }

    CellScoutProcessor(const CellScoutProcessor&) = delete;
    CellScoutProcessor& operator=(const CellScoutProcessor&) = delete;

    ~CellScoutProcessor() {
        chThdTerminate(thread);
        chThdWait(thread);
    }

    void execute(const buffer_c8_t& buffer) override {
        if (state != State::Settling && state != State::Capturing) return;

        const auto in = reinterpret_cast<const lcs::Sample*>(buffer.p);
        const int count = buffer.count / 2;  // after decimation

        if (state == State::Settling) {
            if (--settle > 0) return;
            // the last settling buffer primes the filter
            decimator.process(in, buffer.count, capture);
            missed_before = shared_memory.m4_buffer_missed;
            state = State::Capturing;
            return;
        }

        for (size_t i = 0; i < buffer.count; i += 8) {
            if (in[i].i >= 127 || in[i].i <= -128 || in[i].q >= 127 || in[i].q <= -128) trouble |= lcs_link::trouble_clipped;
        }
        if (shared_memory.m4_buffer_missed != missed_before) trouble |= lcs_link::trouble_dropped;
        if (position + count <= length) {
            decimator.process(in, buffer.count, &capture[position]);
            position += count;
        }
        if (position + count > length) state = State::Ready;
    }

    void on_message(const Message* const) override {}

   private:
    Thread* thread{nullptr};
    BasebandThread baseband_thread{lcs_link::adc_rate, this, baseband::Direction::Receive, /*auto_start*/ false};
};

}  // namespace

int main() {
    EventDispatcher event_dispatcher{std::make_unique<CellScoutProcessor>()};
    event_dispatcher.run();
    return 0;
}
