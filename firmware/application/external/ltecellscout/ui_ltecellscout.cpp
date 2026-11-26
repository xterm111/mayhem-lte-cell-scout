/*
 * LTE Cell Scout: user interface (application core).
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include "ui_ltecellscout.hpp"

#include "baseband_api.hpp"
#include "memory_map.hpp"
#include "portapack.hpp"
#include "portapack_shared_memory.hpp"

using namespace portapack;

namespace ui::external_app::ltecellscout {

namespace {

lcs_link::Link& link() {
    return *reinterpret_cast<lcs_link::Link*>(&shared_memory.bb_data.data[0]);
}

inline void barrier() {
    __asm__ volatile("dmb" ::: "memory");
}

/* --- text ---------------------------------------------------------------- */

/* A screen line built in place. */
struct Line {
    char text[31];
    int length{0};

    void add(const char* s) {
        while (*s && length < 30) text[length++] = *s++;
    }
    /* right aligned in width columns, with decimals digits after the point */
    void number(int32_t value, int width, int decimals = 0) {
        char digits[14];
        int n = 0;
        uint32_t v = value < 0 ? -value : value;
        do {
            if (n == decimals && decimals) digits[n++] = '.';
            digits[n++] = '0' + v % 10;
            v /= 10;
        } while (v || (decimals && n <= decimals + 1));
        if (value < 0) digits[n++] = '-';
        for (int pad = width - n; pad > 0; pad--) add(" ");
        while (n && length < 30) text[length++] = digits[--n];
    }
    void column(int at) {
        while (length < at && length < 30) text[length++] = ' ';
    }
    void draw(Painter& painter, Point at, const Style& style) {
        column(30);
        painter.draw_string(at, style, std::string_view{text, static_cast<size_t>(length)});
    }
};

int32_t rounded(int32_t tenths) {
    return (tenths + (tenths < 0 ? -5 : 5)) / 10;
}

void bandwidth(Line& line, const Entry& entry) {
    if (!(entry.flags & lcs_link::flag_mib))
        line.add("  -");
    else if (entry.n_rb < 15)
        line.add("1.4");
    else
        line.number(entry.n_rb / 5, 3);
}

const TCHAR* tchar(const char16_t* s) {
    return reinterpret_cast<const TCHAR*>(s);
}

static constexpr char16_t preset_directory[] = u"CELLSCOUT";

/* The central 1.2 MHz of a spectrum answer as up to twelve 100 kHz values. */
void fold(const uint8_t* bins, uint8_t* out, int count) {
    for (int u = 0; u < 12 && u < count; u++) {
        const int from = 64 + (u - 6) * 20 / 3;
        int sum = 0;
        for (int k = 0; k < 6; k++) sum += bins[from + k];
        // the decimating filter takes off about 1 dB towards the edges
        const int edge = u < 6 ? u : 11 - u;
        const int value = sum / 6 + (edge < 2 ? 2 - edge : 0);
        out[u] = value > 255 ? 255 : value;
    }
}

}  // namespace

/* --- panel --------------------------------------------------------------- */

Panel::Panel(CellScoutView& view, Rect rect)
    : Widget{rect}, view_{view} {
    set_focusable(true);
}

void Panel::paint(Painter& painter) {
    view_.draw(painter, screen_rect().location());
}

bool Panel::on_encoder(const EncoderEvent delta) {
    view_.move_selection(delta);
    return true;
}

bool Panel::on_key(const KeyEvent key) {
    if (key != KeyEvent::Select) return false;
    view_.pick();
    return true;
}

/* --- view ---------------------------------------------------------------- */

CellScoutView::CellScoutView(NavigationView& nav)
    : nav_{nav} {
    add_children({&labels_, &field_lna_, &field_vga_, &field_amp_, &field_calibration_,
                  &button_preset_, &options_band_, &button_search_, &field_earfcn_,
                  &button_monitor_, &panel_});

    field_calibration_.set_value(calibration_);
    field_calibration_.on_change = [this](int32_t value) {
        calibration_ = value;
        panel_.set_dirty();
    };

    field_earfcn_.set_value(last_earfcn_);
    field_earfcn_.on_change = [this](int32_t value) {
        last_earfcn_ = value;
    };
    field_earfcn_.on_select = [this](NumberField& field) {
        coarse_ = !coarse_;
        field.set_step(coarse_ ? 100 : 1);
    };

    list_presets();
    button_preset_.on_select = [this](Button&) {
        // none, then each file in turn
        select_preset(file_ + 1 < file_count_ ? file_ + 1 : -1);
    };

    button_search_.on_select = [this](Button&) {
        if (job_ == Job::Sweep || job_ == Job::Probe)
            stop("Stopped");
        else
            start_search();
    };
    button_monitor_.on_select = [this](Button&) {
        if (job_ == Job::Monitor)
            stop("Stopped");
        else
            start_monitor();
    };

    // nothing may be left over from whatever used the shared memory before
    auto& shared = link();
    shared.command.seq = 0;
    shared.response.seq = 0;
    barrier();
    baseband::run_prepared_image(memory::map::m4_code.base());

    receiver_model.set_sampling_rate(lcs_link::adc_rate);
    receiver_model.set_baseband_bandwidth(lcs_link::filter_bandwidth);
    receiver_model.enable();
}

CellScoutView::~CellScoutView() {
    receiver_model.disable();
    baseband::shutdown();
}

void CellScoutView::focus() {
    button_search_.focus();
}

/* --- presets ------------------------------------------------------------- */

void CellScoutView::list_presets() {
    DIR& dir = storage_.listing.dir;
    FILINFO& info = storage_.listing.info;
    if (f_opendir(&dir, tchar(preset_directory)) != FR_OK) return;
    while (file_count_ < max_files && f_readdir(&dir, &info) == FR_OK && info.fname[0]) {
        int length = 0;
        while (info.fname[length]) length++;
        if ((info.fattrib & AM_DIR) || length < 5 || length >= file_name_size) continue;
        const auto end = &info.fname[length - 4];
        if (end[0] != '.' || (end[1] | 32) != 't' || (end[2] | 32) != 'x' || (end[3] | 32) != 't') continue;
        for (int i = 0; i <= length; i++) files_[file_count_][i] = info.fname[i];
        file_count_++;
    }
    f_closedir(&dir);
}

void CellScoutView::select_preset(int file) {
    file_ = file;
    char label[13] = {};
    if (file < 0) {
        copy_text(label, "none", sizeof(label));
    } else {
        for (int i = 0; i < 12 && files_[file][i] && files_[file][i] != '.'; i++) label[i] = files_[file][i];
    }
    button_preset_.set_text(label);
    if (job_ != Job::Idle) stop(nullptr);
    load_preset(file);
}

void CellScoutView::load_preset(int file) {
    preset_ = {};
    message_ = "No preset: searching blind";
    panel_.set_dirty();
    if (file < 0) return;

    char16_t path[10 + file_name_size];
    for (int i = 0; i < 9; i++) path[i] = preset_directory[i];
    path[9] = '/';
    for (int i = 0; i < file_name_size; i++) path[10 + i] = files_[file][i];

    FIL& input = storage_.file;
    if (f_open(&input, tchar(path), FA_READ) != FR_OK) {
        message_ = "Cannot open preset";
        return;
    }

    char line[96];
    char chunk[64];
    size_t length = 0;
    UINT got = 0;
    do {
        if (f_read(&input, chunk, sizeof(chunk), &got) != FR_OK) break;
        // a short read ends the file, and with it the last line
        const size_t end = got < sizeof(chunk) ? got + 1 : got;
        for (size_t i = 0; i < end; i++) {
            const char c = i < got ? chunk[i] : '\n';
            if (c == '\n') {
                line[length] = 0;
                preset_.parse_line(line);
                length = 0;
            } else if (c != '\r' && length < sizeof(line) - 1) {
                line[length++] = c;
            }
        }
    } while (got == sizeof(chunk));
    f_close(&input);
    message_ = "Preset loaded";
}

/* --- search and monitor -------------------------------------------------- */

void CellScoutView::start_search() {
    const int choice = options_band_.selected_index();
    if (choice)
        band_mask_ = 1 << (choice - 1);
    else
        band_mask_ = preset_.sweep_mask ? preset_.sweep_mask : (1 << band_count) - 1;

    result_count_ = 0;
    selected_ = scroll_ = 0;
    monitoring_view_ = false;
    trouble_ = 0;
    band_ = -1;
    pending_ = false;
    button_search_.set_text("Stop");
    button_monitor_.set_text("Monitor");
    message_ = nullptr;
    next_band();
}

void CellScoutView::start_monitor() {
    const uint32_t earfcn = field_earfcn_.value();
    const int band = band_of(earfcn);
    if (band < 0) {
        stop("No European band has that");
        return;
    }
    earfcn_ = earfcn;
    channel_ = channel_of(band, earfcn);
    watched_count_ = 0;
    selected_ = scroll_ = 0;
    misses_ = 0;
    rssi_ = 0;
    trouble_ = 0;
    carrier_step_ = -1;
    for (auto& level : carrier_levels_) level = 0;
    monitoring_view_ = true;
    pending_ = false;
    job_ = Job::Monitor;
    button_search_.set_text("Search");
    button_monitor_.set_text("Stop");
    message_ = nullptr;
    panel_.set_dirty();
}

void CellScoutView::stop(const char* message) {
    job_ = Job::Idle;
    pending_ = false;
    button_search_.set_text("Search");
    button_monitor_.set_text("Monitor");
    message_ = message;
    panel_.set_dirty();
}

void CellScoutView::next_band() {
    while (++band_ < band_count) {
        if (band_mask_ & (1 << band_)) {
            job_ = Job::Sweep;
            step_ = 0;
            levels_band_ = band_;
            for (auto& level : levels_) level = 0;
            panel_.set_dirty();
            return;
        }
    }
    band_ = -1;
    stop(result_count_ ? "Done: select a cell" : "Done: nothing found");
}

void CellScoutView::send(lcs_link::Op op, uint32_t channel, int steps, int32_t center) {
    receiver_model.set_target_frequency(static_cast<int64_t>(channel) * 100'000);
    auto& command = link().command;
    command.op = op;
    command.cfo_steps = steps;
    command.cfo_center_hz = center;
    barrier();
    command.seq = ++sequence_;
    pending_ = true;
    waited_ = 0;
}

/* Hz of error to expect at a frequency. */
int32_t CellScoutView::expected_error(uint32_t channel) const {
    return error_known_ ? error_ppb_ / 100 * static_cast<int32_t>(channel) / 100 : 0;
}

/* One carrier: narrow once the receiver error is known. */
void CellScoutView::search_at(uint32_t channel) {
    const bool narrow = error_known_ && misses_ < 3;
    send(lcs_link::Op::Search, channel, narrow ? 1 : 7, expected_error(channel));
}

void CellScoutView::next_command() {
    if (job_ == Job::Monitor) {
        // one measurement, then the spectrum around the carrier, 1.2 MHz a step
        if (carrier_step_ < 0)
            search_at(channel_);
        else
            send(lcs_link::Op::Spectrum, channel_ - carrier_span / 2 + carrier_step_ * 12 + 6, 0, 0);
        return;
    }
    const Band& band = bands[band_];
    if (job_ == Job::Sweep) {
        send(lcs_link::Op::Spectrum, band.low + step_ * 12 + 6, 0, 0);
        return;
    }
    // Job::Probe
    if (listed_) {
        while (step_ < preset_.carrier_count) {
            const uint16_t earfcn = preset_.carriers[step_++].earfcn;
            if (earfcn < band.first || earfcn > band.last) continue;
            position_ = earfcn - band.first;
            misses_ = 0;
            search_at(band.low + position_);
            panel_.set_dirty();
            return;
        }
    } else {
        int position;
        if (walker_.next(levels_, band.last - band.first + 1, threshold_, position)) {
            position_ = position;
            // 27 steps cover the 200 kHz to either side
            send(lcs_link::Op::Search, band.low + position, 27, expected_error(band.low + position));
            panel_.set_dirty();
            return;
        }
    }
    next_band();
}

void CellScoutView::on_frame() {
    if (job_ == Job::Idle) return;
    if (!pending_) {
        next_command();
        return;
    }
    if (link().response.seq != sequence_) {
        if (++waited_ > 900) stop("No answer from baseband");  // 15 s
        return;
    }
    barrier();
    memcpy(&response_, const_cast<lcs_link::Response*>(&link().response), sizeof(response_));
    pending_ = false;

    if (job_ == Job::Monitor) {
        on_monitor();
    } else {
        trouble_ |= response_.trouble;
        if (job_ == Job::Sweep) {
            on_spectrum();
        } else {
            millis_ = response_.millis;
            on_search();
        }
    }
    // straight on: waiting for the next frame would double the time of a sweep
    if (job_ != Job::Idle && !pending_) next_command();
}

void CellScoutView::on_monitor() {
    if (carrier_step_ < 0) {
        cycle_trouble_ = response_.trouble;
        trouble_ |= response_.trouble;
        millis_ = response_.millis;
        watch(response_.cells, response_.cell_count);
        carrier_step_ = 0;
        panel_.set_dirty();
        return;
    }
    // clipping on a neighbouring frequency says nothing about the measurement
    cycle_trouble_ |= response_.trouble & lcs_link::trouble_dropped;
    fold(response_.spectrum, &carrier_levels_[carrier_step_ * 12], 12);
    if (++carrier_step_ * 12 < carrier_span) return;
    carrier_step_ = -1;
    trouble_ = cycle_trouble_;
    panel_.set_dirty();
}

void CellScoutView::on_spectrum() {
    const Band& band = bands[band_];
    const int width = band.last - band.first + 1;

    fold(response_.spectrum, &levels_[step_ * 12], width - step_ * 12);
    step_++;
    if (step_ % 8 == 0) panel_.set_dirty();
    if (step_ * 12 < width) return;

    // 4 dB above the quietest part counts as occupied
    uint8_t low = 255;
    for (int i = 0; i < width; i++)
        if (levels_[i] < low) low = levels_[i];
    threshold_ = low < 247 ? low + 8 : 255;

    job_ = Job::Probe;
    listed_ = false;
    for (int i = 0; i < preset_.carrier_count; i++)
        if (preset_.carriers[i].earfcn >= band.first && preset_.carriers[i].earfcn <= band.last) listed_ = true;
    step_ = 0;
    walker_.reset();
    panel_.set_dirty();
}

void CellScoutView::on_search() {
    const Band& band = bands[band_];
    const uint32_t channel = band.low + position_;

    for (int i = 0; i < response_.cell_count; i++) {
        const auto& cell = response_.cells[i];
        if (cell.flags & lcs_link::flag_tentative) continue;

        // blind: the carrier may be a few EARFCN away from where we tuned
        int32_t offset = 0;
        if (!listed_) {
            const int32_t delta = cell.cfo_hz - expected_error(channel);
            offset = (delta + (delta < 0 ? -50'000 : 50'000)) / 100'000;
        }
        const int position = position_ + offset;
        if (position < 0 || position > band.last - band.first) continue;

        if (cell.flags & lcs_link::flag_mib) {
            error_ppb_ = (cell.cfo_hz - offset * 100'000) * 10'000 / static_cast<int32_t>(channel);
            error_known_ = true;
        }
        add_result({static_cast<uint16_t>(band.first + position), cell.pci,
                    cell.rsrp_x10, cell.rsrq_x10, cell.sinr_x10,
                    cell.n_rb, cell.ports, cell.flags, 0, 1});

        if (!listed_) walker_.found(position, (cell.flags & lcs_link::flag_mib) ? cell.n_rb : 0);
    }
    panel_.set_dirty();
}

void CellScoutView::add_result(const Entry& entry) {
    for (int i = 0; i < result_count_; i++) {
        if (results_[i].earfcn == entry.earfcn && results_[i].pci == entry.pci) {
            if (entry.sinr > results_[i].sinr) results_[i] = entry;
            return;
        }
    }
    if (result_count_ < max_results) results_[result_count_++] = entry;
}

/* Merges one measurement into the monitored cells. */
void CellScoutView::watch(const lcs_link::Cell* cells, int count) {
    for (int i = 0; i < watched_count_; i++) watched_[i].age++;

    bool certain = false;
    for (int c = 0; c < count; c++) {
        const auto& cell = cells[c];
        const bool tentative = cell.flags & lcs_link::flag_tentative;
        if (!tentative) {
            certain = true;
            if (cell.flags & lcs_link::flag_mib) {
                error_ppb_ = cell.cfo_hz * 10'000 / static_cast<int32_t>(channel_);
                error_known_ = true;
            }
        }
        if (c == 0) rssi_ = cell.rssi_x10;

        int slot = 0;
        while (slot < watched_count_ && watched_[slot].pci != cell.pci) slot++;
        if (slot == watched_count_) {
            if (watched_count_ == max_watched) continue;
            watched_count_++;
            watched_[slot] = {earfcn_, cell.pci, cell.rsrp_x10, cell.rsrq_x10, cell.sinr_x10,
                              0, 0, lcs_link::flag_tentative, 0, 0};
        }
        Entry& entry = watched_[slot];
        // running average over about four measurements
        entry.rsrp += (cell.rsrp_x10 - entry.rsrp) / 4;
        entry.rsrq += (cell.rsrq_x10 - entry.rsrq) / 4;
        entry.sinr += (cell.sinr_x10 - entry.sinr) / 4;
        entry.age = 0;
        if (entry.hits < 255) entry.hits++;
        // once confirmed, or once the bandwidth is known, it stays that way
        const uint8_t kept = entry.flags & (lcs_link::flag_mib | lcs_link::flag_tentative);
        uint8_t flags = cell.flags;
        if (!(kept & lcs_link::flag_tentative)) flags &= ~lcs_link::flag_tentative;
        if (cell.flags & lcs_link::flag_mib) {
            entry.n_rb = cell.n_rb;
            entry.ports = cell.ports;
        }
        entry.flags = flags | (kept & lcs_link::flag_mib);
    }
    misses_ = certain ? 0 : (misses_ < 255 ? misses_ + 1 : misses_);

    // forget what has not been seen for a while
    int kept = 0;
    for (int i = 0; i < watched_count_; i++) {
        const bool weak = watched_[i].flags & lcs_link::flag_tentative && watched_[i].hits < 2;
        if (watched_[i].age < (weak ? 6 : 12)) watched_[kept++] = watched_[i];
    }
    watched_count_ = kept;
}

/* --- screen -------------------------------------------------------------- */

int16_t CellScoutView::absolute(int16_t dbfs_x10) const {
    const int gain = receiver_model.lna() + receiver_model.vga() + (receiver_model.rf_amp() ? 14 : 0);
    return dbfs_x10 + 10 * (calibration_ - gain);
}

/* The monitor gives most of the screen to the spectrum. */
int CellScoutView::list_rows() const {
    return monitoring_view_ ? 4 : 8;
}

void CellScoutView::move_selection(int delta) {
    const int count = monitoring_view_ ? watched_count_ : result_count_;
    int next = selected_ + delta;
    if (next >= count) next = count - 1;
    if (next < 0) next = 0;
    selected_ = next;
    if (selected_ < scroll_) scroll_ = selected_;
    if (selected_ >= scroll_ + list_rows()) scroll_ = selected_ - list_rows() + 1;
    panel_.set_dirty();
}

/* Select in the result list: monitor that carrier. In the monitor: back to it. */
void CellScoutView::pick() {
    if (monitoring_view_) {
        stop("Search results");
        monitoring_view_ = false;
        selected_ = scroll_ = 0;
    } else if (selected_ < result_count_) {
        field_earfcn_.set_value(results_[selected_].earfcn);
        start_monitor();
    }
}

void CellScoutView::draw_row(Painter& painter, Point at, const Entry& entry, bool selected) {
    Line line;
    // not seen the last time: "~"; not confirmed yet: "?"
    const char* mark = (entry.flags & lcs_link::flag_tentative) ? "?" : entry.age ? "~"
                                                                                  : " ";
    if (monitoring_view_) {
        line.number(entry.pci, 3);
        line.add(mark);
        line.number(rounded(absolute(entry.rsrp)), 5);
        line.number(entry.rsrq, 6, 1);
        line.number(entry.sinr, 6, 1);
        bandwidth(line, entry);
        if (entry.flags & lcs_link::flag_mib) line.number(entry.ports, 2);
        line.column(26);
        if (entry.flags & lcs_link::flag_tdd) line.add("TDD");
    } else {
        line.number(entry.earfcn, 5);
        line.number(entry.pci, 4);
        bandwidth(line, entry);
        line.number(rounded(absolute(entry.rsrp)), 5);
        line.number(rounded(entry.sinr), 4);
        line.add(" ");
        const uint8_t mask = preset_.operators_at(entry.earfcn);
        int shown = 0;
        for (int i = 0; i < preset_.operator_count; i++) {
            if (!(mask & (1 << i))) continue;
            if (!shown++) line.add(preset_.operators[i].plmn);
        }
        // from the list, not read off the air
        line.add(shown > 1 ? "+?" : shown ? "?" : "-");
    }
    const Style& style = *Theme::getInstance()->bg_darkest;
    line.draw(painter, at, selected ? style.invert() : style);
}

void CellScoutView::draw(Painter& painter, Point at) {
    const Style& style = *Theme::getInstance()->bg_darkest;
    const Style& heading = *Theme::getInstance()->fg_light;

    Line status;
    Line info;
    const int shown_band = monitoring_view_ ? band_of(earfcn_) : band_;
    if (monitoring_view_) {
        status.add("EARFCN ");
        status.number(earfcn_, 0);
        status.add(" B");
        status.number(bands[shown_band].number, 0);
        status.add(" ");
        status.number(channel_, 0, 1);
        status.add("MHz");
        // the first operator of the list, "+" if there are more
        const uint8_t mask = preset_.operators_at(earfcn_);
        int shown = 0;
        for (int i = 0; i < preset_.operator_count; i++) {
            if (!(mask & (1 << i))) continue;
            if (!shown++) info.add(preset_.operators[i].name);
        }
        if (shown) info.add(shown > 1 ? "+?" : "?");
        info.column(11);
        info.add("RSSI");
        if (watched_count_) info.number(rounded(absolute(rssi_)), 5);
    } else if (shown_band >= 0) {
        status.add("B");
        status.number(bands[shown_band].number, 0);
        if (job_ == Job::Sweep) {
            status.add(" spectrum ");
            status.number(step_ * 1200 / (bands[shown_band].last - bands[shown_band].first + 1), 0);
            status.add("%");
        } else {
            status.add(listed_ ? " listed " : " looking ");
            status.number(bands[shown_band].first + position_, 0);
        }
    }
    if (!monitoring_view_) {
        info.add("Cells ");
        info.number(result_count_, 0);
    }
    if (message_) {
        status.length = 0;
        status.add(message_);
    }
    status.column(26);
    if (trouble_ & lcs_link::trouble_dropped)
        status.add("DROP");
    else if (trouble_ & lcs_link::trouble_clipped)
        status.add("CLIP");
    status.draw(painter, at, style);

    info.column(24);
    if (millis_) {
        info.number(millis_, 4);
        info.add("ms");
    }
    info.draw(painter, at + Point{0, 16}, style);

    Line title;
    // "*": power levels are relative until CAL is set
    title.add(monitoring_view_ ? "PCI  RSRP" : "EARFCN PCI BW RSRP");
    title.add(calibration_ ? " " : "*");
    title.add(monitoring_view_ ? " RSRQ  SINR BW P" : "SNR PLMN");
    title.draw(painter, at + Point{0, 32}, heading);

    const int count = monitoring_view_ ? watched_count_ : result_count_;
    int i = scroll_;
    for (int row = 0; row < list_rows(); row++, i++) {
        const Point where = at + Point{0, 48 + row * 16};
        // a weak detection is only shown once it has come back
        while (monitoring_view_ && i < count && (watched_[i].flags & lcs_link::flag_tentative) && watched_[i].hits < 2) i++;
        if (i < count) {
            const Entry& entry = monitoring_view_ ? watched_[i] : results_[i];
            draw_row(painter, where, entry, panel_.has_focus() && i == selected_);
        } else {
            Line blank;
            blank.draw(painter, where, panel_.has_focus() && i == 0 ? style.invert() : style);
        }
    }
    const Point below = at + Point{0, 48 + list_rows() * 16};
    if (monitoring_view_)
        draw_carrier(painter, below);
    else
        draw_spectrum(painter, below);
}

/* The 24 MHz around the monitored carrier, 100 kHz a column, 10 dB between the
 * grid lines, with the edges of the carrier once its bandwidth is known. */
void CellScoutView::draw_carrier(Painter& painter, Point at) {
    constexpr int height = 118;
    const Entry* known = nullptr;
    for (int i = 0; i < watched_count_; i++) {
        if ((watched_[i].flags & lcs_link::flag_mib) && (!known || watched_[i].rsrp > known->rsrp))
            known = &watched_[i];
    }
    // half the occupied width in columns: n_rb * 180 kHz / 2
    const int half = known ? known->n_rb * 9 / 10 : 0;

    int low = 255;
    for (int x = 0; x < carrier_span; x++)
        if (carrier_levels_[x] && carrier_levels_[x] < low) low = carrier_levels_[x];
    low -= 12;  // 6 dB of room under the quietest part

    int previous = -1;
    for (int x = 0; x < carrier_span; x++) {
        int level = carrier_levels_[x] - low;  // one pixel per half dB
        if (level < 1) level = 1;
        if (level > height - 2) level = height - 2;
        const int y = height - 1 - level;
        if (previous < 0) previous = y;
        const int top = y < previous ? y : previous;
        const int bottom = y < previous ? previous : y;
        previous = y;

        const int from_centre = x < carrier_span / 2 ? carrier_span / 2 - x : x - carrier_span / 2;
        const int column = at.x() + x;
        painter.fill_rectangle({column, at.y(), 1, height}, half && from_centre == half ? Color::yellow() : Color::black());
        for (int line = 17; line < height; line += 20)
            painter.fill_rectangle({column, at.y() + line, 1, 1}, Color::dark_grey());
        if (from_centre < half) painter.fill_rectangle({column, at.y(), 1, 1}, Color::yellow());
        painter.fill_rectangle({column, at.y() + top, 1, bottom - top + 1}, Color::green());
    }

    Line scale;
    scale.number(channel_ - carrier_span / 2, 0, 1);
    scale.column(11);
    if (known) {
        bandwidth(scale, *known);
        scale.add("MHz");
    }
    scale.column(23);
    scale.number(channel_ + carrier_span / 2, 7, 1);
    scale.draw(painter, at + Point{0, height}, *Theme::getInstance()->fg_light);
}

/* The last swept band, 70 px high, with its frequency limits. */
void CellScoutView::draw_spectrum(Painter& painter, Point at) {
    if (levels_band_ < 0) {
        painter.fill_rectangle({at.x(), at.y(), 240, 70}, Color::black());
        return;
    }
    const Band& band = bands[levels_band_];
    const int width = band.last - band.first + 1;

    int low = 255;
    for (int i = 0; i < width; i++)
        if (levels_[i] && levels_[i] < low) low = levels_[i];
    constexpr int height = 54;
    for (int x = 0; x < 240; x++) {
        int level = 0;
        for (int i = x * width / 240; i <= (x + 1) * width / 240 && i < width; i++)
            if (levels_[i] > level) level = levels_[i];
        int bar = (level - low) / 2;  // one pixel per dB
        if (bar < 1) bar = 1;
        if (bar >= height) bar = height - 1;
        painter.fill_rectangle({at.x() + x, at.y(), 1, height - bar}, Color::black());
        painter.fill_rectangle({at.x() + x, at.y() + height - bar, 1, bar}, Color::green());
    }
    Line scale;
    scale.number(band.low, 0, 1);
    scale.column(12);
    scale.add("B");
    scale.number(band.number, 0);
    scale.column(23);
    scale.number(band.low + width, 7, 1);
    scale.draw(painter, at + Point{0, height}, *Theme::getInstance()->fg_light);
}

}  // namespace ui::external_app::ltecellscout
