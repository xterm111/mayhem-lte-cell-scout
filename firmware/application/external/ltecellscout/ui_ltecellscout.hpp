/*
 * LTE Cell Scout: user interface (application core).
 *
 * "Search" sweeps the selected bands and fills the cell list, from the carriers
 * of a preset or, where there are none, by looking for them. "Monitor" measures
 * the cells of one EARFCN over and over.
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#ifndef __UI_LTECELLSCOUT_H__
#define __UI_LTECELLSCOUT_H__

#include "ui.hpp"
#include "ui_widget.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "app_settings.hpp"
#include "radio_state.hpp"
#include "message.hpp"
#include "ltecellscout_link.hpp"
#include "ff.h"
#include "ltecellscout_logic.hpp"

namespace ui::external_app::ltecellscout {

constexpr int max_results = 40;
constexpr int max_watched = 8;
constexpr int max_files = 6;
constexpr int file_name_size = 24;
constexpr int carrier_span = 240;  // 100 kHz values around a monitored carrier

/* In the app's own sections: a literal here would land in the firmware image. */
static constexpr char calibration_name[] = "calibration";
static constexpr char earfcn_name[] = "earfcn";

struct Entry {
    uint16_t earfcn;
    uint16_t pci;
    int16_t rsrp, rsrq, sinr;  // tenths of dB
    uint8_t n_rb, ports, flags;
    uint8_t age, hits;
};

class CellScoutView;

/* Everything below the controls: status, cell list and spectrum. */
class Panel : public Widget {
   public:
    Panel(CellScoutView& view, Rect rect);
    void paint(Painter& painter) override;
    bool on_encoder(const EncoderEvent delta) override;
    bool on_key(const KeyEvent key) override;

   private:
    CellScoutView& view_;
};

class CellScoutView : public View {
   public:
    CellScoutView(NavigationView& nav);
    ~CellScoutView();
    void focus() override;
    std::string title() const override { return "LTE Cell Scout"; };

   private:
    friend class Panel;

    enum class Job : uint8_t {
        Idle,
        Sweep,    // spectrum of a band
        Probe,    // searching carriers of a band
        Monitor,  // one carrier, repeatedly
    };

    void list_presets();
    void select_preset(int file);
    void load_preset(int file);

    void start_search();
    void start_monitor();
    void stop(const char* message);
    void next_band();
    void next_command();
    void send(lcs_link::Op op, uint32_t channel, int steps, int32_t center);
    void search_at(uint32_t channel);
    void on_frame();
    void on_spectrum();
    void on_search();
    void on_monitor();
    void add_result(const Entry& entry);
    void watch(const lcs_link::Cell* cells, int count);

    void draw(Painter& painter, Point at);
    void draw_row(Painter& painter, Point at, const Entry& entry, bool selected);
    void draw_spectrum(Painter& painter, Point at);
    void draw_carrier(Painter& painter, Point at);
    int list_rows() const;
    void move_selection(int delta);
    void pick();

    int32_t expected_error(uint32_t channel) const;
    int16_t absolute(int16_t dbfs_x10) const;

    NavigationView& nav_;
    RxRadioState radio_state_{ReceiverModel::Mode::SpectrumAnalysis};

    int32_t calibration_{0};
    uint32_t last_earfcn_{0};
    app_settings::SettingsManager settings_{
        "rx_ltecellscout",
        app_settings::Mode::RX,
        {{std::string_view{calibration_name}, &calibration_},
         {std::string_view{earfcn_name}, &last_earfcn_}}};

    Preset preset_{};
    char16_t files_[max_files][file_name_size]{};
    uint8_t file_count_{0};
    int8_t file_{-1};
    // FatFs directly: path and string helpers would grow the firmware image
    union {
        FIL file;
        struct {
            DIR dir;
            FILINFO info;
        } listing;
    } storage_{};

    Job job_{Job::Idle};
    bool pending_{false};
    uint32_t sequence_{0};
    uint16_t waited_{0};
    lcs_link::Response response_{};

    // search progress
    uint16_t band_mask_{0};
    int8_t band_{-1};
    int8_t levels_band_{-1};
    bool listed_{false};  // the preset names carriers in this band
    int16_t step_{0};
    Walker walker_{};
    int16_t position_{0};
    uint8_t threshold_{0};
    bool coarse_{false};  // the EARFCN field steps by 100
    uint8_t levels_[max_channels]{};

    // monitor
    uint16_t earfcn_{0};
    uint32_t channel_{0};  // frequency in 100 kHz units
    uint8_t misses_{0};
    int8_t carrier_step_{-1};  // spectrum step in progress, -1 while searching
    uint8_t cycle_trouble_{0};
    uint8_t carrier_levels_[carrier_span]{};

    // receiver frequency error in parts per thousand million
    int32_t error_ppb_{0};
    bool error_known_{false};

    Entry results_[max_results]{};
    Entry watched_[max_watched]{};
    uint8_t result_count_{0};
    uint8_t watched_count_{0};
    uint8_t selected_{0};
    uint8_t scroll_{0};
    bool monitoring_view_{false};
    uint8_t trouble_{0};
    int16_t rssi_{0};
    uint16_t millis_{0};
    const char* message_{"Choose preset and band"};

    Labels labels_{
        {{0, 0}, "LNA", Theme::getInstance()->fg_light->foreground},
        {{7 * 8, 0}, "VGA", Theme::getInstance()->fg_light->foreground},
        {{14 * 8, 0}, "AMP", Theme::getInstance()->fg_light->foreground},
        {{20 * 8, 0}, "CAL", Theme::getInstance()->fg_light->foreground},
        {{0, 16}, "List", Theme::getInstance()->fg_light->foreground},
        {{19 * 8, 16}, "Band", Theme::getInstance()->fg_light->foreground}};

    LNAGainField field_lna_{{4 * 8, 0}};
    VGAGainField field_vga_{{11 * 8, 0}};
    RFAmpField field_amp_{{18 * 8, 0}};
    NumberField field_calibration_{{24 * 8, 0}, 4, {-150, 150}, 1, ' '};
    Button button_preset_{{5 * 8, 16, 13 * 8, 16}, "none"};
    OptionsField options_band_{
        {24 * 8, 16},
        3,
        {{"All", 0}, {"B1", 1}, {"B3", 2}, {"B7", 3}, {"B8", 4}, {"B20", 5}, {"B28", 6}, {"B32", 7}, {"B38", 8}, {"B40", 9}}};

    Button button_search_{{0, 34, 9 * 8, 22}, "Search"};
    NumberField field_earfcn_{{12 * 8, 38}, 5, {0, 39649}, 1, ' '};
    Button button_monitor_{{20 * 8, 34, 10 * 8, 22}, "Monitor"};

    Panel panel_{*this, {0, 58, 240, 246}};

    MessageHandlerRegistration message_handler_frame_{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            this->on_frame();
        }};
};

}  // namespace ui::external_app::ltecellscout

#endif
