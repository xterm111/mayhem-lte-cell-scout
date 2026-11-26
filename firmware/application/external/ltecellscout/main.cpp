/*
 * LTE Cell Scout: receive-only LTE cell measurement app.
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include "ui.hpp"
#include "ui_ltecellscout.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::ltecellscout {
void initialize_app(ui::NavigationView& nav) {
    nav.push<CellScoutView>();
}
}  // namespace ui::external_app::ltecellscout

extern "C" {

__attribute__((section(".external_app.app_ltecellscout.application_information"), used)) application_information_t _application_information_ltecellscout = {
    /*.memory_location = */ (uint8_t*)0x00000000,
    /*.externalAppEntry = */ ui::external_app::ltecellscout::initialize_app,
    /*.header_version = */ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,

    /*.app_name = */ "LTE Cell Scout",
    /*.bitmap_data = */
    {0x00, 0x00, 0x80, 0x01, 0x84, 0x21, 0x92, 0x49, 0xAA, 0x55, 0xAA, 0x55, 0x92, 0x49, 0x84, 0x21,
     0x80, 0x01, 0x80, 0x01, 0xC0, 0x03, 0x40, 0x02, 0x60, 0x06, 0x20, 0x04, 0x30, 0x0C, 0xF0, 0x0F},
    /*.icon_color = */ ui::Color::green().v,
    /*.menu_location = */ app_location_t::RX,
    /*.desired_menu_position = */ -1,

    /*.m4_app_tag = */ {'P', 'L', 'C', 'S'},
    /*.m4_app_offset = */ 0x00000000,  // filled at compile time
};
}
