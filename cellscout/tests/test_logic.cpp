// Preset parsing and the blind walk, on a PC.  test_logic UK.TXT IT.TXT
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "ltecellscout_logic.hpp"

using namespace ui::external_app::ltecellscout;

static int failures = 0;
#define EXPECT(condition, ...)               \
    do {                                     \
        if (!(condition)) {                  \
            failures++;                      \
            std::printf("FAIL " __VA_ARGS__); \
            std::printf("\n");               \
        }                                    \
    } while (0)

static Preset load(const char* path) {
    static Preset preset;
    preset = {};
    FILE* f = std::fopen(path, "r");
    EXPECT(f, "cannot open %s", path);
    if (!f) return preset;
    char line[96];
    int length = 0, c;
    do {  // truncating long lines, as the app does
        c = std::fgetc(f);
        if (c == '\n' || c == EOF) {
            line[length] = 0;
            preset.parse_line(line);
            length = 0;
        } else if (c != '\r' && length < static_cast<int>(sizeof(line)) - 1) {
            line[length++] = c;
        }
    } while (c != EOF);
    std::fclose(f);
    return preset;
}

static void check_preset(const char* path) {
    // count the records independently
    int ops = 0, carriers = 0, blocks = 0;
    FILE* f = std::fopen(path, "r");
    char text[256];
    while (f && std::fgets(text, sizeof(text), f)) {
        ops += !std::strncmp(text, "OP,", 3);
        carriers += !std::strncmp(text, "CARRIER,", 8);
        blocks += !std::strncmp(text, "BLOCK,", 6);
    }
    if (f) std::fclose(f);

    const Preset p = load(path);
    EXPECT(p.operator_count == ops, "%s: %d of %d operators", path, p.operator_count, ops);
    EXPECT(p.carrier_count == carriers, "%s: %d of %d carriers (limit %d)", path, p.carrier_count, carriers, max_carriers);
    EXPECT(p.block_count == blocks, "%s: %d of %d blocks (limit %d)", path, p.block_count, blocks, max_blocks);
    for (int i = 0; i < p.carrier_count; i++)
        EXPECT(band_of(p.carriers[i].earfcn) >= 0, "%s: EARFCN %d outside the bands", path, p.carriers[i].earfcn);
    for (int i = 0; i < p.block_count; i++)
        EXPECT(p.blocks[i].mask && p.blocks[i].low < p.blocks[i].high, "%s: bad block %d", path, i);
    std::printf("ok   %s: %d operators, %d carriers, %d blocks, bands", path, p.operator_count, p.carrier_count, p.block_count);
    for (int i = 0; i < band_count; i++)
        if (p.sweep_mask & (1 << i)) std::printf(" B%d", bands[i].number);
    std::printf("\n");
}

struct Carrier {
    int centre, n_rb;  // centre in 100 kHz from the band bottom
};

/* Walks a made up band. A look finds a carrier within 200 kHz of its centre. */
static void walk(const char* label, int width, const std::vector<Carrier>& carriers) {
    std::vector<uint8_t> levels(width, 20);
    for (const auto& c : carriers) {
        const int channel = c.n_rb == 6 ? 14 : c.n_rb == 15 ? 30 : c.n_rb * 2;
        const int half = channel * 9 / 20;  // 90 % of the channel is occupied
        for (int i = c.centre - half; i < c.centre + half; i++)
            if (i >= 0 && i < width) levels[i] = 60;
    }
    Walker walker;
    walker.reset();
    std::vector<bool> found(carriers.size(), false);
    int looks = 0, position;
    while (looks < 2000 && walker.next(levels.data(), width, 28, position)) {
        looks++;
        for (size_t i = 0; i < carriers.size(); i++) {
            if (position - carriers[i].centre >= -2 && position - carriers[i].centre <= 2) {
                found[i] = true;
                walker.found(carriers[i].centre, carriers[i].n_rb);
            }
        }
    }
    int count = 0;
    for (bool f : found) count += f;
    EXPECT(count == static_cast<int>(carriers.size()), "%s: found %d of %zu", label, count, carriers.size());
    EXPECT(looks < 2000, "%s: does not end", label);
    std::printf("info %-34s %d/%zu carriers in %d looks\n", label, count, carriers.size(), looks);
}

int main(int argc, char** argv) {
    for (int i = 1; i < argc; i++) check_preset(argv[i]);

    Preset p = load(argv[1]);
    // UK file: block 2110.3-2130.3 is shared by 234-15 and 234-20
    EXPECT(p.operators_at(100) != 0, "UK: EARFCN 100 should have operators");

    walk("empty band", 750, {});
    walk("one 20 MHz carrier", 750, {{300, 100}});
    walk("one 10 MHz carrier", 300, {{150, 50}});
    walk("one 1.4 MHz carrier", 350, {{101, 6}});
    walk("three 10 MHz, adjacent (B20)", 300, {{50, 50}, {150, 50}, {250, 50}});
    walk("20 + 15 + 20 + 20 adjacent (B3)", 750, {{101, 100}, {276, 75}, {451, 100}, {650, 100}});
    walk("5 + 5 + 10 adjacent", 350, {{25, 25}, {75, 25}, {150, 50}});
    walk("5 MHz then 20 MHz", 350, {{26, 25}, {151, 100}});
    walk("gaps and odd centres", 750, {{63, 50}, {218, 15}, {457, 100}, {702, 50}});
    walk("carrier at the top edge", 350, {{300, 50}});

    std::printf("logic failures: %d\n", failures);
    return failures ? 1 : 0;
}
