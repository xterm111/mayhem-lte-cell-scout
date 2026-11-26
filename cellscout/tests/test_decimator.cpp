// The decimator against the plain definition of its filter, in uneven pieces.
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "ltecellscout_dsp.hpp"

int main() {
    std::srand(1);
    const int length = 200000;
    std::vector<lcs::Sample> in(length), out(length / 2);
    for (int n = 0; n < length; n++) {
        // mostly full scale values, to exercise the clamping and the packing
        auto value = [] { const int r = std::rand() % 4; return static_cast<int8_t>(r == 0 ? 127 : r == 1 ? -128 : std::rand() % 256 - 128); };
        in[n] = {value(), value()};
    }
    lcs::Decimator decimator;
    for (int done = 0; done < length;) {
        int take = 2 * (4 + std::rand() % 1500);
        if (take > length - done) take = length - done;
        decimator.process(&in[done], take, &out[done / 2]);
        done += take;
    }

    const int taps[7] = {-1, 0, 9, 16, 9, 0, -1};
    long wrong = 0;
    for (int n = 0; n < length; n += 2) {
        int i = 16, q = 16;
        for (int t = 0; t < 7; t++) {
            const int at = n - 5 + t;
            if (at < 0) continue;
            i += taps[t] * in[at].i;
            q += taps[t] * in[at].q;
        }
        i >>= 5;
        q >>= 5;
        i = i > 127 ? 127 : i < -128 ? -128 : i;
        q = q > 127 ? 127 : q < -128 ? -128 : q;
        wrong += out[n / 2].i != i || out[n / 2].q != q;
    }
    std::printf("decimator: %d outputs, %ld differ from the filter definition\n", length / 2, wrong);
    return wrong ? 1 : 0;
}
