// Cross-language core check: one complete numeric record per stdin line.
//   cpa: sx sy sz vx vy vz
//   wcv: sx sy sz vx vy vz D Z TTHR T
//   alert cpa|wcv D TTHR Z1 T1 Z2 T2 Z3 T3: sx sy sz vx vy vz
// Invalid input fails with exit 2; valid outputs use 17-digit doubles.
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "daa_core.hpp"

namespace {

int usage() {
    std::fprintf(stderr, "usage: daa_cli cpa | wcv | alert cpa|wcv D TTHR Z1 T1 Z2 T2 Z3 T3\n");
    return 2;
}

double number(const std::string& text) {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size()) throw std::invalid_argument("invalid numeric token: " + text);
    return value;  // Core validation rejects NaN/Inf and negative thresholds.
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string mode = argv[1];
    if (mode != "cpa" && mode != "wcv" && mode != "alert") return usage();
    if ((mode == "alert" && argc != 11) || (mode != "alert" && argc != 2)) return usage();
    try {
        daa::Strategy strategy = daa::Strategy::Wcv;
        std::array<daa::Tier, 3> tiers{};
        if (mode == "alert") {
            const std::string strat = argv[2];
            if (strat != "cpa" && strat != "wcv") return usage();
            strategy = strat == "cpa" ? daa::Strategy::Cpa : daa::Strategy::Wcv;
            const double D = number(argv[3]);
            const double tau = number(argv[4]);
            for (int i = 0; i < 3; ++i) {
                tiers[i] = daa::Tier{D, number(argv[5 + 2 * i]), tau, number(argv[6 + 2 * i])};
                daa::nonnegative(tiers[i].D); daa::nonnegative(tiers[i].Z);
                daa::nonnegative(tiers[i].TTHR); daa::nonnegative(tiers[i].T);
            }
        }
        std::string line;
        while (std::getline(std::cin, line)) {
            std::istringstream input(line);
            std::vector<double> values;
            std::string token;
            while (input >> token) values.push_back(number(token));
            if (values.empty()) continue;
            const std::size_t expected = mode == "wcv" ? 10 : 6;
            if (values.size() != expected) throw std::invalid_argument("wrong number of fields in input record");
            const daa::Vec3 s{values[0], values[1], values[2]};
            const daa::Vec3 v{values[3], values[4], values[5]};
            if (mode == "cpa") {
                const daa::Cpa c = daa::cpa(s, v);
                std::printf("%.17g %.17g %.17g\n", c.tcpa, c.hmd, c.vmd);
            } else if (mode == "wcv") {
                const auto r = daa::wcv_interval(s, v, values[6], values[7], values[8], values[9]);
                if (r) std::printf("1 %.17g %.17g\n", r->first, r->second);
                else std::printf("0 nan nan\n");
            } else {
                std::printf("%d\n", daa::alert_level(strategy, s, v, tiers));
            }
        }
        if (!std::cin.eof()) throw std::runtime_error("failed to read input stream");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "invalid input: %s\n", error.what());
        return 2;
    }
    return 0;
}
