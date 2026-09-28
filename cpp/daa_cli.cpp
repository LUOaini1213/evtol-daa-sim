// daa_cli.cpp -- tiny command-line front end to daa_core.hpp for cross-checking.
//
//   daa_cli cpa                      stdin: sx sy sz vx vy vz          -> tcpa hmd vmd
//   daa_cli wcv                      stdin: sx sy sz vx vy vz D Z TTHR T -> hit lo hi
//   daa_cli alert cpa|wcv D TTHR Z1 T1 Z2 T2 Z3 T3
//                                    stdin: sx sy sz vx vy vz          -> level
// Numbers are printed with %.17g so doubles round-trip exactly.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "daa_core.hpp"

namespace {

int usage() {
    std::fprintf(stderr, "usage: daa_cli cpa | wcv | alert cpa|wcv D TTHR Z1 T1 Z2 T2 Z3 T3\n");
    return 2;
}

bool read_vec(daa::Vec3& s, daa::Vec3& v) {
    return std::scanf("%lf %lf %lf %lf %lf %lf", &s[0], &s[1], &s[2], &v[0], &v[1], &v[2]) == 6;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string mode = argv[1];
    daa::Vec3 s{}, v{};
    if (mode == "cpa") {
        while (read_vec(s, v)) {
            const daa::Cpa c = daa::cpa(s, v);
            std::printf("%.17g %.17g %.17g\n", c.tcpa, c.hmd, c.vmd);
        }
        return 0;
    }
    if (mode == "wcv") {
        double D, Z, TTHR, T;
        while (read_vec(s, v) && std::scanf("%lf %lf %lf %lf", &D, &Z, &TTHR, &T) == 4) {
            const auto r = daa::wcv_interval(s, v, D, Z, TTHR, T);
            if (r) std::printf("1 %.17g %.17g\n", r->first, r->second);
            else std::printf("0 nan nan\n");
        }
        return 0;
    }
    if (mode == "alert") {
        if (argc != 11) return usage();
        const std::string strat = argv[2];
        if (strat != "cpa" && strat != "wcv") return usage();
        const double D = std::atof(argv[3]);
        const double TTHR = std::atof(argv[4]);
        std::array<daa::Tier, 3> tiers{};
        for (int i = 0; i < 3; ++i) {
            tiers[i] = daa::Tier{D, std::atof(argv[5 + 2 * i]), TTHR, std::atof(argv[6 + 2 * i])};
        }
        const auto st = strat == "cpa" ? daa::Strategy::Cpa : daa::Strategy::Wcv;
        while (read_vec(s, v)) std::printf("%d\n", daa::alert_level(st, s, v, tiers));
        return 0;
    }
    return usage();
}
