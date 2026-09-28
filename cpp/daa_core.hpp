// daa_core.hpp -- CPA and well-clear-violation (WCV) core, C++17, header only.
//
// Mirrors the scalar functions in daa/geometry.py and daa/alerting.py line by
// line; tests/test_cpp_agreement.py checks both on the same inputs.
// Relative state: s = p_intruder - p_ownship, v = v_intruder - v_ownship (m, m/s, z up).
// WCV test follows the WCV_TAUMOD definition in NASA DAIDALUS (TCOA = 0).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace daa {

using Vec3 = std::array<double, 3>;
using Interval = std::pair<double, double>;

constexpr double kEps = 1e-12;  // m^2/s^2, horizontal relative speed^2 treated as zero
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Cpa {
    double tcpa;  // unclamped horizontal time of closest approach (s)
    double hmd;   // horizontal miss distance at max(tcpa, 0) (m)
    double vmd;   // vertical separation at max(tcpa, 0) (m, signed)
};

inline Cpa cpa(const Vec3& s, const Vec3& v) {
    const double a = v[0] * v[0] + v[1] * v[1];
    const double sv = s[0] * v[0] + s[1] * v[1];
    const double tcpa = a > kEps ? -sv / a : 0.0;
    const double t = tcpa > 0.0 ? tcpa : 0.0;
    const double hx = s[0] + t * v[0];
    const double hy = s[1] + t * v[1];
    return {tcpa, std::sqrt(hx * hx + hy * hy), s[2] + t * v[2]};
}

inline std::optional<Interval> vertical_interval(double sz, double vz, double Z) {
    if (std::fabs(vz) <= 1e-9) {
        if (std::fabs(sz) <= Z) return Interval{-kInf, kInf};
        return std::nullopt;
    }
    const double t1 = (-Z - sz) / vz;
    const double t2 = (Z - sz) / vz;
    return t1 <= t2 ? Interval{t1, t2} : Interval{t2, t1};
}

inline std::optional<Interval> horizontal_interval(double sx, double sy, double vx, double vy,
                                                   double D, double TTHR) {
    const double a = vx * vx + vy * vy;
    const double ss = sx * sx + sy * sy;
    const double D2 = D * D;
    if (a <= kEps) {
        if (ss <= D2) return Interval{-kInf, kInf};
        return std::nullopt;
    }
    const double sv = sx * vx + sy * vy;
    const double disc_c = sv * sv - a * (ss - D2);
    if (disc_c < 0.0) return std::nullopt;  // dcpa > DTHR
    const double t_out = (-sv + std::sqrt(disc_c)) / a;
    const double b = 2.0 * sv + TTHR * a;
    const double c = ss + TTHR * sv - D2;
    double disc_t = b * b - 4.0 * a * c;
    if (disc_t < 0.0) disc_t = 0.0;  // round-off guard
    const double t_in = (-b - std::sqrt(disc_t)) / (2.0 * a);
    return Interval{t_in, t_out};
}

// First/last time in [0, T] of predicted WCV; nullopt if none.
inline std::optional<Interval> wcv_interval(const Vec3& s, const Vec3& v, double D, double Z,
                                            double TTHR, double T) {
    const auto h = horizontal_interval(s[0], s[1], v[0], v[1], D, TTHR);
    if (!h) return std::nullopt;
    const auto vi = vertical_interval(s[2], v[2], Z);
    if (!vi) return std::nullopt;
    const double lo = std::max({h->first, vi->first, 0.0});
    const double hi = std::min({h->second, vi->second, T});
    if (lo > hi) return std::nullopt;
    return Interval{lo, hi};
}

inline bool cpa_condition(const Vec3& s, const Vec3& v, double Dh, double Dv, double T) {
    const Cpa c = cpa(s, v);
    const bool inside = (s[0] * s[0] + s[1] * s[1] <= Dh * Dh) && std::fabs(s[2]) <= Dv;
    return inside || (0.0 < c.tcpa && c.tcpa <= T && c.hmd <= Dh && std::fabs(c.vmd) <= Dv);
}

struct Tier {
    double D;     // horizontal threshold (m)
    double Z;     // vertical threshold (m)
    double TTHR;  // modified-tau threshold (s); ignored by the CPA strategy
    double T;     // alerting time / time-to-CPA threshold (s)
};

enum class Strategy { Cpa, Wcv };

// Highest tier (1-based) whose condition holds; 0 = no alert.
template <std::size_t N>
int alert_level(Strategy st, const Vec3& s, const Vec3& v, const std::array<Tier, N>& tiers) {
    int level = 0;
    for (std::size_t i = 0; i < N; ++i) {
        const Tier& t = tiers[i];
        const bool hit = st == Strategy::Cpa ? cpa_condition(s, v, t.D, t.Z, t.T)
                                             : wcv_interval(s, v, t.D, t.Z, t.TTHR, t.T).has_value();
        if (hit) level = static_cast<int>(i) + 1;
    }
    return level;
}

}  // namespace daa
