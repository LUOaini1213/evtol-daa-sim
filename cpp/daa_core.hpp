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
#include <stdexcept>
#include <utility>

namespace daa {

using Vec3 = std::array<double, 3>;
using Interval = std::pair<double, double>;

constexpr double kEps = 1e-12;  // m^2/s^2, horizontal relative speed^2 treated as zero
constexpr double kInf = std::numeric_limits<double>::infinity();

inline void finite(double value) {
    if (!std::isfinite(value)) throw std::invalid_argument("geometry values must be finite");
}

inline void state(const Vec3& value) {
    for (double component : value) finite(component);
}

inline void nonnegative(double value) {
    finite(value);
    if (value < 0.0) throw std::invalid_argument("geometry thresholds must be nonnegative");
}

struct Cpa {
    double tcpa;  // unclamped horizontal time of closest approach (s)
    double hmd;   // horizontal miss distance at max(tcpa, 0) (m)
    double vmd;   // vertical separation at max(tcpa, 0) (m, signed)
};

inline Cpa cpa(const Vec3& s, const Vec3& v) {
    state(s); state(v);
    const double a = v[0] * v[0] + v[1] * v[1];
    const double sv = s[0] * v[0] + s[1] * v[1];
    const double tcpa = a > kEps ? -sv / a : 0.0;
    const double t = tcpa > 0.0 ? tcpa : 0.0;
    const double hx = s[0] + t * v[0];
    const double hy = s[1] + t * v[1];
    const double hmd = std::sqrt(hx * hx + hy * hy);
    const double vmd = s[2] + t * v[2];
    finite(a); finite(sv); finite(tcpa); finite(hmd); finite(vmd);
    return {tcpa, hmd, vmd};
}

inline std::optional<Interval> vertical_interval(double sz, double vz, double Z) {
    finite(sz); finite(vz); nonnegative(Z);
    if (std::fabs(vz) <= 1e-9) {
        if (std::fabs(sz) <= Z) return Interval{-kInf, kInf};
        return std::nullopt;
    }
    const double t1 = (-Z - sz) / vz;
    const double t2 = (Z - sz) / vz;
    finite(t1); finite(t2);
    return t1 <= t2 ? Interval{t1, t2} : Interval{t2, t1};
}

inline std::optional<Interval> horizontal_interval(double sx, double sy, double vx, double vy,
                                                   double D, double TTHR) {
    finite(sx); finite(sy); finite(vx); finite(vy);
    nonnegative(D); nonnegative(TTHR);
    const double a = vx * vx + vy * vy;
    const double ss = sx * sx + sy * sy;
    const double D2 = D * D;
    finite(a); finite(ss); finite(D2);
    if (a <= kEps) {
        if (ss <= D2) return Interval{-kInf, kInf};
        return std::nullopt;
    }
    const double sv = sx * vx + sy * vy;
    // Cross-track distance avoids cancellation of along-track squares.
    const double speed = std::sqrt(a);
    const double miss = std::fabs(sx * (vy / speed) - sy * (vx / speed));
    finite(sv); finite(miss);
    if (miss > D) return std::nullopt;
    const double half_width2 = (D - miss) * (D + miss) / a;
    const double tcpa = -sv / a;
    const double half_tau = TTHR / 2.0;
    const double t_out = tcpa + std::sqrt(half_width2);
    const double t_in = tcpa - half_tau - std::sqrt(half_tau * half_tau + half_width2);
    finite(t_in); finite(t_out);
    return Interval{t_in, t_out};
}

// First/last time in [0, T] of predicted WCV; nullopt if none.
inline std::optional<Interval> wcv_interval(const Vec3& s, const Vec3& v, double D, double Z,
                                            double TTHR, double T) {
    state(s); state(v);
    nonnegative(D); nonnegative(Z); nonnegative(TTHR); nonnegative(T);
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
    nonnegative(Dh); nonnegative(Dv); nonnegative(T);
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
    if (st != Strategy::Cpa && st != Strategy::Wcv) throw std::invalid_argument("unknown alert strategy");
    state(s); state(v);
    for (const Tier& t : tiers) {
        nonnegative(t.D); nonnegative(t.Z); nonnegative(t.TTHR); nonnegative(t.T);
    }
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
