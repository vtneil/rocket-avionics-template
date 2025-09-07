#ifndef MINI_FC_FIRMWARE_KALMAN_H
#define MINI_FC_FIRMWARE_KALMAN_H

#include <lib_xcore>
#include <xcore/math_module>

// Kalman Filters
constexpr size_t FILTER_ORDER = 3;
constexpr double BASE_NOISE   = 0.1;

constexpr xcore::numeric_matrix<FILTER_ORDER, 1>            B  = xcore::make_numeric_matrix<FILTER_ORDER, 1>();
constexpr xcore::numeric_matrix<1, FILTER_ORDER>            H  = xcore::make_numeric_matrix<1, FILTER_ORDER>({
  {
   1,
   }
});
constexpr xcore::numeric_vector<FILTER_ORDER>               x0 = xcore::make_numeric_vector<FILTER_ORDER>();
constexpr xcore::numeric_matrix<FILTER_ORDER, FILTER_ORDER> P0 = xcore::numeric_matrix<FILTER_ORDER, FILTER_ORDER>::diagonals(1000.);

struct FilterT {
  xcore::numeric_matrix<FILTER_ORDER, FILTER_ORDER> F = xcore::make_numeric_matrix<FILTER_ORDER, FILTER_ORDER>();
  xcore::numeric_matrix<FILTER_ORDER, FILTER_ORDER> Q = xcore::numeric_matrix<FILTER_ORDER, FILTER_ORDER>::diagonals(BASE_NOISE);
  xcore::numeric_matrix<1, 1>                       R = xcore::numeric_matrix<1, 1>::diagonals(BASE_NOISE);

  xcore::r_iae_kalman_filter_t<FILTER_ORDER, 1, 1> kf{F, B, H, Q, R, x0, P0, 0.20, 0.05, 2.5, 1.e-12};
};

static xcore::vdt<FILTER_ORDER - 1> vdt{0.100};

#endif  //MINI_FC_FIRMWARE_KALMAN_H
