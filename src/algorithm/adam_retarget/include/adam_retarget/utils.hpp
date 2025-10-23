#pragma once

#include <casadi/casadi.hpp>

casadi::DM transform(double x, double y, double z, double qx, double qy, double qz, double qw) {
  casadi::DM T = casadi::DM::eye(4);
  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;

  auto x2 = qx * qx;
  auto y2 = qy * qy;
  auto z2 = qz * qz;
  auto w2 = qw * qw;

  auto xy = qx * qy;
  auto zw = qz * qw;
  auto xz = qx * qz;
  auto yw = qy * qw;
  auto yz = qy * qz;
  auto xw = qx * qw;

  T(0, 0) = x2 - y2 - z2 + w2;
  T(0, 1) = 2.0 * (xy - zw);
  T(0, 2) = 2.0 * (xz + yw);
  T(1, 0) = 2.0 * (xy + zw);
  T(1, 1) = -x2 + y2 - z2 + w2;
  T(1, 2) = 2.0 * (yz - xw);
  T(2, 0) = 2.0 * (xz - yw);
  T(2, 1) = 2.0 * (yz + xw);
  T(2, 2) = -x2 - y2 + z2 + w2;

  return T;
}