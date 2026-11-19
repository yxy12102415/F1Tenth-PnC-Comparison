#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "core.hpp"

namespace takeover {

using frenet::Assets;
using frenet::INF;
using frenet::Path;
using frenet::Point;
using frenet::Pose;
using frenet::clip;
using frenet::wrap;

struct Prediction {
  double t = 0.0;
  Pose pose;
  double sigma = 0.0;
};

using Polygon = std::array<Point, 4>;

inline Polygon rectangle(const Pose &pose, double length, double width) {
  const Point longitudinal(std::cos(pose.yaw), std::sin(pose.yaw));
  const Point lateral(-std::sin(pose.yaw), std::cos(pose.yaw));
  const Point center(pose.x, pose.y);
  const Point a = 0.5 * length * longitudinal;
  const Point b = 0.5 * width * lateral;
  return {center + a + b, center + a - b, center - a - b, center - a + b};
}

inline bool overlap_on_axis(const Polygon &a, const Polygon &b, Point axis) {
  const double norm = axis.norm();
  if (norm < 1e-12) return true;
  axis /= norm;
  double amin = INF, amax = -INF, bmin = INF, bmax = -INF;
  for (const auto &p : a) { amin = std::min(amin, p.dot(axis)); amax = std::max(amax, p.dot(axis)); }
  for (const auto &p : b) { bmin = std::min(bmin, p.dot(axis)); bmax = std::max(bmax, p.dot(axis)); }
  return !(amax < bmin || bmax < amin);
}

inline bool polygons_overlap(const Polygon &a, const Polygon &b) {
  for (const Polygon *polygon : {&a, &b}) {
    for (size_t i = 0; i < polygon->size(); ++i) {
      const Point edge = (*polygon)[(i + 1) % polygon->size()] - (*polygon)[i];
      if (!overlap_on_axis(a, b, Point(-edge.y(), edge.x()))) return false;
    }
  }
  return true;
}

} // namespace takeover
