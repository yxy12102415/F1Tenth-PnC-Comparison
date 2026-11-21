// Headless adapter to the same Gym-profile controllers used by takeover_sim.
// One process per trial. Each JSON line contains state [x,y,yaw,v,vy,r], the
// remaining route distance in meters and 25 [x,y,yaw,v] reference samples.
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <unistd.h>

#include "takeover_sim/mppi_control.hpp"

namespace {
constexpr double sample_period = .05;
constexpr unsigned reference_count = 25;

void check_row(const Json::Value &row, unsigned size) {
  if (!row.isArray() || row.size() != size)
    throw std::runtime_error("unexpected row size");
  for (const auto &v : row)
    if (!v.isNumeric() || !std::isfinite(v.asDouble()))
      throw std::runtime_error("expected a finite number");
}

// The v8 reference starts and ends at the same pose. Select a local length
// window using unwrapped lap progress so the endpoint coast cannot fire at
// startup. Use the original geometric path even when timed stop samples repeat.
frenet::Path geometric_window(const frenet::Assets &assets, double remaining) {
  const double progress = assets.arc.back() - remaining;
  const auto first = std::lower_bound(assets.arc.begin(), assets.arc.end(), progress - 1.);
  const auto last = std::upper_bound(first, assets.arc.end(), progress + 4.);
  return {assets.route.begin() + (first - assets.arc.begin()),
          assets.route.begin() + (last - assets.arc.begin())};
}

class FixedRouteController {
 public:
  FixedRouteController(const std::string &method, const std::string &map, double cap,
                       unsigned seed, bool vehicle_envelope)
      : assets_(map), cap_(cap), tracker_("dynamic_mpc") {
    const auto limits = vehicle_envelope ? frenet::joint_model::vehicle_limits : frenet::joint_model::default_limits;
    if (method == "nmpc") nmpc_.reset(new frenet::JointControl(
        true, cap, true, frenet::TimedDriveProfile::gym_fixed_route, limits));
    else if (method == "mppi") mppi_.reset(new takeover::MppiControl(cap, seed, limits));
    else throw std::runtime_error("expected nmpc or mppi");
  }

  Json::Value step(const Json::Value &request) {
    const auto &state = request["state"];
    check_row(state, 6);
    const auto &references = request["ref"];
    if (!references.isArray() || references.size() != reference_count)
      throw std::runtime_error("expected 25 reference stages at 20 Hz");
    frenet::TimedReference timed;
    for (const auto &p : references) {
      check_row(p, 4);
      timed.points.push_back({sample_period * timed.points.size(),
          {p[0].asDouble(), p[1].asDouble(), p[2].asDouble()}, p[3].asDouble()});
    }
    const auto &remaining_value = request["remaining_m"];
    if (!remaining_value.isNumeric()) throw std::runtime_error("remaining_m is required");
    double remaining = remaining_value.asDouble();
    if (!std::isfinite(remaining) || remaining < 0 || remaining > assets_.arc.back() + 1e-3)
      throw std::runtime_error("invalid unwrapped remaining distance");

    const frenet::Pose pose{state[0].asDouble(), state[1].asDouble(), state[2].asDouble()};
    const double v = state[3].asDouble(), vy = state[4].asDouble(), r = state[5].asDouble();
    const double start = frenet::mono();
    Json::Value reply;
    try {
      frenet::Control command;
      const frenet::Path *predicted;
      if (nmpc_) {
        auto geometry = geometric_window(assets_, remaining);
        if (request.isMember("local_geometry")) {
          geometry.clear();
          for (const auto &p : request["local_geometry"]) {
            check_row(p, 3);
            geometry.push_back({p[0].asDouble(), p[1].asDouble(), p[2].asDouble()});
          }
        }
        auto solve = [&]() {
          return nmpc_->compute(assets_, geometry, pose, v, vy, r, remaining, cap_,
                                sample_period, tracker_, {}, &timed);
        };
        try {
          command = solve();
        } catch (const std::runtime_error &failure) {
          if (std::string(failure.what()) != "joint_mpc_solver_4") throw;
          nmpc_->cold_retry(v, true);
          reply["cold_retry"] = true;
          command = solve();
        }
        predicted = &nmpc_->accepted_prediction();
      } else {
        command = mppi_->compute(assets_, timed, pose, v, vy, r, world_);
        predicted = &mppi_->accepted_prediction();
      }
      // Same obstacle-free map/body check as the simulator control node.
      for (const auto &p : *predicted) {
        for (const auto &corner : takeover::rectangle(p, world_.length, world_.width)) {
          if (assets_.clearance(corner) < world_.map_margin)
            throw std::runtime_error("predicted_footprint_map_conflict");
        }
      }
      // Static-study detections come from the scan; pass no scenario truth to the solver.
      for (const auto &disc : request["sensed_discs"]) {
        check_row(disc, 3);
        if (disc[2].asDouble() <= 0.) throw std::runtime_error("invalid_observed_radius");
        auto previous = pose;
        for (const auto &p : *predicted) {
          const double angle = frenet::wrap(p.yaw - previous.yaw);
          const double motion = (p.xy()-previous.xy()).norm() + .34*std::abs(angle);
          const int count = std::max(1, int(std::ceil(motion/.02)));
          for (int k=1; k<=count; ++k) {
            const double f = double(k)/count;
            const double dx = disc[0].asDouble() - (previous.x + f*(p.x-previous.x));
            const double dy = disc[1].asDouble() - (previous.y + f*(p.y-previous.y));
            const double yaw = previous.yaw + f*angle;
            const double lon = std::abs(dx*std::cos(yaw) + dy*std::sin(yaw)) - .58/2;
            const double lat = std::abs(dy*std::cos(yaw) - dx*std::sin(yaw)) - .31/2;
            if (std::hypot(std::max(lon,0.), std::max(lat,0.)) <= disc[2].asDouble() + .01)
              throw std::runtime_error("predicted_observed_disc_conflict");
          }
          previous = p;
        }
      }
      reply["speed"] = command.speed;
      reply["steer"] = command.angle;
      reply["ok"] = true;
    } catch (const std::exception &failure) {
      reply["error"] = failure.what();
      reply["ok"] = false;
    }
    reply["compute_ms"] = (frenet::mono() - start) * 1000.;
    return reply;
  }

 private:
  frenet::Assets assets_;
  double cap_;
  frenet::Tracker tracker_;
  std::unique_ptr<frenet::JointControl> nmpc_;
  std::unique_ptr<takeover::MppiControl> mppi_;
  takeover::MppiControl::World world_;
};
}  // namespace

int main(int argc, char **argv) {
  const bool vehicle_envelope = argc == 6 && std::string(argv[5]) == "--vehicle-envelope";
  if (argc != 5 && !vehicle_envelope) {
    std::cerr << "Usage: fitz_tracking_joint nmpc|mppi assets.json max_speed seed [--vehicle-envelope]\n";
    return 2;
  }
  // Keep native C solver diagnostics off the JSON response stream.
  FILE *responses = fdopen(dup(STDOUT_FILENO), "w");
  if (!responses || dup2(STDERR_FILENO, STDOUT_FILENO) < 0) return 2;
  try {
    FixedRouteController controller(argv[1], argv[2], std::stod(argv[3]), std::stoul(argv[4]), vehicle_envelope);
    std::string line;
    while (std::getline(std::cin, line)) {
      Json::Value request;
      std::istringstream input(line);
      input >> request;
      const Json::Value reply = controller.step(request);
      if (std::fprintf(responses, "%s\n", frenet::dump(reply).c_str()) < 0 ||
          std::fflush(responses) != 0) return 2;
      // A rejected command ends this trial. Never reuse its solver state.
      if (!reply["ok"].asBool()) return 1;
    }
  } catch (const std::exception &failure) {
    std::cerr << failure.what() << '\n';
    return 2;
  }
  return 0;
}
