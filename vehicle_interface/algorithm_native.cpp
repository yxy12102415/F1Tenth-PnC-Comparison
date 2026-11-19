// stdin/stdout only. Replay shares v8 perception, NMPC and MPPI's physical
// parameter branch without linking ROS, actuator drivers or the Gym solver.
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unistd.h>
#include "perception.hpp"
#include "takeover_sim/mppi_control.hpp"

#ifdef TAKEOVER_OFFICIAL_GYM_MODEL
#error "physical shadow build requires the commissioned physical solver"
#endif
static_assert(frenet::joint_model::wheelbase == .25, "physical wheelbase required");
static_assert(frenet::joint_model::steering_limit == .269, "physical steering limits required");

namespace {
class Replay {
    frenet::Assets assets;
    frenet::Perception perception;
    frenet::Tracker tracker{"dynamic_mpc"};
    std::map<std::string, std::unique_ptr<frenet::JointControl>> solvers;
    std::vector<frenet::Point> points;
    takeover::MppiControl mppi;
    takeover::MppiControl::World world;
    // The VESC yaw rate is command-derived. Predict the unmeasured lateral
    // states using only the previous accepted (hypothetical) input.
    takeover::MppiControl::State observer{};
public:
    Replay(const char *file, unsigned seed) : assets(file), perception(assets), mppi(2., seed) {
        world.length = .46;
        world.width = .30;
    }
    Json::Value run(const Json::Value &req) {
        Json::Value out;
        const auto method = req["method"].asString();
        if (method == "ready") {
            out["wheelbase_m"] = frenet::joint_model::wheelbase;
            return out;
        }
        if (method == "perception") {
            const auto &p = req["scan_pose"];
            frenet::Pose pose{p[0].asDouble(), p[1].asDouble(), p[2].asDouble()};
            std::vector<float> ranges;
            for (const auto &v : req["ranges"])
                ranges.push_back(v.isNull() ? NAN : v.asFloat());
            points = perception.scan(ranges, req["angle_min"].asDouble(),
                    req["angle_increment"].asDouble(), req["range_min"].asDouble(),
                    req["range_max"].asDouble(), req["scan_stamp"].asDouble(), pose);
            out = perception.diagnostic();
            out["discs"] = Json::Value(Json::arrayValue);
            for (const auto &t : perception.tracks) {
                if (!t.confirmed) continue;
                // A scan sees the front surface. Bound all its beam endpoints
                // plus 10 cm of unseen depth. This assumption needs measurement.
                double radius = 0.;
                for (auto hit : t.cluster.points)
                    radius = std::max(radius, (hit - t.cluster.center).norm());
                Json::Value d(Json::arrayValue);
                d.append(t.cluster.center.x());
                d.append(t.cluster.center.y());
                d.append(radius + .10);
                out["discs"].append(d);
            }
            return out;
        }
        const auto &z = req["state"];
        frenet::Pose pose{z[0].asDouble(), z[1].asDouble(), z[2].asDouble()};
        const double speed = z[3].asDouble();
        auto path = frenet::path_read(req["path"]);
        frenet::TimedReference ref;
        for (const auto &p : req["ref"])
            ref.points.push_back({.05 * ref.points.size(),
                    {p[0].asDouble(), p[1].asDouble(), p[2].asDouble()}, p[3].asDouble()});
        frenet::Control cmd;
        if (method == "mppi") {
            cmd = mppi.compute(assets, ref, pose, speed, observer.vy, observer.r, world);
            out["prediction"] = frenet::path_json(mppi.accepted_prediction());
            observer.pose = pose;
            observer.v = speed;
            takeover::MppiControl::Input u{mppi.command_acceleration(), cmd.angle};
            observer = takeover::MppiControl::step(observer, u, 2.);
        } else {
            auto &solver = solvers[method];
            if (!solver) solver.reset(new frenet::JointControl(true));
            cmd = solver->compute(assets, path, pose, speed, z[4].asDouble(), z[5].asDouble(),
                                  req["remaining"].asDouble(), 2., .05, tracker, points, &ref);
            out["prediction"] = frenet::path_json(solver->accepted_prediction());
        }
        out["speed"] = cmd.speed;
        out["steering"] = cmd.angle;
        return out;
    }
};
}

int main(int argc, char **argv) {
    if (argc != 3) return 2;
    // acados may write numerical diagnostics to stdout. Keep JSONL distinct.
    FILE *responses = fdopen(dup(STDOUT_FILENO), "w");
    if (!responses || dup2(STDERR_FILENO, STDOUT_FILENO) < 0) return 2;
    try {
        Replay replay(argv[1], std::stoul(argv[2]));
        std::string line;
        while (std::getline(std::cin, line)) {
            Json::Value out;
            try {
                Json::Value request;
                std::istringstream input(line);
                input >> request;
                out = replay.run(request);
                out["ok"] = true;
            } catch (const std::exception &error) {
                out["ok"] = false;
                out["reason"] = error.what();
            }
            if (std::fprintf(responses, "%s\n", frenet::dump(out).c_str()) < 0 ||
                    std::fflush(responses) != 0) return 2;
        }
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
