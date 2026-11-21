#pragma once
// CPU MPPI, independently implemented for the native Gym vehicle adapter.
// Algorithm references / license provenance: docs/FITZPATRICK_FIXED_TRACKING.md.
#include <random>
#include "joint_control.hpp"
#include "takeover_sim/dynamic_overtaking.hpp"

namespace takeover {

class MppiControl {
public:
  static constexpr int horizon=24;
  struct Input { double a=0., delta=0.; };
  struct State {
    Pose pose;
    double v=0., vy=0., r=0., command=0., servo=0., a=0., delta=0.;
  };
  using Sequence=std::array<Input,horizon>;
  struct World {
    std::vector<Prediction> lead;
    double length=.58,width=.31,lead_length=.58,lead_width=.31,map_margin=.05,body_margin=.10,sigma=2.;
    double lead_speed=0.;
  };
  int samples=256, iterations=2;
  double temperature=.5, sigma_a=.8, sigma_delta=.065;
  Json::Value diag;
  explicit MppiControl(double cap=3.,unsigned seed=0,
                       frenet::joint_model::CommandLimits limits=frenet::joint_model::default_limits)
      :cap_(cap),limits_(limits),rng_(seed) {
    limits_.validate();
    if(!std::isfinite(cap) || cap<.7 || cap>3.) throw std::runtime_error("mppi_cap_invalid");
  }
  double command_speed() const {return command_;}
  double command_acceleration() const {return acceleration_;}
  const Path &accepted_prediction() const {return prediction_;}
  void reset() {command_=acceleration_=delta_=servo_=0.;warm_=false;prediction_.clear();diag=Json::Value();}

  // Same backward-Euler dynamic bicycle and actuator model as native Gym MPC.
  // Public for analytic/model-agreement tests; dt is fixed to the 20 Hz clock.
  static State step(State x,Input &u,double cap,
                    frenet::joint_model::CommandLimits limits=frenet::joint_model::default_limits) {
    constexpr double h=.05;
    u.a=clip(u.a,std::max(-limits.deceleration,x.a-.15),
                std::min(limits.acceleration,x.a+.15));
    u.delta=clip(u.delta,std::max(-limits.steering,x.delta-.04),
                        std::min(limits.steering,x.delta+.04));
    u.a=clip(u.a,-x.command/h,(cap-x.command)/h);
    const double c=x.command+h*u.a;
    const double s=x.servo+h/(frenet::joint_model::steering_tau+h)*(u.delta-x.servo);
    const double v=std::max(.35,x.v),m=3.74,iz=.04712;
    const double lf=frenet::joint_model::lf,lr=frenet::joint_model::lr;
    const double cf=frenet::joint_model::cf,cr=frenet::joint_model::cr;
    const double a00=1+h*(cf+cr)/(m*v),a01=h*(v+(cf*lf-cr*lr)/(m*v));
    const double a10=h*(cf*lf-cr*lr)/(iz*v),a11=1+h*(cf*lf*lf+cr*lr*lr)/(iz*v);
    const double b0=x.vy+h*cf/m*s,b1=x.r+h*lf*cf/iz*s,det=a00*a11-a01*a10;
    double blend=clip((x.v-.25)/.30,0.,1.);blend=blend*blend*(3-2*blend);
    const double nv=x.v+h/(frenet::joint_model::longitudinal_tau(c,x.v)+h)*(c-x.v);
    const double vy=blend*(a11*b0-a01*b1)/det;
    const double r=blend*(a00*b1-a10*b0)/det+(1-blend)*nv/(lf+lr)*std::tan(s);
    const double yaw=x.pose.yaw+.5*h*r,avg=.5*(x.v+nv);
    x.pose.x+=h*(avg*std::cos(yaw)-vy*std::sin(yaw));
    x.pose.y+=h*(avg*std::sin(yaw)+vy*std::cos(yaw));x.pose.yaw+=h*r;
    x.v=nv;x.vy=vy;x.r=r;x.command=c;x.servo=s;x.a=u.a;x.delta=u.delta;
    return x;
  }

  frenet::Control compute(const Assets &assets,const frenet::TimedReference &timed,
      Pose pose,double speed,double lateral,double yaw_rate,const World &world) {
    const double begin=frenet::mono();
    if(samples<8 || samples>4096 || iterations<1 || iterations>6 || temperature<=0 ||
        sigma_a<=0 || sigma_delta<=0 || !std::isfinite(temperature+sigma_a+sigma_delta) ||
        !std::isfinite(speed+lateral+yaw_rate+pose.x+pose.y+pose.yaw) || speed<-.01 || speed>cap_+.4)
      throw std::runtime_error("mppi_configuration_or_state_invalid");
    timed.validate();timed.at(1.2);
    std::array<frenet::TimedReferencePoint,horizon+1> ref;
    for(int j=0;j<=horizon;++j) ref[j]=timed.at(j*.05);
    State initial{pose,std::max(0.,speed),lateral,yaw_rate,command_,servo_,acceleration_,delta_};
    // Receding sequence + a reference-derived proposal. This supplies a useful
    // initial mean, never an alternative controller that bypasses MPPI.
    Sequence nominal=sequence_, seed{};
    if(warm_) {for(int j=0;j<horizon-1;++j)nominal[j]=sequence_[j+1];}
    State x=initial;
    for(int j=0;j<horizon;++j) {
      const auto &r=ref[j];
      const double dx=x.pose.x-r.pose.x,dy=x.pose.y-r.pose.y;
      const double cross=-std::sin(r.pose.yaw)*dx+std::cos(r.pose.yaw)*dy;
      const double ds=std::max(.03,(ref[j+1].pose.xy()-r.pose.xy()).norm());
      const double k=wrap(ref[j+1].pose.yaw-r.pose.yaw)/ds;
      seed[j].delta=std::atan(frenet::joint_model::wheelbase*k)+
          1.2*wrap(r.pose.yaw-x.pose.yaw)-1.2*cross/std::max(.6,x.v);
      const double dv=(ref[j+1].speed-r.speed)/.05;
      const double desired=ref[j+1].speed+frenet::joint_model::longitudinal_tau(ref[j+1].speed,x.v)*dv;
      seed[j].a=3.*(desired-x.command)+2.*(r.speed-x.v);
      x=step(x,seed[j],cap_,limits_);
    }
    auto rollout=[&](Sequence &u,bool save) {
      State s=initial;double cost=0.;
      if(save){prediction_={pose};diag["predicted_speed_mps"]=Json::Value(Json::arrayValue);diag["predicted_speed_mps"].append(speed);}
      for(int j=0;j<horizon;++j) {
        const auto old=s;s=step(s,u[j],cap_,limits_);
        if(!safe(assets,s,j+1,world,initial.v)) return INF;
        const auto &r=ref[j+1];
        const double dx=s.pose.x-r.pose.x,dy=s.pose.y-r.pose.y;
        const double along=std::cos(r.pose.yaw)*dx+std::sin(r.pose.yaw)*dy;
        const double cross=-std::sin(r.pose.yaw)*dx+std::cos(r.pose.yaw)*dy;
        const double heading=wrap(s.pose.yaw-r.pose.yaw),ev=s.v-r.speed;
        const double weight=j==horizon-1?4.:1.;
        // Integrate running costs in seconds. Without dt, a temperature tuned
        // for integral cost collapses weights onto the unperturbed seed.
        cost+=.05*weight*(8*along*along+180*cross*cross+100*heading*heading+18*ev*ev+
            .1*u[j].a*u[j].a+1.*u[j].delta*u[j].delta+
            2*std::pow(u[j].a-old.a,2)+180*std::pow(u[j].delta-old.delta,2));
        if(save){prediction_.push_back(s.pose);diag["predicted_speed_mps"].append(s.v);}
      }
      return cost;
    };
    double base=rollout(nominal,false),sc=rollout(seed,false);
    if(!warm_ || sc<base)nominal=seed;
    std::normal_distribution<double> gaussian(0.,1.);
    std::vector<Sequence> sampled(samples);
    std::vector<double> costs(samples);
    int feasible=0,fallbacks=0;double ess=0.,update_norm=0.;
    for(int iteration=0;iteration<iterations;++iteration) {
      double minimum=INF;int best=-1;feasible=0;
      for(int k=0;k<samples;++k) {
        double correction=0.;
        for(int j=0;j<horizon;++j) {
          const double ea=k? sigma_a*gaussian(rng_):0.;
          const double ed=k? sigma_delta*gaussian(rng_):0.;
          sampled[k][j]={nominal[j].a+ea,nominal[j].delta+ed};
          correction+=.05*.015*(nominal[j].a*ea/(sigma_a*sigma_a)+
                            nominal[j].delta*ed/(sigma_delta*sigma_delta));
        }
        costs[k]=rollout(sampled[k],false)+correction;
        if(std::isfinite(costs[k])) {++feasible;if(costs[k]<minimum){minimum=costs[k];best=k;}}
      }
      if(best<0) throw std::runtime_error("mppi_no_feasible_rollout");
      double sum=0.,sum2=0.;Sequence updated{};
      for(int k=0;k<samples;++k) {
        const double w=std::isfinite(costs[k])?std::exp(-(costs[k]-minimum)/temperature):0.;
        sum+=w;sum2+=w*w;
        for(int j=0;j<horizon;++j){updated[j].a+=w*sampled[k][j].a;updated[j].delta+=w*sampled[k][j].delta;}
      }
      for(int j=0;j<horizon;++j) {
        updated[j].a/=sum;updated[j].delta/=sum;
        update_norm+=std::abs(updated[j].a-nominal[j].a)+std::abs(updated[j].delta-nominal[j].delta);
      }
      ess=sum*sum/sum2;
      // Nonconvex collision constraints are NOT preserved by averaging. Reject
      // an unsafe mean and use the already validated best sampled sequence.
      if(!std::isfinite(rollout(updated,false))) {updated=sampled[best];++fallbacks;}
      nominal=updated;
    }
    diag=Json::Value();
    if(!std::isfinite(rollout(nominal,true))) throw std::runtime_error("mppi_updated_rollout_invalid");
    Input first=nominal[0];const State next=step(initial,first,cap_,limits_);
    const double elapsed=(frenet::mono()-begin)*1000.;
    if(elapsed>45.)throw std::runtime_error("mppi_control_deadline");
    sequence_=nominal;warm_=true;command_=next.command;servo_=next.servo;
    acceleration_=first.a;delta_=first.delta;
    diag["controller_type"]="mppi";diag["controller_compute_ms"]=elapsed;
    diag["samples"]=samples;diag["iterations"]=iterations;diag["horizon_s"]=1.2;
    diag["temperature"]=temperature;diag["effective_sample_size"]=ess;
    diag["feasible_samples_last_iteration"]=feasible;diag["validated_sample_fallbacks"]=fallbacks;
    diag["weighted_update_l1"]=update_norm;
    diag["speed_command_mps"]=command_;diag["acceleration_command_mps2"]=acceleration_;
    diag["steering_command_rad"]=delta_;diag["timed_reference_age_s"]=timed.age;
    diag["timed_position_error_m"]=(pose.xy()-ref[0].pose.xy()).norm();
    diag["timed_speed_error_mps"]=speed-ref[0].speed;
    diag["configured_acceleration_limit_mps2"]=limits_.acceleration;
    diag["configured_deceleration_limit_mps2"]=limits_.deceleration;
    return {command_,delta_};
  }
private:
  double cap_,command_=0.,acceleration_=0.,delta_=0.,servo_=0.;
  frenet::joint_model::CommandLimits limits_;
  bool warm_=false;
  Sequence sequence_{};
  std::mt19937 rng_;
  Path prediction_;
  static bool safe(const Assets &a,const State &s,int j,const World &world,double initial_speed) {
    if(!std::isfinite(s.pose.x+s.pose.y+s.pose.yaw+s.v+s.vy+s.r) ||
        std::abs(s.v*s.r)>1.65 || std::abs(s.vy)>2.5 || std::abs(s.r)>8.)return false;
    const auto body=rectangle(s.pose,world.length,world.width);
    // Same map/body and temporal inflation policy as the existing adapter;
    // final accepted trajectory also passes the shared live validator.
    for(const auto &corner:body)if(a.clearance(corner)<world.map_margin)return false;
    if(a.clearance(s.pose.xy())<.25)return false;
    if(world.lead.empty())return true;
    const auto &lead=world.lead.at(j);
    const double inflation=world.body_margin+std::min(.30,world.sigma*lead.sigma)+
        .5*(initial_speed+world.lead_speed)*.05;
    const auto other=rectangle(lead.pose,world.lead_length+2*inflation,world.lead_width+2*inflation);
    return !polygons_overlap(body,other);
  }
};
} // namespace takeover
