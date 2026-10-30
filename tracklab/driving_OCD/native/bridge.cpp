// Tracklab adapter. The vehicle, tire, steering, drivetrain and integration
// equations below are instantiated from unmodified, pinned TUM OCD sources.
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ocd_aerodynamics_models_cpp/default.hpp"
#include "ocd_drivetrain_wheel_torque_cpp/drivetrain_direct_torque_model.hpp"
#include "ocd_steering_actuator_pt1_cpp/steering_actuator_pt1_model.hpp"
#include "ocd_tire_models_cpp/linear.hpp"
#include "ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_model.hpp"
#include "ocd_vehicle_model_cpp/vehicle_model.hpp"

#ifdef _WIN32
#define OCD_EXPORT extern "C" __declspec(dllexport)
#else
#define OCD_EXPORT extern "C" __attribute__((visibility("default")))
#endif

namespace {
using VD = tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel<
  tam::ocd::tire_models::Linear, tam::ocd::aerodynamics::DefaultAerodynamicsModel>;
using DT = tam::ocd::drivetrain::DrivetrainWheelTorqueModel;
using SA = tam::ocd::steering_actuator::PT1SteeringActuatorModel;
using Vehicle = tam::ocd::VehicleModel<DT, SA, VD>;
using Wheels = tam::types::common::DataPerWheel<double>;
thread_local std::string error;

struct Handle {
  Vehicle model;
  Vehicle evaluator;
  std::vector<std::string> parameter_names;
  Handle() {
    auto names = model.get_param_manager()->list_parameters();
    parameter_names.assign(names.begin(), names.end());
    std::sort(parameter_names.begin(), parameter_names.end());
  }
};
Wheels wheels(const double * p) { return {p[0], p[1], p[2], p[3]}; }
void copy_wheels(const Wheels & p, double * out) {
  out[0] = p.front_left; out[1] = p.front_right;
  out[2] = p.rear_left; out[3] = p.rear_right;
}
void inputs(Vehicle & v, const double * torque, double steering, const double * friction) {
  DT::DriverInputType d{};
  d.drivetrain_input_torque_per_wheel_Nm = wheels(torque);
  v.set_drivetrain_input(d);
  SA::DriverInputType s{};
  s.steering_angle_rad = steering;
  v.set_steering_input(s);
  tam::ocd::types::ExternalInfluences e{};
  e.lambda_mue = wheels(friction);
  v.set_external_influences(e);
}
// An upstream step of duration zero keeps the continuous state unchanged while
// refreshing its public output and cached wheel speeds. This is an explicit
// adapter boundary: physical substeps still call upstream VehicleModel::step.
void synchronize(Vehicle & v) {
  auto p = v.get_param_manager();
  double old_h = p->get_value("integration_step_size_s").as_double();
  p->set_value("integration_step_size_s", 0.0);
  v.step();
  p->set_value("integration_step_size_s", old_h);
}
void reset(Vehicle & v, const double * state) {
  auto p = v.get_param_manager();
  for (size_t i = 0; i < 6; ++i) {
    p->set_value("initial_state.vehicle_dynamics." + std::string(VD::StateNamesTrait::value[i]), state[i]);
  }
  for (size_t i = 0; i < 4; ++i) {
    p->set_value("initial_state.drivetrain." + std::string(DT::StateNamesTrait::value[i]), state[6 + i]);
  }
  p->set_value("initial_state.steering_actuator.position_rad", state[10]);
  v.reset();
}
void state(const Vehicle & v, double * out) {
  auto o = v.get_vehicle_model_output();
  auto & b = o.vehicle_dynamics_output;
  out[0] = b.position_m.x; out[1] = b.position_m.y;
  out[2] = b.orientation_rad.z; out[3] = b.velocity_mps.x;
  out[4] = b.velocity_mps.y; out[5] = b.angular_velocity_radps.z;
  copy_wheels(o.wheel_speeds_radps, out + 6);
  out[10] = v.get_steering_actuator_feedback().steering_angle_rad;
}
double logged_double(const tam::tsl::data_map_t & values, const std::string & name) {
  return std::get<double>(values.at(name));
}
void derivative(const Vehicle & v, double * out) {
  auto & values = v.get_logger()->get_data();
  for (size_t i = 0; i < 6; ++i) {
    out[i] = logged_double(values, "vehicle_dynamics/x_dot_vec/" + std::string(VD::StateNamesTrait::value[i]));
  }
  for (size_t i = 0; i < 4; ++i) {
    out[6 + i] = logged_double(values, "drivetrain/x_dot_vec/" + std::string(DT::StateNamesTrait::value[i]));
  }
  out[10] = logged_double(values, "steering_actuator/steering_angle_rate_radps");
}
void diagnostics(const Vehicle & v, double * out) {
  auto o = v.get_vehicle_model_output();
  auto & b = o.vehicle_dynamics_output;
  out[0] = b.acceleration_mps2.x; out[1] = b.acceleration_mps2.y;
  out[2] = b.acceleration_mps2.z; out[3] = b.angular_acceleration_radps2.z;
  copy_wheels(b.tire_slip_angle_per_wheel_rad, out + 4);
  copy_wheels(b.tire_longitudinal_slip_per_wheel, out + 8);
  copy_wheels(b.longitudinal_tire_force_tire_frame_per_wheel_N, out + 12);
  copy_wheels(b.lateral_tire_force_tire_frame_per_wheel_N, out + 16);
  copy_wheels(b.vertical_tire_force_per_wheel_N, out + 20);
  copy_wheels(o.drivetrain_load_torque_per_wheel_Nm, out + 24);
  copy_wheels(o.steering_angle_per_wheel_rad, out + 28);
}
template<class F> int guarded(F && f) {
  try { f(); error.clear(); return 0; }
  catch (const std::exception & e) { error = e.what(); return -1; }
  catch (...) { error = "Unknown C++ exception in the TUM OCD adapter"; return -1; }
}
} // namespace

OCD_EXPORT const char * ocd_error() { return error.c_str(); }
OCD_EXPORT const char * ocd_identity() {
  return "94f8fb187fb0ed22bba1d809bd74f66d1ff75af4:WHEEL_TORQUE:PT1:SINGLE_TRACK:LINEAR:DEFAULT";
}
OCD_EXPORT void * ocd_create() {
  try { error.clear(); return new Handle(); }
  catch (const std::exception & e) { error = e.what(); return nullptr; }
}
OCD_EXPORT void ocd_destroy(void * ptr) { delete static_cast<Handle *>(ptr); }
OCD_EXPORT int ocd_parameter_count(void * ptr) { return static_cast<Handle *>(ptr)->parameter_names.size(); }
OCD_EXPORT const char * ocd_parameter_name(void * ptr, int index) {
  auto & names = static_cast<Handle *>(ptr)->parameter_names;
  if (index < 0 || static_cast<size_t>(index) >= names.size()) return nullptr;
  return names[index].c_str();
}
OCD_EXPORT int ocd_parameter_get(void * ptr, const char * name, double * out, int capacity, int * count) {
  return guarded([&] {
    auto value = static_cast<Handle *>(ptr)->model.get_param_manager()->get_value(name);
    std::vector<double> values;
    if (value.get_type() == tam::pmg::ParameterType::DOUBLE) values = {value.as_double()};
    else values = value.as_double_array();
    *count = values.size();
    if (capacity < *count) throw std::runtime_error("Parameter buffer too small");
    std::copy(values.begin(), values.end(), out);
  });
}
OCD_EXPORT int ocd_parameter_set(void * ptr, const char * name, const double * values, int count, int array) {
  return guarded([&] {
    auto & h = *static_cast<Handle *>(ptr);
    tam::pmg::param_value_variant_t value = array ? tam::pmg::param_value_variant_t(std::vector<double>(values, values + count)) : tam::pmg::param_value_variant_t(values[0]);
    h.model.get_param_manager()->set_value(name, value);
    h.evaluator.get_param_manager()->set_value(name, value);
  });
}
OCD_EXPORT int ocd_reset(void * ptr, const double * x, const double * friction, double * out) {
  return guarded([&] {
    auto & v = static_cast<Handle *>(ptr)->model;
    reset(v, x);
    double zero[4] = {0, 0, 0, 0};
    inputs(v, zero, x[10], friction);
    synchronize(v);
    state(v, out);
  });
}
OCD_EXPORT int ocd_step(void * ptr, const double * torque, double steering, double dt, int substeps, const double * friction, double * out) {
  return guarded([&] {
    if (!std::isfinite(dt) || dt <= 0 || substeps < 1) throw std::invalid_argument("dt and substeps must be positive");
    auto & v = static_cast<Handle *>(ptr)->model;
    v.get_param_manager()->set_value("integration_step_size_s", dt / substeps);
    inputs(v, torque, steering, friction);
    for (int i = 0; i < substeps; ++i) v.step();
    synchronize(v);
    state(v, out);
  });
}
OCD_EXPORT int ocd_state(void * ptr, double * out) {
  return guarded([&] { state(static_cast<Handle *>(ptr)->model, out); });
}
OCD_EXPORT int ocd_diagnostics(void * ptr, double * out) {
  return guarded([&] { diagnostics(static_cast<Handle *>(ptr)->model, out); });
}
OCD_EXPORT int ocd_evaluate(void * ptr, const double * x, const double * torque, double steering, const double * friction, double * out, double * diag) {
  return guarded([&] {
    auto & v = static_cast<Handle *>(ptr)->evaluator;
    reset(v, x);
    inputs(v, torque, steering, friction);
    synchronize(v);
    derivative(v, out);
    diagnostics(v, diag);
  });
}
