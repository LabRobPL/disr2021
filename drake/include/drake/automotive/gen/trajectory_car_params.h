#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_bool.h"
#include "drake/common/dummy_value.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"

// TODO(jwnimmer-tri) Elevate this to drake/common.
#if __has_cpp_attribute(nodiscard)
#define DRAKE_VECTOR_GEN_NODISCARD [[nodiscard]]  // NOLINT(whitespace/braces)
#else
#define DRAKE_VECTOR_GEN_NODISCARD
#endif

namespace drake {
namespace automotive {

/// Describes the row indices of a TrajectoryCarParams.
struct TrajectoryCarParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kMaxSpeed = 0;
  static const int kSpeedLimitKp = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `TrajectoryCarParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class TrajectoryCarParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef TrajectoryCarParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c max_speed defaults to 45.0 m/s.
  /// @arg @c speed_limit_kp defaults to 10.0 Hz.
  TrajectoryCarParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_max_speed(45.0);
    this->set_speed_limit_kp(10.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  TrajectoryCarParams(const TrajectoryCarParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  TrajectoryCarParams(TrajectoryCarParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  TrajectoryCarParams& operator=(const TrajectoryCarParams& other) {
    this->values() = other.values();
    return *this;
  }
  TrajectoryCarParams& operator=(TrajectoryCarParams&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_max_speed(symbolic::Variable("max_speed"));
    this->set_speed_limit_kp(symbolic::Variable("speed_limit_kp"));
  }

  TrajectoryCarParams<T>* DoClone() const final {
    return new TrajectoryCarParams;
  }

  /// @name Getters and Setters
  //@{
  /// The limit on the car's forward speed.
  /// @note @c max_speed is expressed in units of m/s.
  /// @note @c max_speed has a limited domain of [0.0, +Inf].
  const T& max_speed() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMaxSpeed);
  }
  /// Setter that matches max_speed().
  void set_max_speed(const T& max_speed) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMaxSpeed, max_speed);
  }
  /// Fluent setter that matches max_speed().
  /// Returns a copy of `this` with max_speed set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  TrajectoryCarParams<T> with_max_speed(const T& max_speed) const {
    TrajectoryCarParams<T> result(*this);
    result.set_max_speed(max_speed);
    return result;
  }
  /// The smoothing constant for min/max speed limits.
  /// @note @c speed_limit_kp is expressed in units of Hz.
  /// @note @c speed_limit_kp has a limited domain of [0.0, +Inf].
  const T& speed_limit_kp() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kSpeedLimitKp);
  }
  /// Setter that matches speed_limit_kp().
  void set_speed_limit_kp(const T& speed_limit_kp) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSpeedLimitKp, speed_limit_kp);
  }
  /// Fluent setter that matches speed_limit_kp().
  /// Returns a copy of `this` with speed_limit_kp set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  TrajectoryCarParams<T> with_speed_limit_kp(const T& speed_limit_kp) const {
    TrajectoryCarParams<T> result(*this);
    result.set_speed_limit_kp(speed_limit_kp);
    return result;
  }
  //@}

  /// See TrajectoryCarParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return TrajectoryCarParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(max_speed());
    result = result && (max_speed() >= T(0.0));
    result = result && !isnan(speed_limit_kp());
    result = result && (speed_limit_kp() >= T(0.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(2);
    (*value)[0] = max_speed() - T(0.0);
    (*value)[1] = speed_limit_kp() - T(0.0);
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The TrajectoryCarParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace automotive
}  // namespace drake

#undef DRAKE_VECTOR_GEN_NODISCARD
