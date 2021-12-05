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

/// Describes the row indices of a MaliputRailcarParams.
struct MaliputRailcarParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kR = 0;
  static const int kH = 1;
  static const int kMaxSpeed = 2;
  static const int kVelocityLimitKp = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `MaliputRailcarParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class MaliputRailcarParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef MaliputRailcarParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c r defaults to 0.0 m.
  /// @arg @c h defaults to 0.0 m.
  /// @arg @c max_speed defaults to 45.0 m/s.
  /// @arg @c velocity_limit_kp defaults to 10.0 Hz.
  MaliputRailcarParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_r(0.0);
    this->set_h(0.0);
    this->set_max_speed(45.0);
    this->set_velocity_limit_kp(10.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  MaliputRailcarParams(const MaliputRailcarParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  MaliputRailcarParams(MaliputRailcarParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  MaliputRailcarParams& operator=(const MaliputRailcarParams& other) {
    this->values() = other.values();
    return *this;
  }
  MaliputRailcarParams& operator=(MaliputRailcarParams&& other) noexcept {
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
    this->set_r(symbolic::Variable("r"));
    this->set_h(symbolic::Variable("h"));
    this->set_max_speed(symbolic::Variable("max_speed"));
    this->set_velocity_limit_kp(symbolic::Variable("velocity_limit_kp"));
  }

  MaliputRailcarParams<T>* DoClone() const final {
    return new MaliputRailcarParams;
  }

  /// @name Getters and Setters
  //@{
  /// The vehicle's position on the lane's r-axis.
  /// @note @c r is expressed in units of m.
  const T& r() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kR);
  }
  /// Setter that matches r().
  void set_r(const T& r) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kR, r);
  }
  /// Fluent setter that matches r().
  /// Returns a copy of `this` with r set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  MaliputRailcarParams<T> with_r(const T& r) const {
    MaliputRailcarParams<T> result(*this);
    result.set_r(r);
    return result;
  }
  /// The vehicle's height above the lane's surface.
  /// @note @c h is expressed in units of m.
  const T& h() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kH);
  }
  /// Setter that matches h().
  void set_h(const T& h) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kH, h);
  }
  /// Fluent setter that matches h().
  /// Returns a copy of `this` with h set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  MaliputRailcarParams<T> with_h(const T& h) const {
    MaliputRailcarParams<T> result(*this);
    result.set_h(h);
    return result;
  }
  /// The limit on the vehicle's forward speed, in meters per second; this
  /// element must be positive.
  /// @note @c max_speed is expressed in units of m/s.
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
  MaliputRailcarParams<T> with_max_speed(const T& max_speed) const {
    MaliputRailcarParams<T> result(*this);
    result.set_max_speed(max_speed);
    return result;
  }
  /// The smoothing constant for min/max velocity limits; this element must be
  /// positive.
  /// @note @c velocity_limit_kp is expressed in units of Hz.
  const T& velocity_limit_kp() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kVelocityLimitKp);
  }
  /// Setter that matches velocity_limit_kp().
  void set_velocity_limit_kp(const T& velocity_limit_kp) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kVelocityLimitKp, velocity_limit_kp);
  }
  /// Fluent setter that matches velocity_limit_kp().
  /// Returns a copy of `this` with velocity_limit_kp set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  MaliputRailcarParams<T> with_velocity_limit_kp(
      const T& velocity_limit_kp) const {
    MaliputRailcarParams<T> result(*this);
    result.set_velocity_limit_kp(velocity_limit_kp);
    return result;
  }
  //@}

  /// See MaliputRailcarParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return MaliputRailcarParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(r());
    result = result && !isnan(h());
    result = result && !isnan(max_speed());
    result = result && !isnan(velocity_limit_kp());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The MaliputRailcarParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace automotive
}  // namespace drake

#undef DRAKE_VECTOR_GEN_NODISCARD
