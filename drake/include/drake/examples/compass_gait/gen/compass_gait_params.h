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
namespace examples {
namespace compass_gait {

/// Describes the row indices of a CompassGaitParams.
struct CompassGaitParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kMassHip = 0;
  static const int kMassLeg = 1;
  static const int kLengthLeg = 2;
  static const int kCenterOfMassLeg = 3;
  static const int kGravity = 4;
  static const int kSlope = 5;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `CompassGaitParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class CompassGaitParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef CompassGaitParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c mass_hip defaults to 10.0 kg.
  /// @arg @c mass_leg defaults to 5.0 kg.
  /// @arg @c length_leg defaults to 1.0 m.
  /// @arg @c center_of_mass_leg defaults to .5 m.
  /// @arg @c gravity defaults to 9.81 m/s^2.
  /// @arg @c slope defaults to 0.0525 radians.
  CompassGaitParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_mass_hip(10.0);
    this->set_mass_leg(5.0);
    this->set_length_leg(1.0);
    this->set_center_of_mass_leg(.5);
    this->set_gravity(9.81);
    this->set_slope(0.0525);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  CompassGaitParams(const CompassGaitParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  CompassGaitParams(CompassGaitParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  CompassGaitParams& operator=(const CompassGaitParams& other) {
    this->values() = other.values();
    return *this;
  }
  CompassGaitParams& operator=(CompassGaitParams&& other) noexcept {
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
    this->set_mass_hip(symbolic::Variable("mass_hip"));
    this->set_mass_leg(symbolic::Variable("mass_leg"));
    this->set_length_leg(symbolic::Variable("length_leg"));
    this->set_center_of_mass_leg(symbolic::Variable("center_of_mass_leg"));
    this->set_gravity(symbolic::Variable("gravity"));
    this->set_slope(symbolic::Variable("slope"));
  }

  CompassGaitParams<T>* DoClone() const final { return new CompassGaitParams; }

  /// @name Getters and Setters
  //@{
  /// Point mass at the hip.
  /// @note @c mass_hip is expressed in units of kg.
  /// @note @c mass_hip has a limited domain of [0.0, +Inf].
  const T& mass_hip() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMassHip);
  }
  /// Setter that matches mass_hip().
  void set_mass_hip(const T& mass_hip) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMassHip, mass_hip);
  }
  /// Fluent setter that matches mass_hip().
  /// Returns a copy of `this` with mass_hip set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  CompassGaitParams<T> with_mass_hip(const T& mass_hip) const {
    CompassGaitParams<T> result(*this);
    result.set_mass_hip(mass_hip);
    return result;
  }
  /// Mass of each leg (modeled as a point mass at the center of mass).
  /// @note @c mass_leg is expressed in units of kg.
  /// @note @c mass_leg has a limited domain of [0.0, +Inf].
  const T& mass_leg() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMassLeg);
  }
  /// Setter that matches mass_leg().
  void set_mass_leg(const T& mass_leg) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMassLeg, mass_leg);
  }
  /// Fluent setter that matches mass_leg().
  /// Returns a copy of `this` with mass_leg set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  CompassGaitParams<T> with_mass_leg(const T& mass_leg) const {
    CompassGaitParams<T> result(*this);
    result.set_mass_leg(mass_leg);
    return result;
  }
  /// The length of each leg.
  /// @note @c length_leg is expressed in units of m.
  /// @note @c length_leg has a limited domain of [0.0, +Inf].
  const T& length_leg() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLengthLeg);
  }
  /// Setter that matches length_leg().
  void set_length_leg(const T& length_leg) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLengthLeg, length_leg);
  }
  /// Fluent setter that matches length_leg().
  /// Returns a copy of `this` with length_leg set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  CompassGaitParams<T> with_length_leg(const T& length_leg) const {
    CompassGaitParams<T> result(*this);
    result.set_length_leg(length_leg);
    return result;
  }
  /// Distance from the hip to the center of mass of each leg.
  /// @note @c center_of_mass_leg is expressed in units of m.
  /// @note @c center_of_mass_leg has a limited domain of [0.0, +Inf].
  const T& center_of_mass_leg() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kCenterOfMassLeg);
  }
  /// Setter that matches center_of_mass_leg().
  void set_center_of_mass_leg(const T& center_of_mass_leg) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kCenterOfMassLeg, center_of_mass_leg);
  }
  /// Fluent setter that matches center_of_mass_leg().
  /// Returns a copy of `this` with center_of_mass_leg set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  CompassGaitParams<T> with_center_of_mass_leg(
      const T& center_of_mass_leg) const {
    CompassGaitParams<T> result(*this);
    result.set_center_of_mass_leg(center_of_mass_leg);
    return result;
  }
  /// An approximate value for gravitational acceleration.
  /// @note @c gravity is expressed in units of m/s^2.
  /// @note @c gravity has a limited domain of [0.0, +Inf].
  const T& gravity() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kGravity);
  }
  /// Setter that matches gravity().
  void set_gravity(const T& gravity) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kGravity, gravity);
  }
  /// Fluent setter that matches gravity().
  /// Returns a copy of `this` with gravity set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  CompassGaitParams<T> with_gravity(const T& gravity) const {
    CompassGaitParams<T> result(*this);
    result.set_gravity(gravity);
    return result;
  }
  /// The angle of the ramp on which the compass gait is walking.  Must have 0
  /// <= slope < PI/2 so that forward == downhill (an assumption used in the
  /// foot collision witness function).
  /// @note @c slope is expressed in units of radians.
  /// @note @c slope has a limited domain of [0.0, 1.5707].
  const T& slope() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kSlope);
  }
  /// Setter that matches slope().
  void set_slope(const T& slope) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSlope, slope);
  }
  /// Fluent setter that matches slope().
  /// Returns a copy of `this` with slope set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  CompassGaitParams<T> with_slope(const T& slope) const {
    CompassGaitParams<T> result(*this);
    result.set_slope(slope);
    return result;
  }
  //@}

  /// See CompassGaitParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return CompassGaitParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(mass_hip());
    result = result && (mass_hip() >= T(0.0));
    result = result && !isnan(mass_leg());
    result = result && (mass_leg() >= T(0.0));
    result = result && !isnan(length_leg());
    result = result && (length_leg() >= T(0.0));
    result = result && !isnan(center_of_mass_leg());
    result = result && (center_of_mass_leg() >= T(0.0));
    result = result && !isnan(gravity());
    result = result && (gravity() >= T(0.0));
    result = result && !isnan(slope());
    result = result && (slope() >= T(0.0));
    result = result && (slope() <= T(1.5707));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(7);
    (*value)[0] = mass_hip() - T(0.0);
    (*value)[1] = mass_leg() - T(0.0);
    (*value)[2] = length_leg() - T(0.0);
    (*value)[3] = center_of_mass_leg() - T(0.0);
    (*value)[4] = gravity() - T(0.0);
    (*value)[5] = slope() - T(0.0);
    (*value)[6] = T(1.5707) - slope();
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The CompassGaitParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace compass_gait
}  // namespace examples
}  // namespace drake

#undef DRAKE_VECTOR_GEN_NODISCARD
