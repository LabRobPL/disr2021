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
namespace rimless_wheel {

/// Describes the row indices of a RimlessWheelParams.
struct RimlessWheelParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 5;

  // The index of each individual coordinate.
  static const int kMass = 0;
  static const int kLength = 1;
  static const int kGravity = 2;
  static const int kNumberOfSpokes = 3;
  static const int kSlope = 4;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `RimlessWheelParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class RimlessWheelParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef RimlessWheelParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c mass defaults to 1.0 kg.
  /// @arg @c length defaults to 1.0 m.
  /// @arg @c gravity defaults to 9.81 m/s^2.
  /// @arg @c number_of_spokes defaults to 8 integer.
  /// @arg @c slope defaults to 0.08 radians.
  RimlessWheelParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_mass(1.0);
    this->set_length(1.0);
    this->set_gravity(9.81);
    this->set_number_of_spokes(8);
    this->set_slope(0.08);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  RimlessWheelParams(const RimlessWheelParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  RimlessWheelParams(RimlessWheelParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  RimlessWheelParams& operator=(const RimlessWheelParams& other) {
    this->values() = other.values();
    return *this;
  }
  RimlessWheelParams& operator=(RimlessWheelParams&& other) noexcept {
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
    this->set_mass(symbolic::Variable("mass"));
    this->set_length(symbolic::Variable("length"));
    this->set_gravity(symbolic::Variable("gravity"));
    this->set_number_of_spokes(symbolic::Variable("number_of_spokes"));
    this->set_slope(symbolic::Variable("slope"));
  }

  RimlessWheelParams<T>* DoClone() const final {
    return new RimlessWheelParams;
  }

  /// @name Getters and Setters
  //@{
  /// The rimless wheel has a single point mass at the hub.
  /// @note @c mass is expressed in units of kg.
  /// @note @c mass has a limited domain of [0.0, +Inf].
  const T& mass() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMass);
  }
  /// Setter that matches mass().
  void set_mass(const T& mass) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMass, mass);
  }
  /// Fluent setter that matches mass().
  /// Returns a copy of `this` with mass set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  RimlessWheelParams<T> with_mass(const T& mass) const {
    RimlessWheelParams<T> result(*this);
    result.set_mass(mass);
    return result;
  }
  /// The length of each spoke.
  /// @note @c length is expressed in units of m.
  /// @note @c length has a limited domain of [0.0, +Inf].
  const T& length() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLength);
  }
  /// Setter that matches length().
  void set_length(const T& length) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLength, length);
  }
  /// Fluent setter that matches length().
  /// Returns a copy of `this` with length set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  RimlessWheelParams<T> with_length(const T& length) const {
    RimlessWheelParams<T> result(*this);
    result.set_length(length);
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
  RimlessWheelParams<T> with_gravity(const T& gravity) const {
    RimlessWheelParams<T> result(*this);
    result.set_gravity(gravity);
    return result;
  }
  /// Total number of spokes on the wheel
  /// @note @c number_of_spokes is expressed in units of integer.
  /// @note @c number_of_spokes has a limited domain of [4, +Inf].
  const T& number_of_spokes() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kNumberOfSpokes);
  }
  /// Setter that matches number_of_spokes().
  void set_number_of_spokes(const T& number_of_spokes) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kNumberOfSpokes, number_of_spokes);
  }
  /// Fluent setter that matches number_of_spokes().
  /// Returns a copy of `this` with number_of_spokes set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  RimlessWheelParams<T> with_number_of_spokes(const T& number_of_spokes) const {
    RimlessWheelParams<T> result(*this);
    result.set_number_of_spokes(number_of_spokes);
    return result;
  }
  /// The angle of the ramp on which the rimless wheel is walking.
  /// @note @c slope is expressed in units of radians.
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
  RimlessWheelParams<T> with_slope(const T& slope) const {
    RimlessWheelParams<T> result(*this);
    result.set_slope(slope);
    return result;
  }
  //@}

  /// See RimlessWheelParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return RimlessWheelParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(mass());
    result = result && (mass() >= T(0.0));
    result = result && !isnan(length());
    result = result && (length() >= T(0.0));
    result = result && !isnan(gravity());
    result = result && (gravity() >= T(0.0));
    result = result && !isnan(number_of_spokes());
    result = result && (number_of_spokes() >= T(4));
    result = result && !isnan(slope());
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(4);
    (*value)[0] = mass() - T(0.0);
    (*value)[1] = length() - T(0.0);
    (*value)[2] = gravity() - T(0.0);
    (*value)[3] = number_of_spokes() - T(4);
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The RimlessWheelParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake

#undef DRAKE_VECTOR_GEN_NODISCARD
