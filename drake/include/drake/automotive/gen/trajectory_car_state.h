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

/// Describes the row indices of a TrajectoryCarState.
struct TrajectoryCarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kPosition = 0;
  static const int kSpeed = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `TrajectoryCarStateIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class TrajectoryCarState final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef TrajectoryCarStateIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c position defaults to 0.0 with unknown units.
  /// @arg @c speed defaults to 0.0 with unknown units.
  TrajectoryCarState() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_position(0.0);
    this->set_speed(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  TrajectoryCarState(const TrajectoryCarState& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  TrajectoryCarState(TrajectoryCarState&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  TrajectoryCarState& operator=(const TrajectoryCarState& other) {
    this->values() = other.values();
    return *this;
  }
  TrajectoryCarState& operator=(TrajectoryCarState&& other) noexcept {
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
    this->set_position(symbolic::Variable("position"));
    this->set_speed(symbolic::Variable("speed"));
  }

  TrajectoryCarState<T>* DoClone() const final {
    return new TrajectoryCarState;
  }

  /// @name Getters and Setters
  //@{
  /// The along-curve position of the vehicle.
  const T& position() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kPosition);
  }
  /// Setter that matches position().
  void set_position(const T& position) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kPosition, position);
  }
  /// Fluent setter that matches position().
  /// Returns a copy of `this` with position set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  TrajectoryCarState<T> with_position(const T& position) const {
    TrajectoryCarState<T> result(*this);
    result.set_position(position);
    return result;
  }
  /// The along-curve speed of the vehicle.
  const T& speed() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kSpeed);
  }
  /// Setter that matches speed().
  void set_speed(const T& speed) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSpeed, speed);
  }
  /// Fluent setter that matches speed().
  /// Returns a copy of `this` with speed set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  TrajectoryCarState<T> with_speed(const T& speed) const {
    TrajectoryCarState<T> result(*this);
    result.set_speed(speed);
    return result;
  }
  //@}

  /// See TrajectoryCarStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return TrajectoryCarStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(position());
    result = result && !isnan(speed());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The TrajectoryCarState vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace automotive
}  // namespace drake

#undef DRAKE_VECTOR_GEN_NODISCARD
