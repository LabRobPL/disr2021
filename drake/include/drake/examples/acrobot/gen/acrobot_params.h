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
namespace acrobot {

/// Describes the row indices of a AcrobotParams.
struct AcrobotParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 10;

  // The index of each individual coordinate.
  static const int kM1 = 0;
  static const int kM2 = 1;
  static const int kL1 = 2;
  static const int kLc1 = 3;
  static const int kLc2 = 4;
  static const int kIc1 = 5;
  static const int kIc2 = 6;
  static const int kB1 = 7;
  static const int kB2 = 8;
  static const int kGravity = 9;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `AcrobotParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class AcrobotParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef AcrobotParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c m1 defaults to 1.0 kg.
  /// @arg @c m2 defaults to 1.0 kg.
  /// @arg @c l1 defaults to 1.0 m.
  /// @arg @c lc1 defaults to 0.5 m.
  /// @arg @c lc2 defaults to 1.0 m.
  /// @arg @c Ic1 defaults to 0.083 kg*m^2.
  /// @arg @c Ic2 defaults to 0.33 kg*m^2.
  /// @arg @c b1 defaults to 0.1 kg*m^2/s.
  /// @arg @c b2 defaults to 0.1 kg*m^2/s.
  /// @arg @c gravity defaults to 9.81 m/s^2.
  AcrobotParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_m1(1.0);
    this->set_m2(1.0);
    this->set_l1(1.0);
    this->set_lc1(0.5);
    this->set_lc2(1.0);
    this->set_Ic1(0.083);
    this->set_Ic2(0.33);
    this->set_b1(0.1);
    this->set_b2(0.1);
    this->set_gravity(9.81);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  AcrobotParams(const AcrobotParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  AcrobotParams(AcrobotParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  AcrobotParams& operator=(const AcrobotParams& other) {
    this->values() = other.values();
    return *this;
  }
  AcrobotParams& operator=(AcrobotParams&& other) noexcept {
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
    this->set_m1(symbolic::Variable("m1"));
    this->set_m2(symbolic::Variable("m2"));
    this->set_l1(symbolic::Variable("l1"));
    this->set_lc1(symbolic::Variable("lc1"));
    this->set_lc2(symbolic::Variable("lc2"));
    this->set_Ic1(symbolic::Variable("Ic1"));
    this->set_Ic2(symbolic::Variable("Ic2"));
    this->set_b1(symbolic::Variable("b1"));
    this->set_b2(symbolic::Variable("b2"));
    this->set_gravity(symbolic::Variable("gravity"));
  }

  AcrobotParams<T>* DoClone() const final { return new AcrobotParams; }

  /// @name Getters and Setters
  //@{
  /// Mass of link 1.
  /// @note @c m1 is expressed in units of kg.
  /// @note @c m1 has a limited domain of [0.0, +Inf].
  const T& m1() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kM1);
  }
  /// Setter that matches m1().
  void set_m1(const T& m1) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kM1, m1);
  }
  /// Fluent setter that matches m1().
  /// Returns a copy of `this` with m1 set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  AcrobotParams<T> with_m1(const T& m1) const {
    AcrobotParams<T> result(*this);
    result.set_m1(m1);
    return result;
  }
  /// Mass of link 2.
  /// @note @c m2 is expressed in units of kg.
  /// @note @c m2 has a limited domain of [0.0, +Inf].
  const T& m2() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kM2);
  }
  /// Setter that matches m2().
  void set_m2(const T& m2) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kM2, m2);
  }
  /// Fluent setter that matches m2().
  /// Returns a copy of `this` with m2 set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  AcrobotParams<T> with_m2(const T& m2) const {
    AcrobotParams<T> result(*this);
    result.set_m2(m2);
    return result;
  }
  /// Length of link 1.
  /// @note @c l1 is expressed in units of m.
  /// @note @c l1 has a limited domain of [0.0, +Inf].
  const T& l1() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kL1);
  }
  /// Setter that matches l1().
  void set_l1(const T& l1) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kL1, l1);
  }
  /// Fluent setter that matches l1().
  /// Returns a copy of `this` with l1 set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  AcrobotParams<T> with_l1(const T& l1) const {
    AcrobotParams<T> result(*this);
    result.set_l1(l1);
    return result;
  }
  /// Vertical distance from shoulder joint to center of mass of link 1.
  /// @note @c lc1 is expressed in units of m.
  /// @note @c lc1 has a limited domain of [0.0, +Inf].
  const T& lc1() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLc1);
  }
  /// Setter that matches lc1().
  void set_lc1(const T& lc1) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLc1, lc1);
  }
  /// Fluent setter that matches lc1().
  /// Returns a copy of `this` with lc1 set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  AcrobotParams<T> with_lc1(const T& lc1) const {
    AcrobotParams<T> result(*this);
    result.set_lc1(lc1);
    return result;
  }
  /// Vertical distance from elbow joint to center of mass of link 1.
  /// @note @c lc2 is expressed in units of m.
  /// @note @c lc2 has a limited domain of [0.0, +Inf].
  const T& lc2() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLc2);
  }
  /// Setter that matches lc2().
  void set_lc2(const T& lc2) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLc2, lc2);
  }
  /// Fluent setter that matches lc2().
  /// Returns a copy of `this` with lc2 set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  AcrobotParams<T> with_lc2(const T& lc2) const {
    AcrobotParams<T> result(*this);
    result.set_lc2(lc2);
    return result;
  }
  /// Inertia of link 1 about the center of mass of link 1.
  /// @note @c Ic1 is expressed in units of kg*m^2.
  /// @note @c Ic1 has a limited domain of [0.0, +Inf].
  const T& Ic1() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kIc1);
  }
  /// Setter that matches Ic1().
  void set_Ic1(const T& Ic1) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kIc1, Ic1);
  }
  /// Fluent setter that matches Ic1().
  /// Returns a copy of `this` with Ic1 set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  AcrobotParams<T> with_Ic1(const T& Ic1) const {
    AcrobotParams<T> result(*this);
    result.set_Ic1(Ic1);
    return result;
  }
  /// Inertia of link 2 about the center of mass of link 2.
  /// @note @c Ic2 is expressed in units of kg*m^2.
  /// @note @c Ic2 has a limited domain of [0.0, +Inf].
  const T& Ic2() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kIc2);
  }
  /// Setter that matches Ic2().
  void set_Ic2(const T& Ic2) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kIc2, Ic2);
  }
  /// Fluent setter that matches Ic2().
  /// Returns a copy of `this` with Ic2 set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  AcrobotParams<T> with_Ic2(const T& Ic2) const {
    AcrobotParams<T> result(*this);
    result.set_Ic2(Ic2);
    return result;
  }
  /// Damping coefficient of the shoulder joint.
  /// @note @c b1 is expressed in units of kg*m^2/s.
  /// @note @c b1 has a limited domain of [0.0, +Inf].
  const T& b1() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kB1);
  }
  /// Setter that matches b1().
  void set_b1(const T& b1) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kB1, b1);
  }
  /// Fluent setter that matches b1().
  /// Returns a copy of `this` with b1 set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  AcrobotParams<T> with_b1(const T& b1) const {
    AcrobotParams<T> result(*this);
    result.set_b1(b1);
    return result;
  }
  /// Damping coefficient of the elbow joint.
  /// @note @c b2 is expressed in units of kg*m^2/s.
  /// @note @c b2 has a limited domain of [0.0, +Inf].
  const T& b2() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kB2);
  }
  /// Setter that matches b2().
  void set_b2(const T& b2) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kB2, b2);
  }
  /// Fluent setter that matches b2().
  /// Returns a copy of `this` with b2 set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  AcrobotParams<T> with_b2(const T& b2) const {
    AcrobotParams<T> result(*this);
    result.set_b2(b2);
    return result;
  }
  /// Gravitational constant.
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
  AcrobotParams<T> with_gravity(const T& gravity) const {
    AcrobotParams<T> result(*this);
    result.set_gravity(gravity);
    return result;
  }
  //@}

  /// See AcrobotParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return AcrobotParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(m1());
    result = result && (m1() >= T(0.0));
    result = result && !isnan(m2());
    result = result && (m2() >= T(0.0));
    result = result && !isnan(l1());
    result = result && (l1() >= T(0.0));
    result = result && !isnan(lc1());
    result = result && (lc1() >= T(0.0));
    result = result && !isnan(lc2());
    result = result && (lc2() >= T(0.0));
    result = result && !isnan(Ic1());
    result = result && (Ic1() >= T(0.0));
    result = result && !isnan(Ic2());
    result = result && (Ic2() >= T(0.0));
    result = result && !isnan(b1());
    result = result && (b1() >= T(0.0));
    result = result && !isnan(b2());
    result = result && (b2() >= T(0.0));
    result = result && !isnan(gravity());
    result = result && (gravity() >= T(0.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(10);
    (*value)[0] = m1() - T(0.0);
    (*value)[1] = m2() - T(0.0);
    (*value)[2] = l1() - T(0.0);
    (*value)[3] = lc1() - T(0.0);
    (*value)[4] = lc2() - T(0.0);
    (*value)[5] = Ic1() - T(0.0);
    (*value)[6] = Ic2() - T(0.0);
    (*value)[7] = b1() - T(0.0);
    (*value)[8] = b2() - T(0.0);
    (*value)[9] = gravity() - T(0.0);
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The AcrobotParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake

#undef DRAKE_VECTOR_GEN_NODISCARD
