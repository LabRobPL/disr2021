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
namespace systems {
namespace sensors {

/// Describes the row indices of a BeamModelParams.
struct BeamModelParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 5;

  // The index of each individual coordinate.
  static const int kLambdaShort = 0;
  static const int kSigmaHit = 1;
  static const int kProbabilityShort = 2;
  static const int kProbabilityMiss = 3;
  static const int kProbabilityUniform = 4;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `BeamModelParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class BeamModelParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef BeamModelParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c lambda_short defaults to 1.0 dimensionless.
  /// @arg @c sigma_hit defaults to 0.0 m.
  /// @arg @c probability_short defaults to 0.0 dimensionless.
  /// @arg @c probability_miss defaults to 0.0 dimensionless.
  /// @arg @c probability_uniform defaults to 0.0 dimensionless.
  BeamModelParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_lambda_short(1.0);
    this->set_sigma_hit(0.0);
    this->set_probability_short(0.0);
    this->set_probability_miss(0.0);
    this->set_probability_uniform(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  BeamModelParams(const BeamModelParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  BeamModelParams(BeamModelParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  BeamModelParams& operator=(const BeamModelParams& other) {
    this->values() = other.values();
    return *this;
  }
  BeamModelParams& operator=(BeamModelParams&& other) noexcept {
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
    this->set_lambda_short(symbolic::Variable("lambda_short"));
    this->set_sigma_hit(symbolic::Variable("sigma_hit"));
    this->set_probability_short(symbolic::Variable("probability_short"));
    this->set_probability_miss(symbolic::Variable("probability_miss"));
    this->set_probability_uniform(symbolic::Variable("probability_uniform"));
  }

  BeamModelParams<T>* DoClone() const final { return new BeamModelParams; }

  /// @name Getters and Setters
  //@{
  /// The rate parameter of the (truncated) exponential distribution governing
  /// short returns
  /// @note @c lambda_short is expressed in units of dimensionless.
  /// @note @c lambda_short has a limited domain of [0.0, +Inf].
  const T& lambda_short() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLambdaShort);
  }
  /// Setter that matches lambda_short().
  void set_lambda_short(const T& lambda_short) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLambdaShort, lambda_short);
  }
  /// Fluent setter that matches lambda_short().
  /// Returns a copy of `this` with lambda_short set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BeamModelParams<T> with_lambda_short(const T& lambda_short) const {
    BeamModelParams<T> result(*this);
    result.set_lambda_short(lambda_short);
    return result;
  }
  /// The standard deviation of the (truncated) Gaussian distribution governing
  /// the noisy returns of the true depth (aka hit)
  /// @note @c sigma_hit is expressed in units of m.
  /// @note @c sigma_hit has a limited domain of [0.0, +Inf].
  const T& sigma_hit() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kSigmaHit);
  }
  /// Setter that matches sigma_hit().
  void set_sigma_hit(const T& sigma_hit) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSigmaHit, sigma_hit);
  }
  /// Fluent setter that matches sigma_hit().
  /// Returns a copy of `this` with sigma_hit set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BeamModelParams<T> with_sigma_hit(const T& sigma_hit) const {
    BeamModelParams<T> result(*this);
    result.set_sigma_hit(sigma_hit);
    return result;
  }
  /// The total probability of getting a short return is probability_short *
  /// p(lambda_short*w_short <= input_depth)
  /// @note @c probability_short is expressed in units of dimensionless.
  /// @note @c probability_short has a limited domain of [0.0, 1.0].
  const T& probability_short() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kProbabilityShort);
  }
  /// Setter that matches probability_short().
  void set_probability_short(const T& probability_short) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kProbabilityShort, probability_short);
  }
  /// Fluent setter that matches probability_short().
  /// Returns a copy of `this` with probability_short set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BeamModelParams<T> with_probability_short(const T& probability_short) const {
    BeamModelParams<T> result(*this);
    result.set_probability_short(probability_short);
    return result;
  }
  /// The probability of ignoring the input depth and simply returning the max
  /// range of the sensor
  /// @note @c probability_miss is expressed in units of dimensionless.
  /// @note @c probability_miss has a limited domain of [0.0, 1.0].
  const T& probability_miss() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kProbabilityMiss);
  }
  /// Setter that matches probability_miss().
  void set_probability_miss(const T& probability_miss) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kProbabilityMiss, probability_miss);
  }
  /// Fluent setter that matches probability_miss().
  /// Returns a copy of `this` with probability_miss set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BeamModelParams<T> with_probability_miss(const T& probability_miss) const {
    BeamModelParams<T> result(*this);
    result.set_probability_miss(probability_miss);
    return result;
  }
  /// The probability of ignoring the input depth and simple returning a uniform
  /// random value between 0 and the max range of the sensor
  /// @note @c probability_uniform is expressed in units of dimensionless.
  /// @note @c probability_uniform has a limited domain of [0.0, 1.0].
  const T& probability_uniform() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kProbabilityUniform);
  }
  /// Setter that matches probability_uniform().
  void set_probability_uniform(const T& probability_uniform) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kProbabilityUniform, probability_uniform);
  }
  /// Fluent setter that matches probability_uniform().
  /// Returns a copy of `this` with probability_uniform set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BeamModelParams<T> with_probability_uniform(
      const T& probability_uniform) const {
    BeamModelParams<T> result(*this);
    result.set_probability_uniform(probability_uniform);
    return result;
  }
  //@}

  /// See BeamModelParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return BeamModelParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(lambda_short());
    result = result && (lambda_short() >= T(0.0));
    result = result && !isnan(sigma_hit());
    result = result && (sigma_hit() >= T(0.0));
    result = result && !isnan(probability_short());
    result = result && (probability_short() >= T(0.0));
    result = result && (probability_short() <= T(1.0));
    result = result && !isnan(probability_miss());
    result = result && (probability_miss() >= T(0.0));
    result = result && (probability_miss() <= T(1.0));
    result = result && !isnan(probability_uniform());
    result = result && (probability_uniform() >= T(0.0));
    result = result && (probability_uniform() <= T(1.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(8);
    (*value)[0] = lambda_short() - T(0.0);
    (*value)[1] = sigma_hit() - T(0.0);
    (*value)[2] = probability_short() - T(0.0);
    (*value)[3] = T(1.0) - probability_short();
    (*value)[4] = probability_miss() - T(0.0);
    (*value)[5] = T(1.0) - probability_miss();
    (*value)[6] = probability_uniform() - T(0.0);
    (*value)[7] = T(1.0) - probability_uniform();
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The BeamModelParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake

#undef DRAKE_VECTOR_GEN_NODISCARD
