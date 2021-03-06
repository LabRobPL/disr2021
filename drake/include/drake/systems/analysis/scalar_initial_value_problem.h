#pragma once

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/initial_value_problem.h"
#include "drake/systems/analysis/scalar_view_dense_output.h"

namespace drake {
namespace systems {

/// A thin wrapper of the InitialValueProblem class to provide a simple
/// interface when solving scalar initial value problems i.e. when evaluating
/// the x(t; ๐ค) solution function to the given ODE dx/dt = f(t, x; ๐ค),
/// where f : t โจฏ x โ  โ , t โ โ, x โ โ, ๐ค โ โแต, along with an initial
/// condition x(tโ; ๐ค) = xโ. The parameter vector ๐ค allows for generic IVP
/// definitions, which can later be solved for any instance of said vector.
///
/// Note the distinction from general initial value problems where
/// f : t โจฏ ๐ฑ โ โโฟ and ๐ฑ โ โโฟ, addressed by the class being wrapped. While
/// every scalar initial value problem could be written in vector form, this
/// wrapper keeps both problem definition and solution in their scalar form
/// with almost zero overhead, leading to clearer code if applicable.
/// Moreover, this scalar form facilitates single-dimensional quadrature
/// using methods for solving initial value problems.
///
/// See InitialValueProblem class documentation for information on caching
/// support and dense output usage for improved efficiency in scalar IVP
/// solving.
///
/// For further insight into its use, consider the following examples of scalar
/// IVPs:
///
/// - The population growth of an hypothetical bacteria colony is described
///   by dN/dt = r * N. The colony has Nโ subjects at time tโ. In this
///   context, x โ N, xโ โ Nโ, ๐ค โ [r], dx/dt = f(t, x; ๐ค) = ๐คโ * x.
///
/// - The charge Q stored in the capacitor of a (potentially equivalent) series
///   RC circuit driven by a time varying voltage source E(t) can be described
///   by dQ/dt = (E(t) - Q / Cs) / Rs, where Rs refers to the resistor's
///   resistance and Cs refers to the capacitor's capacitance. In this context,
///   and assuming an initial stored charge Qโ at time tโ, x โ Q, ๐ค โ [Rs, Cs],
///   xโ โ Qโ, dx/dt = f(t, x; ๐ค) = (E(t) - x / ๐คโ) / ๐คโ.
///
/// @tparam T The โ domain scalar type, which must be a valid Eigen scalar.
///
/// @note
/// Instantiated templates for the following scalar types @p T are provided:
///
/// - double
template <typename T>
class ScalarInitialValueProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScalarInitialValueProblem);

  /// Scalar ODE dx/dt = f(t, x; ๐ค) function type.
  ///
  /// @param t The independent variable t โ โ .
  /// @param x The dependent variable x โ โ .
  /// @param k The parameter vector ๐ค โ โแต.
  /// @return The derivative dx/dt โ โ.
  using ScalarODEFunction = std::function<T(const T& t, const T& x,
                                            const VectorX<T>& k)>;

  /// A collection of values i.e. initial time tโ, initial state xโ
  /// and parameter vector ๐ค to further specify the ODE system (in
  /// order to become a scalar initial value problem).
  struct SpecifiedValues {
    /// Default constructor, leaving all values unspecified.
    SpecifiedValues() = default;

    /// Constructor specifying all values.
    ///
    /// @param t0_in Specified initial time tโ.
    /// @param x0_in Specified initial state xโ.
    /// @param k_in Specified parameter vector ๐ค.
    SpecifiedValues(const optional<T>& t0_in,
                    const optional<T>& x0_in,
                    const optional<VectorX<T>>& k_in)
        : t0(t0_in), x0(x0_in), k(k_in) {}

    optional<T> t0;  ///< The initial time tโ for the IVP.
    optional<T> x0;  ///< The initial state xโ for the IVP.
    optional<VectorX<T>> k;  ///< The parameter vector ๐ค for the IVP.
  };

  /// Constructs an scalar IVP described by the given @p scalar_ode_function,
  /// using given @p default_values.t0 and @p default_values.x0 as initial
  /// conditions, and parameterized with @p default_values.k by default.
  ///
  /// @param scalar_ode_function The ODE function f(t, x; ๐ค) that describes the
  ///                            state evolution over time.
  /// @param default_values The values specified by default for this IVP, i.e.
  ///                       default initial time tโ โ โ and state xโ โ โ, and
  ///                       default parameter vector ๐ค โ โแต.
  /// @pre An initial time @p default_values.t0 is provided.
  /// @pre An initial state @p default_values.x0 is provided.
  /// @pre An parameter vector @p default_values.k is provided.
  /// @throws std::logic_error if preconditions are not met.
  ScalarInitialValueProblem(const ScalarODEFunction& scalar_ode_function,
                            const SpecifiedValues& default_values) {
    // Wraps the given scalar ODE function as a vector ODE function.
    typename InitialValueProblem<T>::ODEFunction ode_function =
        [scalar_ode_function](const T& t, const VectorX<T>& x,
                              const VectorX<T>& k) -> VectorX<T> {
      return VectorX<T>::Constant(1, scalar_ode_function(t, x[0], k));
    };
    // Instantiates the vector initial value problem.
    vector_ivp_ = std::make_unique<InitialValueProblem<T>>(
        ode_function, ToVectorIVPSpecifiedValues(default_values));
  }

  /// Solves the IVP for time @p tf, using the initial time tโ, initial state
  /// xโ and parameter vector ๐ค present in @p values, falling back to the ones
  /// given on construction if not given.
  ///
  /// @param tf The IVP will be solved for this time.
  /// @param values IVP initial conditions and parameters.
  /// @returns The IVP solution x(@p tf; ๐ค) for x(tโ; ๐ค) = xโ.
  /// @pre Given @p tf must be larger than or equal to the specified initial
  ///      time tโ (either given or default).
  /// @pre If given, the dimension of the parameter vector @p values.k
  ///      must match that of the parameter vector in the default specified
  ///      values given on construction.
  /// @throws std::logic_error if any of the preconditions is not met.
  T Solve(const T& tf, const SpecifiedValues& values = {}) const {
    return this->vector_ivp_->Solve(tf, ToVectorIVPSpecifiedValues(values))[0];
  }

  /// Solves and yields an approximation of the IVP solution x(t; ๐ค) for the
  /// closed time interval between the initial time tโ and the given final
  /// time @p tf, using initial state xโ and parameter vector ๐ค present in
  /// @p values (falling back to the ones given on construction if not given).
  ///
  /// To this end, the wrapped IntegratorBase instance solves this scalar IVP,
  /// advancing time and state from tโ and xโ = x(tโ) to @p tf and x(@p tf),
  /// creating a scalar dense output over that [tโ, @p tf] interval along the
  /// way.
  ///
  /// @param tf The IVP will be solved up to this time. Usually, tโ < @p tf as
  ///           an empty dense output would result if tโ = @p tf.
  /// @param values IVP initial conditions and parameters.
  /// @returns A dense approximation to x(t; ๐ค) with x(tโ; ๐ค) = xโ, defined for
  ///          tโ โค t โค tf.
  /// @note The larger the given @p tf value is, the larger the approximated
  ///       interval will be. See documentation of the specific dense output
  ///       technique in use for reference on performance impact as this
  ///       interval grows.
  /// @pre Given @p tf must be larger than or equal to the specified initial
  ///      time tโ (either given or default).
  /// @pre If given, the dimension of the initial state vector @p values.x0
  ///      must match that of the default initial state vector in the default
  ///      specified values given on construction.
  /// @pre If given, the dimension of the parameter vector @p values.k
  ///      must match that of the parameter vector in the default specified
  ///      values given on construction.
  /// @throws std::logic_error if any of the preconditions is not met.
  std::unique_ptr<ScalarDenseOutput<T>> DenseSolve(
      const T& tf, const SpecifiedValues& values = {}) const {
    // Delegates request to the vector form of this IVP by putting
    // specified values in vector form and the resulting dense output
    // back into scalar form.
    const int kDimension = 0;
    std::unique_ptr<DenseOutput<T>> vector_dense_output =
        this->vector_ivp_->DenseSolve(tf, ToVectorIVPSpecifiedValues(values));
    return std::make_unique<ScalarViewDenseOutput<T>>(
        std::move(vector_dense_output), kDimension);
  }

  /// Resets the internal integrator instance by in-place
  /// construction of the given integrator type.
  ///
  /// A usage example is shown below.
  /// @code{.cpp}
  ///    scalar_ivp.reset_integrator<RungeKutta2Integrator<T>>(max_step);
  /// @endcode
  ///
  /// @param args The integrator type-specific arguments.
  /// @returns The new integrator instance.
  /// @tparam Integrator The integrator type, which must be an
  ///                    IntegratorBase subclass.
  /// @tparam Args The integrator specific argument types.
  /// @warning This operation invalidates pointers returned by
  ///          ScalarInitialValueProblem::get_integrator() and
  ///          ScalarInitialValueProblem::get_mutable_integrator().
  template <typename Integrator, typename... Args>
  Integrator* reset_integrator(Args&&... args) {
    return vector_ivp_->template reset_integrator<Integrator>(
        std::forward<Args>(args)...);
  }

  /// Gets a pointer to the internal integrator instance.
  const IntegratorBase<T>* get_integrator() const {
    return vector_ivp_->get_integrator();
  }

  /// Gets a pointer to the internal mutable integrator instance.
  IntegratorBase<T>* get_mutable_integrator() {
    return vector_ivp_->get_mutable_integrator();
  }

 private:
  // Transforms given scalar IVP specified @p values into vector
  // IVP specified values.
  static typename InitialValueProblem<T>::SpecifiedValues
  ToVectorIVPSpecifiedValues(const SpecifiedValues& values) {
    typename InitialValueProblem<T>::SpecifiedValues vector_ivp_values;
    vector_ivp_values.k = values.k;
    vector_ivp_values.t0 = values.t0;
    if (values.x0.has_value()) {
      // Scalar initial state xโ as a vector initial state ๐ฑโ
      // of a single dimension.
      vector_ivp_values.x0 = VectorX<T>::Constant(
          1, values.x0.value()).eval();
    }
    return vector_ivp_values;
  }

  // Vector IVP representation of this scalar IVP.
  std::unique_ptr<InitialValueProblem<T>> vector_ivp_;
};

}  // namespace systems
}  // namespace drake
