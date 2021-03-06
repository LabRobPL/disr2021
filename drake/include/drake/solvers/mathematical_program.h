#pragma once

#include <array>
#include <cstddef>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/create_constraint.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/function.h"
#include "drake/solvers/indeterminate.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/mathematical_program_solver_interface.h"
#include "drake/solvers/program_attribute.h"
#include "drake/solvers/solution_result.h"
#include "drake/solvers/solver_options.h"
#include "drake/solvers/solver_result.h"

namespace drake {
namespace solvers {

/** @addtogroup solvers
 * @{
 * Drake wraps a number of commercial solvers (+ a few custom solvers) to
 * provide a common interface for convex optimization, mixed-integer convex
 * optimization, and other non-convex mathematical programs.
 *
 * The MathematicalProgram class handles the coordination of decision variables,
 * objectives, and constraints.  The MathematicalProgram::Solve() method
 * reflects on the accumulated objectives and constraints and will dispatch to
 * the most appropriate solver.  Alternatively, one can invoke specific solver
 * by instantiating its MathematicalProgramSolverInterface and passing the
 * MathematicalProgram directly to the
 * MathematicalProgramSolverInterface::Solve() method.
 *
 * Our solver coverage still has many gaps, but is under active development.
 *
 * <b>Closed-form solutions</b>
 *
 * The LinearSystemSolver and EqualityConstrainedQPSolver classes provide
 * efficient closed-form solutions to these special cases.
 *
 * <b>Convex Optimization</b>
 *
 * <table>
 * <tr>
 *   <td>Solver</td>
 *   <td><a href="https://en.wikipedia.org/wiki/Linear_programming">LP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Quadratic_programming">
 *     QP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Second-order_cone_programming">
 *     SOCP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Semidefinite_programming">
 *     SDP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Sum-of-squares_optimization">
 *     SOS</a></td>
 * </tr>
 * <tr><td>&dagger; <a href="https://www.gurobi.com/products/gurobi-optimizer">
 *    Gurobi</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 *  </tr>
 * <tr><td>&dagger; <a href="https://www.mosek.com/products/mosek">
 *    Mosek</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 * </tr>
 * <tr><td> <a href="https://github.com/cvxgrp/scs">
 *    SCS</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 * </tr>
 * <tr><td> <a href="https://github.com/oxfordcontrol/osqp">
 *    OSQP</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 *    <td></td>
 * </tr>
 * </table>
 *
 * <b>Mixed-Integer Convex Optimization</b>
 *
 * <table>
 * <tr>
 *   <td>Solver</td>
 *   <td>MILP</a></td>
 *   <td>MIQP</a></td>
 *   <td>MISOCP</a></td>
 *   <td>MISDP</a></td>
 * </tr>
 * <tr><td>&dagger; <a href="https://www.gurobi.com/products/gurobi-optimizer">
 *    Gurobi</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *  </tr>
 * <tr><td>&dagger; <a href="https://www.mosek.com/products/mosek">
 *    Mosek</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 * </tr>
 * </table>
 *
 * <b>Nonconvex Programming</b>
 *
 * <table>
 * <tr>
 *   <td>Solver</td>
 *   <td><a href="https://en.wikipedia.org/wiki/Nonlinear_programming">
 *     Nonlinear Program</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Linear_complementarity_problem">
 *   LCP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Satisfiability_modulo_theories">
 *     SMT</a></td>
 * </tr>
 * <tr><td>&dagger;
 *   <a href="http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm">
 *    SNOPT</a></td></tr>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 * <tr><td><a href="https://projects.coin-or.org/Ipopt">Ipopt</a></td></tr>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 * <tr><td><a href="http://ab-initio.mit.edu/wiki/index.php/NLopt">
 *    NLopt</a></td></tr>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 * <tr><td><a href="https://github.com/PositronicsLab/Moby">
 *    Moby LCP</a></td>
 *    <td></td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 * <tr><td><a href="https://dreal.github.io/">dReal</a></td>
 *    <td></td>
 *    <td></td>
 *    <td align="center">&diams;</td>
 * </tr>
 * </table>
 *
 * &dagger; indicates that this is a commercial solver which requires a license
 * (note that some have free licenses for academics).
 * @}
 */
class MathematicalProgram;

template <int...>
struct NewVariableNames {};
/**
   * The type of the names for the newly added variables.
   * @tparam Size If Size is a fixed non-negative integer, then the type of the
   * name is std::array<std::string, Size>. Otherwise the type is
   * std::vector<std::string>.
   */
template <int Size>
struct NewVariableNames<Size> {
  typedef std::array<std::string, Size> type;
};

template <>
struct NewVariableNames<Eigen::Dynamic> {
  typedef std::vector<std::string> type;
};

template <int Rows, int Cols>
struct NewVariableNames<Rows, Cols>
    : public NewVariableNames<MultiplyEigenSizes<Rows, Cols>::value> {};

template <int Rows>
struct NewSymmetricVariableNames
    : public NewVariableNames<Rows == Eigen::Dynamic ? Eigen::Dynamic
                                                     : Rows*(Rows + 1) / 2> {};

namespace internal {
/**
 * Return un-initialized new variable names.
 */
template <int Size>
typename std::enable_if<Size >= 0, typename NewVariableNames<Size>::type>::type
CreateNewVariableNames(int) {
  typename NewVariableNames<Size>::type names;
  return names;
}

/**
 * Return un-initialized new variable names.
 */
template <int Size>
typename std::enable_if<Size == Eigen::Dynamic,
                        typename NewVariableNames<Size>::type>::type
CreateNewVariableNames(int size) {
  typename NewVariableNames<Eigen::Dynamic>::type names(size);
  return names;
}
/**
 * Set the names of the newly added variables.
 * @param name The common name of all new variables.
 * @param rows The number of rows in the new variables.
 * @param cols The number of columns in the new variables.
 * @pre The size of @p names is @p rows * @p cols.
 */
template <typename Derived>
void SetVariableNames(const std::string& name, int rows, int cols,
                      Derived* names) {
  DRAKE_DEMAND(static_cast<int>(names->size()) == rows * cols);
  if (cols == 1) {
    for (int i = 0; i < rows; ++i) {
      (*names)[i] = name + "(" + std::to_string(i) + ")";
    }
  } else {
    for (int j = 0; j < cols; ++j) {
      for (int i = 0; i < rows; ++i) {
        (*names)[j * rows + i] =
            name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
      }
    }
  }
}
}  // namespace internal

namespace detail {
/**
 * Template condition to only catch when Constraints are inadvertently passed
 * as an argument. If the class is binding-compatible with a Constraint, then
 * this will provide a static assertion to enable easier debugging of which
 * type failed.
 * @tparam F The type to be tested.
 * @see http://stackoverflow.com/a/13366183/7829525
 */
template <typename F>
struct assert_if_is_constraint {
  static constexpr bool value = is_binding_compatible<F, Constraint>::value;
  // Use deferred evaluation
  static_assert(
      !value,
      "You cannot pass a Constraint to create a Cost object from a function. "
      "Please ensure you are passing a Cost.");
};
}  // namespace detail

/**
 * MathematicalProgram stores the decision variables, the constraints and costs
 * of an optimization problem. The user can solve the problem by calling Solve()
 * function, and obtain the results of the optimization.
 *
 * @ingroup solvers
 */
class MathematicalProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MathematicalProgram)
  using VarType = symbolic::Variable::Type;

  /// The optimal cost is +??? when the problem is globally infeasible.
  static constexpr double kGlobalInfeasibleCost =
      std::numeric_limits<double>::infinity();
  /// The optimal cost is -??? when the problem is unbounded.
  static constexpr double kUnboundedCost =
      -std::numeric_limits<double>::infinity();

  MathematicalProgram();
  virtual ~MathematicalProgram();

  /** Clones an optimization program.
   * The clone will be functionally equivalent to the source program with the
   * same:
   *
   * - decision variables
   * - constraints
   * - costs
   * - solver settings
   * - initial guess
   *
   * However, the clone's x values will be initialized to NaN, and all internal
   * solvers will be freshly constructed.
   * @retval new_prog. The newly constructed mathematical program.
   */
  std::unique_ptr<MathematicalProgram> Clone() const;

  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @param rows  The number of rows in the new variables.
   * @param name The name of the newly added variables
   * @return The VectorDecisionVariable of size rows x 1, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables(2, "x");
   * @endcode
   * This adds a 2 x 1 vector containing decision variables into the program.
   * The names of the variables are "x(0)" and "x(1)".
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  VectorXDecisionVariable NewContinuousVariables(
      int rows, const std::string& name = "x") {
    return NewContinuousVariables<Eigen::Dynamic, 1>(rows, 1, name);
  }

  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam Rows The number of rows of the new variables, in the compile time.
   * @tparam Cols The number of columns of the new variables, in the compile
   * time.
   * @param rows The number of rows in the new variables. When Rows is not
   * Eigen::Dynamic, rows is ignored.
   * @param cols The number of columns in the new variables. When Cols is not
   * Eigen::Dynamic, cols is ignored.
   * @param name All variables will share the same name, but different index.
   * @return The MatrixDecisionVariable of size Rows x Cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables(2, 3, "X");
   * auto y = prog.NewContinuousVariables<2, 3>(2, 3, "X");
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  template <int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
  MatrixDecisionVariable<Rows, Cols> NewContinuousVariables(
      int rows, int cols, const std::string& name = "X") {
    rows = Rows == Eigen::Dynamic ? rows : Rows;
    cols = Cols == Eigen::Dynamic ? cols : Cols;
    auto names =
        internal::CreateNewVariableNames<MultiplyEigenSizes<Rows, Cols>::value>(
            rows * cols);
    internal::SetVariableNames(name, rows, cols, &names);
    return NewVariables<Rows, Cols>(VarType::CONTINUOUS, names, rows, cols);
  }

  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam Rows  The number of rows in the new variables.
   * @tparam Cols  The number of columns in the new variables. The default is 1.
   * @param name All variables will share the same name, but different index.
   * @return The MatrixDecisionVariable of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables<2, 3>("X");
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  template <int Rows, int Cols = 1>
  MatrixDecisionVariable<Rows, Cols> NewContinuousVariables(
      const std::string& name = "X") {
    return NewContinuousVariables<Rows, Cols>(Rows, Cols, name);
  }

  /**
   * Adds binary variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam Rows  The number of rows in the new variables.
   * @tparam Cols  The number of columns in the new variables.
   * @param rows The number of rows in the new variables.
   * @param cols The number of columns in the new variables.
   * @param name The commonly shared name of the new variables.
   * @return The MatrixDecisionVariable of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto b = prog.NewBinaryVariables(2, 3, "b");
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  template <int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
  MatrixDecisionVariable<Rows, Cols> NewBinaryVariables(
      int rows, int cols, const std::string& name) {
    rows = Rows == Eigen::Dynamic ? rows : Rows;
    cols = Cols == Eigen::Dynamic ? cols : Cols;
    auto names =
        internal::CreateNewVariableNames<MultiplyEigenSizes<Rows, Cols>::value>(
            rows * cols);
    internal::SetVariableNames(name, rows, cols, &names);
    return NewVariables<Rows, Cols>(VarType::BINARY, names, rows, cols);
  }

  /**
   * Adds a matrix of binary variables into the optimization program.
   * @tparam Rows The number of rows in the newly added binary variables.
   * @tparam Cols  The number of columns in the new variables. The default is 1.
   * @param name Each newly added binary variable will share the same name. The
   * default name is "b".
   * @return A matrix containing the newly added variables.
   */
  template <int Rows, int Cols = 1>
  MatrixDecisionVariable<Rows, Cols> NewBinaryVariables(
      const std::string& name = "b") {
    return NewBinaryVariables<Rows, Cols>(Rows, Cols, name);
  }

  /**
   * Adds binary variables to this MathematicalProgram. The new variables are
   * viewed as a column vector, with size @p rows x 1.
   * @see NewBinaryVariables(int rows, int cols, const
   * std::vector<std::string>& names);
   */
  VectorXDecisionVariable NewBinaryVariables(int rows,
                                             const std::string& name = "b") {
    return NewBinaryVariables<Eigen::Dynamic, 1>(rows, 1, name);
  }

  /**
   * Adds a runtime sized symmetric matrix as decision variables to
   * this MathematicalProgram.
   * The optimization will only use the stacked columns of the
   * lower triangular part of the symmetric matrix as decision variables.
   * @param rows The number of rows in the symmetric matrix.
   * @param name The name of the matrix. It is only used the for user to
   * understand the optimization program. The default name is "Symmetric", and
   * each variable will be named as
   * <pre>
   * Symmetric(0, 0)     Symmetric(1, 0)     ... Symmetric(rows-1, 0)
   * Symmetric(1, 0)     Symmetric(1, 1)     ... Symmetric(rows-1, 1)
   *            ...
   * Symmetric(rows-1,0) Symmetric(rows-1,1) ... Symmetric(rows-1, rows-1)
   * </pre>
   * Notice that the (i,j)'th entry and (j,i)'th entry has the same name.
   * @return The newly added decision variables.
   */
  MatrixXDecisionVariable NewSymmetricContinuousVariables(
      int rows, const std::string& name = "Symmetric");

  /**
   * Adds a static sized symmetric matrix as decision variables to
   * this MathematicalProgram.
   * The optimization will only use the stacked columns of the
   * lower triangular part of the symmetric matrix as decision variables.
   * @tparam rows The number of rows in the symmetric matrix.
   * @param name The name of the matrix. It is only used the for user to
   * understand the optimization program. The default name is "Symmetric", and
   * each variable will be named as
   * <pre>
   * Symmetric(0, 0)     Symmetric(1, 0)     ... Symmetric(rows-1, 0)
   * Symmetric(1, 0)     Symmetric(1, 1)     ... Symmetric(rows-1, 1)
   *            ...
   * Symmetric(rows-1,0) Symmetric(rows-1,1) ... Symmetric(rows-1, rows-1)
   * </pre>
   * Notice that the (i,j)'th entry and (j,i)'th entry has the same name.
   * @return The newly added decision variables.
   */
  template <int rows>
  MatrixDecisionVariable<rows, rows> NewSymmetricContinuousVariables(
      const std::string& name = "Symmetric") {
    typename NewSymmetricVariableNames<rows>::type names;
    int var_count = 0;
    for (int j = 0; j < static_cast<int>(rows); ++j) {
      for (int i = j; i < static_cast<int>(rows); ++i) {
        names[var_count] =
            name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
        ++var_count;
      }
    }
    return NewSymmetricVariables<rows>(VarType::CONTINUOUS, names);
  }

  /** Appends new variables to the end of the existing variables.
   * @param decision_variables The newly added decision_variables.
   * @pre `decision_variables` should not intersect with the existing variables
   * or indeterminates in the optimization program.
   * @pre Each entry in `decision_variables` should not be a dummy variable.
   * @throws std::runtime_error if the preconditions are not satisfied.
   */
  void AddDecisionVariables(
      const Eigen::Ref<const VectorXDecisionVariable>& decision_variables);

  /**
   * Returns a free polynomial in a monomial basis over @p indeterminates of a
   * given @p degree. It uses @p coeff_name to make new decision variables and
   * use them as coefficients. For example, `NewFreePolynomial({x???, x???}, 2)`
   * returns a???x????? + a???x???x??? + a???x????? + a???x??? + a???x??? + a???.
   */
  symbolic::Polynomial NewFreePolynomial(
      const symbolic::Variables& indeterminates, int degree,
      const std::string& coeff_name = "a");

  /** Returns a pair of a SOS polynomial p = m???Qm and a PSD constraint for
   * a new coefficients matrix Q, where m is the @p monomial basis.
   * For example, `NewSosPolynomial(Vector2<Monomial>{x,y})` returns a
   * polynomial
   *   p = Q??????,??????x?? + 2Q??????,??????xy + Q??????,??????y??
   * and a PSD constraint over Q.
   * @note Q is a symmetric monomial_basis.rows() x monomial_basis.rows()
   * matrix.
   */
  std::pair<symbolic::Polynomial, Binding<PositiveSemidefiniteConstraint>>
  NewSosPolynomial(
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis);

  /** Returns a pair of a SOS polynomial p = m(x)???Qm(x) of degree @p degree
   * and a PSD constraint for the coefficients matrix Q, where m(x) is the
   * result of calling `MonomialBasis(indeterminates, degree/2)`. For example,
   * `NewSosPolynomial({x}, 4)` returns a pair of a polynomial
   *   p = Q??????,??????x??? + 2Q??????,?????? x?? + (2Q??????,?????? + Q??????,??????)x?? + 2Q??????,??????x + Q??????,??????
   * and a PSD constraint over Q.
   *
   * @throws std::runtime_error if @p degree is not a positive even integer.
   * @see MonomialBasis.
   */
  std::pair<symbolic::Polynomial, Binding<PositiveSemidefiniteConstraint>>
  NewSosPolynomial(const symbolic::Variables& indeterminates, int degree);

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing indeterminates.
   * @tparam rows  The number of rows in the new indeterminates.
   * @tparam cols  The number of columns in the new indeterminates.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * std::array<std::string, 6> names = {"x1", "x2", "x3", "x4", "x5", "x6"};
   * auto x = prog.NewIndeterminates<2, 3>(names);
   * @endcode
   * This adds a 2 x 3 matrix indeterminates into the program.
   *
   * The name of the indeterminates is only used for the user in order to ease
   * readability.
   */
  template <int rows, int cols>
  MatrixIndeterminate<rows, cols> NewIndeterminates(
      const std::array<std::string, rows * cols>& names) {
    MatrixIndeterminate<rows, cols> indeterminates_matrix;
    NewIndeterminates_impl(names, indeterminates_matrix);
    return indeterminates_matrix;
  }

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing indeterminates.
   * @tparam rows  The number of rows in the new indeterminates.
   * @tparam cols  The number of columns in the new indeterminates.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * std::array<std::string, 2> names = {"x1", "x2"};
   * auto x = prog.NewIndeterminates<2>(names);
   * @endcode
   * This adds a 2 vector indeterminates into the program.
   *
   * The name of the indeterminates is only used for the user in order to ease
   * readability.
   */
  template <int rows>
  VectorIndeterminate<rows> NewIndeterminates(
      const std::array<std::string, rows>& names) {
    return NewIndeterminates<rows, 1>(names);
  }

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing indeterminates.
   * @tparam rows  The number of rows in the new indeterminates.
   * @tparam cols  The number of columns in the new indeterminates.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewIndeterminates<2, 3>("X");
   * @endcode
   * This adds a 2 x 3 matrix indeterminates into the program.
   *
   * The name of the indeterminates is only used for the user in order to ease
   * readability.
   */
  template <int rows, int cols>
  MatrixIndeterminate<rows, cols> NewIndeterminates(
      const std::string& name = "X") {
    std::array<std::string, rows * cols> names;
    for (int j = 0; j < cols; ++j) {
      for (int i = 0; i < rows; ++i) {
        names[j * rows + i] =
            name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
      }
    }
    return NewIndeterminates<rows, cols>(names);
  }

  /**
   * Adds indeterminates to the program.
   * The name for all newly added indeterminates are set to @p name. The default
   * name is "x"
   * @see NewIndeterminates(const std::array<std::string, rows>& names)
   */
  template <int rows>
  VectorIndeterminate<rows> NewIndeterminates(const std::string& name = "x") {
    std::array<std::string, rows> names;
    int offset = (name.compare("x") == 0) ? num_vars() : 0;
    for (int i = 0; i < rows; ++i) {
      names[i] = name + "(" + std::to_string(offset + i) + ")";
    }
    return NewIndeterminates<rows>(names);
  }

  /**
   * Adds indeterminates to this MathematicalProgram.
   * @see NewIndeterminates(int rows, int cols, const
   * std::vector<std::string>& names);
   */
  VectorXIndeterminate NewIndeterminates(int rows,
                                         const std::vector<std::string>& names);

  /**
   * Adds indeterminates to this MathematicalProgram, with default name
   * "x".
   * @see NewIndeterminates(int rows, int cols, const
   * std::vector<std::string>& names);
   */
  VectorXIndeterminate NewIndeterminates(int rows,
                                         const std::string& name = "x");

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing vars.
   * @param rows  The number of rows in the new indeterminates.
   * @param cols  The number of columns in the new indeterminates.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewIndeterminates(2, 3, {"x1", "x2", "x3", "x4",
   * "x5", "x6"});
   * @endcode
   * This adds a 2 x 3 matrix indeterminates into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  MatrixXIndeterminate NewIndeterminates(int rows, int cols,
                                         const std::vector<std::string>& names);

  /**
   * Adds indeterminates to this MathematicalProgram, with default name
   * "X". The new variables are returned and viewed as a matrix, with size
   * @p rows x @p cols.
   * @see NewIndeterminates(int rows, int cols, const
   * std::vector<std::string>& names);
   */
  MatrixXIndeterminate NewIndeterminates(int rows, int cols,
                                         const std::string& name = "X");

  /** Adds indeterminates.
   * This method appends some indeterminates to the end of the program's old
   * indeterminates.
   * @param new_indeterminates The indeterminates to be appended to the
   * program's old indeterminates.
   * @pre `new_indeterminates` should not intersect with the program's old
   * indeterminates or decision variables.
   * @pre Each entry in new_indeterminates should not be dummy.
   * @pre Each entry in new_indeterminates should be of CONTINUOUS type.
   */
  void AddIndeterminates(
      const Eigen::Ref<const VectorXIndeterminate>& new_indeterminates);

  /**
   * Adds a callback method to visualize intermediate results of the
   * optimization.
   *
   * @note Just like other costs/constraints, not all solvers support callbacks.
   * Adding a callback here will force MathematicalProgram::Solve to select a
   * solver that support callbacks.  For instance, adding a visualization
   * callback to a quadratic programming problem may result in using a nonlinear
   * programming solver as the default solver.
   *
   * @param callback a std::function that accepts an Eigen::Vector of doubles
   * representing the bound decision variables.
   * @param vars the decision variables that should be passed to the callback.
   */
  Binding<VisualizationCallback> AddVisualizationCallback(
      const VisualizationCallback::CallbackFunction& callback,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a callback method to visualize intermediate results of the
   * optimization.
   *
   * @note Just like other costs/constraints, not all solvers support callbacks.
   * Adding a callback here will force MathematicalProgram::Solve to select a
   * solver that support callbacks.  For instance, adding a visualization
   * callback to a quadratic programming problem may result in using a nonlinear
   * programming solver as the default solver.
   *
   * @param callback a std::function that accepts an Eigen::Vector of doubles
   * representing the for the bound decision variables.
   * @param vars the decision variables that should be passed to the callback.
   */
  Binding<VisualizationCallback> AddVisualizationCallback(
      const VisualizationCallback::CallbackFunction& callback,
      const VariableRefList& vars) {
    return AddVisualizationCallback(callback,
                                    ConcatenateVariableRefList((vars)));
  }

  /**
   * Adds a generic cost to the optimization program.
   */
  Binding<Cost> AddCost(const Binding<Cost>& binding);

  /**
   * Adds a cost type to the optimization program.
   * @param obj The added objective.
   * @param vars The decision variables on which the cost depend.
   */
  template <typename C>
  auto AddCost(const std::shared_ptr<C>& obj,
               const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    // Redirect to the appropriate type
    // Use auto to enable the overloading method to upcast if needed
    return AddCost(internal::CreateBinding(obj, vars));
  }

  /**
   * Adds a generic cost to the optimization program.
   * @param obj The added objective.
   * @param vars The decision variables on which the cost depend.
   */
  template <typename C>
  auto AddCost(const std::shared_ptr<C>& obj, const VariableRefList& vars) {
    return AddCost(obj, ConcatenateVariableRefList(vars));
  }

  /**
   * Convert an input of type @p F to a FunctionCost object.
   * @tparam F This class should have functions numInputs(), numOutputs and
   * eval(x, y).
   * @see drake::solvers::detail::FunctionTraits.
   */
  template <typename F>
  static std::shared_ptr<Cost> MakeCost(F&& f) {
    return MakeFunctionCost(f);
  }

  /**
   * Adds a cost to the optimization program on a list of variables.
   * @tparam F it should define functions numInputs, numOutputs and eval. Check
   * drake::solvers::detail::FunctionTraits for more detail.
   */
  template <typename F>
  typename std::enable_if<detail::is_cost_functor_candidate<F>::value,
                          Binding<Cost>>::type
  AddCost(F&& f, const VariableRefList& vars) {
    return AddCost(f, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a cost to the optimization program on an Eigen::Vector containing
   * decision variables.
   * @tparam F Type that defines functions numInputs, numOutputs and eval.
   * @see drake::solvers::detail::FunctionTraits.
   */
  template <typename F>
  typename std::enable_if<detail::is_cost_functor_candidate<F>::value,
                          Binding<Cost>>::type
  AddCost(F&& f, const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    auto c = MakeFunctionCost(std::forward<F>(f));
    return AddCost(c, vars);
  }

  /**
   * Statically assert if a user inadvertently passes a
   * binding-compatible Constraint.
   * @tparam F The type to check.
   */
  template <typename F, typename Vars>
  typename std::enable_if<detail::assert_if_is_constraint<F>::value,
                          Binding<Cost>>::type
  AddCost(F&&, Vars&&) {
    throw std::runtime_error("This will assert at compile-time.");
  }

  /**
   * Adds a cost term of the form c'*x.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  Binding<LinearCost> AddCost(const Binding<LinearCost>& binding);

  /**
   * Adds a linear cost term of the form a'*x + b.
   * @param e A linear symbolic expression.
   * @pre e is a linear expression a'*x + b, where each entry of x is a decision
   * variable in the mathematical program.
   * @return The newly added linear constraint, together with the bound
   * variables.
   */
  Binding<LinearCost> AddLinearCost(const symbolic::Expression& e);

  /**
   * Adds a linear cost term of the form a'*x + b.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  Binding<LinearCost> AddLinearCost(const Eigen::Ref<const Eigen::VectorXd>& a,
                                    double b, const VariableRefList& vars) {
    return AddLinearCost(a, b, ConcatenateVariableRefList((vars)));
  }

  /**
   * Adds a linear cost term of the form a'*x + b.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  Binding<LinearCost> AddLinearCost(
      const Eigen::Ref<const Eigen::VectorXd>& a, double b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a linear cost term of the form a'*x.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  template <typename VarType>
  Binding<LinearCost> AddLinearCost(const Eigen::Ref<const Eigen::VectorXd>& a,
                                    const VarType& vars) {
    const double b = 0.;
    return AddLinearCost(a, b, vars);
  }

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x.
   * Applied to subset of the variables and pushes onto
   * the quadratic cost data structure.
   */
  Binding<QuadraticCost> AddCost(const Binding<QuadraticCost>& binding);

  /**
   * Add a quadratic cost term of the form 0.5*x'*Q*x + b'*x + c.
   * Notice that in the optimization program, the constant term `c` in the cost
   * is ignored.
   * @param e A quadratic symbolic expression.
   * @throws std::runtime error if the expression is not quadratic.
   * @return The newly added cost together with the bound variables.
   */
  Binding<QuadraticCost> AddQuadraticCost(const symbolic::Expression& e);

  /**
   * Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).
   */
  Binding<QuadraticCost> AddQuadraticErrorCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& x_desired,
      const VariableRefList& vars) {
    return AddQuadraticErrorCost(Q, x_desired,
                                 ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).
   */
  Binding<QuadraticCost> AddQuadraticErrorCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& x_desired,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a cost term of the form | Ax - b |^2.
   */
  Binding<QuadraticCost> AddL2NormCost(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars) {
    return AddL2NormCost(A, b, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a cost term of the form | Ax - b |^2.
   */
  Binding<QuadraticCost> AddL2NormCost(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddCost(MakeL2NormCost(A, b), vars);
  }

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x.
   * Applied to subset of the variables.
   */
  Binding<QuadraticCost> AddQuadraticCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars) {
    return AddQuadraticCost(Q, b, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x + c
   * Applied to subset of the variables.
   */
  Binding<QuadraticCost> AddQuadraticCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b, double c,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x
   * Applied to subset of the variables.
   */
  Binding<QuadraticCost> AddQuadraticCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a cost term in the polynomial form.
   * @param e A symbolic expression in the polynomial form.
   * @return The newly created cost and the bound variables.
   */
  Binding<PolynomialCost> AddPolynomialCost(const symbolic::Expression& e);

  /**
   * Adds a cost in the symbolic form.
   * Note that the constant part of the cost is ignored. So if you set
   * `e = x + 2`, then only the cost on `x` is added, the constant term 2 is
   * ignored.
   * @param e The linear or quadratic expression of the cost.
   * @pre `e` is linear or `e` is quadratic. Otherwise throws a runtime error.
   * @return The newly created cost, together with the bound variables.
   */
  Binding<Cost> AddCost(const symbolic::Expression& e);

  /**
   * Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   */
  Binding<Constraint> AddConstraint(const Binding<Constraint>& binding);

  /**
   * Adds one row of constraint lb <= e <= ub where @p e is a symbolic
   * expression.
   * @throws std::exception if
   * 1. <tt>lb <= e <= ub</tt> is a trivial constraint such as 1 <= 2 <= 3.
   * 2. <tt>lb <= e <= ub</tt> is unsatisfiable such as 1 <= -5 <= 3
   *
   * @param e A symbolic expression of the the decision variables.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   *
   * The resulting constraint may be a BoundingBoxConstraint, LinearConstraint,
   * LinearEqualityConstraint, or ExpressionConstraint, depending on the
   * arguments.  Constraints of the form x == 1 (which could be created as a
   * BoundingBoxConstraint or LinearEqualityConstraint) will be
   * constructed as a LinearEqualityConstraint.
   */
  Binding<Constraint> AddConstraint(const symbolic::Expression& e, double lb,
                                    double ub);

  /**
   * Adds constraints represented by symbolic expressions to the program. It
   * throws if <tt>lb <= v <= ub</tt> includes trivial/unsatisfiable
   * constraints.
   *
   * @overload Binding<Constraint> AddConstraint(const symbolic::Expression& e,
   *    double lb, double ub)
   */
  Binding<Constraint> AddConstraint(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub);

  /**
   * Add a constraint represented by a symbolic formula to the program. The
   * input formula @p f can be of the following forms:
   *
   * 1. e1 <= e2
   * 2. e1 >= e2
   * 3. e1 == e2
   * 4. A conjunction of relational formulas where each conjunct is
   *    a relational formula matched by 1, 2, or 3.
   *
   * Note that first two cases might return an object of
   * Binding<BoundingBoxConstraint>, Binding<LinearConstraint>, or
   * Binding<ExpressionConstraint>, depending
   * on @p f. Also the third case might return an object of
   * Binding<LinearEqualityConstraint> or Binding<ExpressionConstraint>.
   *
   * It throws an exception if
   *  1. @p f is not matched with one of the above patterns. Especially, strict
   *     inequalities (<, >) are not allowed.
   *  2. @p f is either a trivial constraint such as "1 <= 2" or an
   *     unsatisfiable constraint such as "2 <= 1".
   *  3. It is not possible to find numerical bounds of `e1` and `e2` where @p f
   *     = e1 ??? e2. We allow `e1` and `e2` to be infinite but only if there are
   *     no other terms. For example, `x <= ???` is allowed. However, `x - ??? <= 0`
   *     is not allowed because `x ??? ???` introduces `nan` in the evaluation.
   */
  Binding<Constraint> AddConstraint(const symbolic::Formula& f);

  /**
   * Add a constraint represented by an Eigen::Array<symbolic::Formula>
   * to the program. A common use-case of this function is to add a constraint
   * with the element-wise comparison between two Eigen matrices,
   * using `A.array() <= B.array()`. See the following example.
   *
   * @code
   *   MathematicalProgram prog;
   *   Eigen::Matrix<double, 2, 2> A;
   *   auto x = prog.NewContinuousVariables(2, "x");
   *   Eigen::Vector2d b;
   *   ... // set up A and b
   *   prog.AddConstraint((A * x).array() <= b.array());
   * @endcode
   *
   * A formula in @p formulas can be of the following forms:
   *
   * 1. e1 <= e2
   * 2. e1 >= e2
   * 3. e1 == e2
   *
   * It throws an exception if AddConstraint(const symbolic::Formula& f)
   * throws an exception for f ??? @p formulas.
   *
   * @overload Binding<Constraint> AddConstraint(const symbolic::Formula& f)
   *
   * @tparam Derived An Eigen Array type of Formula.
   */
  template <typename Derived>
  typename std::enable_if<
      is_eigen_scalar_same<Derived, symbolic::Formula>::value,
      Binding<Constraint>>::type
  AddConstraint(const Eigen::ArrayBase<Derived>& formulas) {
    return AddConstraint(internal::ParseConstraint(formulas));
  }

  /**
   * Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   */
  template <typename C>
  auto AddConstraint(std::shared_ptr<C> con, const VariableRefList& vars) {
    return AddConstraint(con, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   */
  template <typename C>
  auto AddConstraint(std::shared_ptr<C> con,
                     const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddConstraint(internal::CreateBinding(con, vars));
  }

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   */
  Binding<LinearConstraint> AddConstraint(
      const Binding<LinearConstraint>& binding);

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const VariableRefList& vars) {
    return AddLinearConstraint(A, lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds one row of linear constraint referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   * lb <= a*vars <= ub
   * @param a A row vector.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   * @param vars The decision variables on which to impose the linear
   * constraint.
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double lb, double ub,
      const VariableRefList& vars) {
    return AddLinearConstraint(a, lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds one row of linear constraint referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   * lb <= a*vars <= ub
   * @param a A row vector.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   * @param vars The decision variables on which to impose the linear
   * constraint.
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double lb, double ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddLinearConstraint(a, Vector1d(lb), Vector1d(ub), vars);
  }

  /**
   * Adds one row of linear constraint lb <= e <= ub where @p e is a symbolic
   * expression.
   * @throws std::exception if
   * 1. @p e is a non-linear expression.
   * 2. <tt>lb <= e <= ub</tt> is a trivial constraint such as 1 <= 2 <= 3.
   * 3. <tt>lb <= e <= ub</tt> is unsatisfiable such as 1 <= -5 <= 3
   *
   * @param e A linear symbolic expression in the form of <tt>c0 + c1 * v1 +
   * ... + cn * vn</tt> where @c c_i is a constant and @v_i is a variable.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   */
  Binding<LinearConstraint> AddLinearConstraint(const symbolic::Expression& e,
                                                double lb, double ub);

  /**
   * Adds linear constraints represented by symbolic expressions to the
   * program. It throws if @v includes a non-linear expression or <tt>lb <= v <=
   * ub</tt> includes trivial/unsatisfiable constraints.
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub);

  /**
   * Add a linear constraint represented by a symbolic formula to the
   * program. The input formula @p f can be of the following forms:
   *
   * 1. e1 <= e2
   * 2. e1 >= e2
   * 3. e1 == e2
   * 4. A conjunction of relational formulas where each conjunct is
   *    a relational formula matched by 1, 2, or 3.
   *
   * Note that first two cases might return an object of
   * Binding<BoundingBoxConstraint> depending on @p f. Also the third case
   * returns an object of Binding<LinearEqualityConstraint>.
   *
   * It throws an exception if
   *  1. @p f is not matched with one of the above patterns. Especially, strict
   *     inequalities (<, >) are not allowed.
   *  2. @p f includes a non-linear expression.
   *  3. @p f is either a trivial constraint such as "1 <= 2" or an
   *     unsatisfiable constraint such as "2 <= 1".
   *  4. It is not possible to find numerical bounds of `e1` and `e2` where @p f
   *     = e1 ??? e2. We allow `e1` and `e2` to be infinite but only if there are
   *     no other terms. For example, `x <= ???` is allowed. However, `x - ??? <= 0`
   *     is not allowed because `x ??? ???` introduces `nan` in the evaluation.
   */
  Binding<LinearConstraint> AddLinearConstraint(const symbolic::Formula& f);

  /**
   * Add a linear constraint represented by an Eigen::Array<symbolic::Formula>
   * to the program. A common use-case of this function is to add a linear
   * constraint with the element-wise comparison between two Eigen matrices,
   * using `A.array() <= B.array()`. See the following example.
   *
   * @code
   *   MathematicalProgram prog;
   *   Eigen::Matrix<double, 2, 2> A;
   *   auto x = prog.NewContinuousVariables(2, "x");
   *   Eigen::Vector2d b;
   *   ... // set up A and b
   *   prog.AddLinearConstraint((A * x).array() <= b.array());
   * @endcode
   *
   * A formula in @p formulas can be of the following forms:
   *
   *  1. e1 <= e2
   *  2. e1 >= e2
   *  3. e1 == e2
   *
   * It throws an exception if AddLinearConstraint(const symbolic::Formula& f)
   * throws an exception for f ??? @p formulas.
   * @tparam Derived An Eigen Array type of Formula.
   */
  template <typename Derived>
  typename std::enable_if<
      is_eigen_scalar_same<Derived, symbolic::Formula>::value,
      Binding<LinearConstraint>>::type
  AddLinearConstraint(const Eigen::ArrayBase<Derived>& formulas) {
    Binding<Constraint> binding = internal::ParseConstraint(formulas);
    Constraint* constraint = binding.evaluator().get();
    if (dynamic_cast<LinearConstraint*>(constraint)) {
      return AddConstraint(
          internal::BindingDynamicCast<LinearConstraint>(binding));
    } else {
      std::stringstream oss;
      oss << "Formulas are non-linear.";
      throw std::runtime_error(
          "AddLinearConstraint called but formulas are non-linear");
    }
  }

  /**
   * Adds linear equality constraints referencing potentially a
   * subset of the decision variables.
   */
  Binding<LinearEqualityConstraint> AddConstraint(
      const Binding<LinearEqualityConstraint>& binding);

  /**
   * Adds one row of linear constraint e = b where @p e is a symbolic
   * expression.
   * @throws std::exception if
   * 1. @p e is a non-linear expression.
   * 2. @p e is a constant.
   *
   * @param e A linear symbolic expression in the form of <tt>c0 + c1 * x1 +
   * ... + cn * xn</tt> where @c c_i is a constant and @x_i is a variable.
   * @param b A scalar.
   * @return The newly added linear equality constraint, together with the
   * bound variable.
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const symbolic::Expression& e, double b);

  /**
   * Adds a linear equality constraint represented by a symbolic formula to the
   * program. The input formula @p f is either an equality formula (`e1 == e2`)
   * or a conjunction of equality formulas.
   *
   * It throws an exception if
   *
   * 1. @p f is neither an equality formula nor a conjunction of equalities.
   * 2. @p f includes a non-linear expression.
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const symbolic::Formula& f);

  /**
   * Adds linear equality constraints \f$ v = b \f$, where \p v(i) is a symbolic
   * linear expression.
   * @throws std::exception if
   * 1. @p v(i) is a non-linear expression.
   * 2. @p v(i) is a constant.
   *
   * @tparam DerivedV An Eigen Matrix type of Expression. A column vector.
   * @tparam DerivedB An Eigen Matrix type of double. A column vector.
   * @param v v(i) is a linear symbolic expression in the form of
   * <tt> c0 + c1 * x1 + ... + cn * xn </tt> where ci is a constant and @xi is
   * a variable.
   * @param b A vector of doubles.
   * @return The newly added linear equality constraint, together with the
   * bound variables.
   */
  template <typename DerivedV, typename DerivedB>
  typename std::enable_if<
      is_eigen_vector_expression_double_pair<DerivedV, DerivedB>::value,
      Binding<LinearEqualityConstraint>>::type
  AddLinearEqualityConstraint(const Eigen::MatrixBase<DerivedV>& v,
                              const Eigen::MatrixBase<DerivedB>& b) {
    return AddConstraint(internal::ParseLinearEqualityConstraint(v, b));
  }

  /**
   * Adds a linear equality constraint for a matrix of linear expression @p V,
   * such that V(i, j) = B(i, j). If V is a symmetric matrix, then the user
   * may only want to constrain the lower triangular part of V.
   * This function is meant to provide convenience to the user, it incurs
   * additional copy and memory allocation. For faster speed, add each column
   * of the matrix equality in a for loop.
   * @tparam DerivedV An Eigen Matrix type of Expression. The number of columns
   * at compile time should not be 1.
   * @tparam DerivedB An Eigen Matrix type of double.
   * @param V An Eigen Matrix of symbolic expressions. V(i, j) should be a
   * linear expression.
   * @param B An Eigen Matrix of doubles.
   * @param lower_triangle If true, then only the lower triangular part of @p V
   * is constrained, otherwise the whole matrix V is constrained. @default is
   * false.
   * @return The newly added linear equality constraint, together with the
   * bound variables.
   */
  template <typename DerivedV, typename DerivedB>
  typename std::enable_if<
      is_eigen_nonvector_expression_double_pair<DerivedV, DerivedB>::value,
      Binding<LinearEqualityConstraint>>::type
  AddLinearEqualityConstraint(const Eigen::MatrixBase<DerivedV>& V,
                              const Eigen::MatrixBase<DerivedB>& B,
                              bool lower_triangle = false) {
    return AddConstraint(
        internal::ParseLinearEqualityConstraint(V, B, lower_triangle));
  }

  /** AddLinearEqualityConstraint
   *
   * Adds linear equality constraints referencing potentially a subset of
   * the decision variables.
   *
   * Example: to add two equality constraints which only depend on two of the
   * elements of x, you could use
   * @code{.cc}
   *   auto x = prog.NewContinuousDecisionVariable(6,"myvar");
   *   Eigen::Matrix2d Aeq;
   *   Aeq << -1, 2,
   *           1, 1;
   *   Eigen::Vector2d beq(1, 3);
   *   prog.AddLinearEqualityConstraint(Aeq, beq, {x.segment<1>(2),
   *                                    x.segment<1>(5)});
   * @endcode
   * The code above imposes constraints
   * @f[-x(2) + 2x(5) = 1 @f]
   * @f[ x(2) +  x(5) = 3 @f]
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& Aeq,
      const Eigen::Ref<const Eigen::VectorXd>& beq,
      const VariableRefList& vars) {
    return AddLinearEqualityConstraint(Aeq, beq,
                                       ConcatenateVariableRefList(vars));
  }

  /** AddLinearEqualityConstraint
   *
   * Adds linear equality constraints referencing potentially a subset of
   * the decision variables.
   *
   * Example: to add two equality constraints which only depend on two of the
   * elements of x, you could use
   * @code{.cc}
   *   auto x = prog.NewContinuousDecisionVariable(6,"myvar");
   *   Eigen::Matrix2d Aeq;
   *   Aeq << -1, 2,
   *           1, 1;
   *   Eigen::Vector2d beq(1, 3);
   *   // Imposes constraint
   *   // -x(0) + 2x(1) = 1
   *   //  x(0) +  x(1) = 3
   *   prog.AddLinearEqualityConstraint(Aeq, beq, x.head<2>());
   * @endcode
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& Aeq,
      const Eigen::Ref<const Eigen::VectorXd>& beq,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds one row of linear equality constraint referencing potentially a subset
   * of decision variables.
   * @f[
   * ax = beq
   * @f]
   * @param a A row vector.
   * @param beq A scalar.
   * @param vars The decision variables on which the constraint is imposed.
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double beq,
      const VariableRefList& vars) {
    return AddConstraint(std::make_shared<LinearEqualityConstraint>(a, beq),
                         ConcatenateVariableRefList(vars));
  }

  /**
   * Adds one row of linear equality constraint referencing potentially a subset
   * of decision variables.
   * @f[
   * ax = beq
   * @f]
   * @param a A row vector.
   * @param beq A scalar.
   * @param vars The decision variables on which the constraint is imposed.
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double beq,
      const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddLinearEqualityConstraint(a, Vector1d(beq), vars);
  }

  /**
   * Adds bounding box constraints referencing potentially a subest of the
   * decision variables.
   * @param binding Binds a BoundingBoxConstraint with some decision variables,
   * such that
   * binding.evaluator()->lower_bound()(i) <= binding.variables()(i)
   *                   <= binding.evaluator().upper_bound()(i)
   */
  Binding<BoundingBoxConstraint> AddConstraint(
      const Binding<BoundingBoxConstraint>& binding);

  /** AddBoundingBoxConstraint
   *
   * Adds bounding box constraints referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   * Example
   * \code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousDecisionVariables<2>("x");
   * auto y = prog.NewContinuousDecisionVariables<1>("y");
   * Eigen::Vector3d lb(0, 1, 2);
   * Eigen::Vector3d ub(1, 2, 3);
   * // Imposes the constraint
   * // 0 ??? x(0) ??? 1
   * // 1 ??? x(1) ??? 2
   * // 2 ??? y    ??? 3
   * prog.AddBoundingBoxConstraint(lb, ub, {x, y});
   * \endcode
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const VariableRefList& vars) {
    return AddBoundingBoxConstraint(lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds bounding box constraints referencing potentially a subset of the
   * decision variables.
   * @param lb The lower bound.
   * @param ub The upper bound.
   * @param vars Will imposes constraint lb(i) <= vars(i) <= ub(i).
   * @return The newly constructed BoundingBoxConstraint.
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds bounds for a single variable.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param var The decision variable.
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      double lb, double ub, const symbolic::Variable& var) {
    MatrixDecisionVariable<1, 1> var_matrix(var);
    return AddBoundingBoxConstraint(Vector1d(lb), Vector1d(ub), var_matrix);
  }

  /**
   * Adds the same scalar lower and upper bound to every variable in the list.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param vars The decision variables.
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      double lb, double ub, const VariableRefList& vars) {
    return AddBoundingBoxConstraint(lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds the same scalar lower and upper bound to every variable in @p vars.
   * @tparam Derived An Eigen Vector type with Variable as the scalar
   * type.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param vars The decision variables.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value &&
          Derived::ColsAtCompileTime == 1,
      Binding<BoundingBoxConstraint>>::type
  AddBoundingBoxConstraint(double lb, double ub,
                           const Eigen::MatrixBase<Derived>& vars) {
    const int kSize = Derived::RowsAtCompileTime;
    return AddBoundingBoxConstraint(
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), lb),
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), ub), vars);
  }

  /**
   * Adds the same scalar lower and upper bound to every variable in @p vars.
   * @tparam Derived An Eigen::Matrix with Variable as the scalar
   * type. The matrix has unknown number of columns at compile time, or has
   * more than one column.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param vars The decision variables.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value &&
          Derived::ColsAtCompileTime != 1,
      Binding<BoundingBoxConstraint>>::type
  AddBoundingBoxConstraint(double lb, double ub,
                           const Eigen::MatrixBase<Derived>& vars) {
    const int kSize =
        Derived::RowsAtCompileTime != Eigen::Dynamic &&
                Derived::ColsAtCompileTime != Eigen::Dynamic
            ? Derived::RowsAtCompileTime * Derived::ColsAtCompileTime
            : Eigen::Dynamic;
    Eigen::Matrix<symbolic::Variable, kSize, 1> flat_vars(vars.size());
    for (int j = 0; j < vars.cols(); ++j) {
      for (int i = 0; i < vars.rows(); ++i) {
        flat_vars(j * vars.rows() + i) = vars(i, j);
      }
    }
    return AddBoundingBoxConstraint(
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), lb),
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), ub), flat_vars);
  }

  /**
   * Adds Lorentz cone constraint referencing potentially a subset
   * of the decision variables.
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)?? + ... + z(n-1)??}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   */
  Binding<LorentzConeConstraint> AddConstraint(
      const Binding<LorentzConeConstraint>& binding);

  /**
   * Adds Lorentz cone constraint referencing potentially a subset of the
   * decision variables.
   * @param v An Eigen::Vector of symbolic::Expression. Constraining that
   * \f[
   * v_0 \ge \sqrt{v_1^2 + ... + v_{n-1}^2}
   * \f]
   * @return The newly constructed Lorentz cone constraint with the bounded
   * variables.
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& v);

  /**
   * Adds Lorentz cone constraint on the linear expression v1 and quadratic
   * expression v2, such that v1 >= sqrt(v2)
   * @param linear_expression The linear expression v1.
   * @param quadratic_expression  The quadratic expression v2.
   * @param tol The tolerance to determine if the matrix in v2 is positive
   * semidefinite or not. @see DecomposePositiveQuadraticForm for more
   * explanation. @default is 0.
   * @retval binding The newly added Lorentz cone constraint, together with the
   * bound variables.
   * @pre
   * 1. `v1` is a linear expression, in the form of c'*x + d.
   * 2. `v2` is a quadratic expression, in the form of
   *    <pre>
   *          x'*Q*x + b'x + a
   *    </pre>
   *    Also the quadratic expression has to be convex, namely Q is a
   *    positive semidefinite matrix, and the quadratic expression needs
   *    to be non-negative for any x.
   * @throws std::runtime_error if the preconditions are not satisfied.
   *
   * Notice this constraint is equivalent to the vector [z;y] is within a
   * Lorentz cone, where
   * <pre>
   *  z = v1
   *  y = R * x + d
   * </pre>
   * while (R, d) satisfies y'*y = x'*Q*x + b'*x + a
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const symbolic::Expression& linear_expression,
      const symbolic::Expression& quadratic_expression, double tol = 0);

  /**
   * Adds Lorentz cone constraint referencing potentially a subset of the
   * decision variables (defined in the vars parameter).
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)?? + ... + z(n-1)??}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   * @param A A @f$\mathbb{R}^{n\times m}@f$ matrix, whose number of columns
   * equals to the size of the decision variables.
   * @param b A @f$\mathbb{R}^n@f$ vector, whose number of rows equals to the
   * size of the decision variables.
   * @param vars The list of @f$ m @f$ decision variables.
   * @return The newly added Lorentz cone constraint.
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars) {
    return AddLorentzConeConstraint(A, b, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds Lorentz cone constraint referencing potentially a subset of the
   * decision variables (defined in the vars parameter).
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)?? + ... + z(n-1)??}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   * @param A A @f$\mathbb{R}^{n\times m}@f$ matrix, whose number of columns
   * equals to the size of the decision variables.
   * @param b A @f$\mathbb{R}^n@f$ vector, whose number of rows equals to the
   * size of the decision variables.
   * @param vars The Eigen vector of @f$ m @f$ decision variables.
   * @return The newly added Lorentz cone constraint.
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Imposes that a vector @f$ x\in\mathbb{R}^m @f$ lies in Lorentz cone. Namely
   * @f[
   * x_0 \ge \sqrt{x_1^2 + .. + x_{m-1}^2}
   * @f]
   * <!-->
   * x(0) >= sqrt(x(1)?? + ... + x(m-1)??)
   * <-->
   * @param vars The stacked column of vars should lie within the Lorentz cone.
   * @return The newly added Lorentz cone constraint.
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const VariableRefList& vars) {
    return AddLorentzConeConstraint(ConcatenateVariableRefList(vars));
  }

  /**
   * Imposes that a vector @f$ x\in\mathbb{R}^m @f$ lies in Lorentz cone. Namely
   * @f[
   * x_0 \ge \sqrt{x_1^2 + .. + x_{m-1}^2}
   * @f]
   * <!-->
   * x(0) >= sqrt(x(1)?? + ... + x(m-1)??)
   * <-->
   * @param vars The stacked column of vars should lie within the Lorentz cone.
   * @return The newly added Lorentz cone constraint.
   */
  template <int rows>
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::MatrixBase<VectorDecisionVariable<rows>>& vars) {
    Eigen::Matrix<double, rows, rows> A(vars.rows(), vars.rows());
    A.setIdentity();
    Eigen::Matrix<double, rows, 1> b(vars.rows());
    b.setZero();
    return AddLorentzConeConstraint(A, b, vars);
  }

  /**
   * Adds a rotated Lorentz cone constraint referencing potentially a subset
   * of decision variables. The linear expression @f$ z=Ax+b @f$ is in rotated
   * Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)?? + ... + z(n-1)??
   * -->
   * @f[
   * z_0z_1 \ge z_2^2 + ... + z_{n-1}^2
   * @f]
   */
  Binding<RotatedLorentzConeConstraint> AddConstraint(
      const Binding<RotatedLorentzConeConstraint>& binding);

  /**
   * Adds rotated Lorentz cone constraint on the linear expression v1, v2 and
   * quadratic expression u, such that v1 * v2 >= u, v1 >= 0, v2 >= 0
   * @param linear_expression1 The linear expression v1.
   * @param linear_expression2 The linear expression v2.
   * @param quadratic_expression The quadratic expression u.
   * @param tol The tolerance to determine if the matrix in v2 is positive
   * semidefinite or not. @see DecomposePositiveQuadraticForm for more
   * explanation. @default is 0.
   * @retval binding The newly added rotated Lorentz cone constraint, together
   * with the bound variables.
   * @pre
   * 1. `linear_expression1` is a linear (affine) expression, in the form of
   *    v1 = c1'*x + d1.
   * 2. `linear_expression2` is a linear (affine) expression, in the form of
   *    v2 = c2'*x + d2.
   * 2. `quadratic_expression` is a quadratic expression, in the form of
   *    <pre>
   *          u = x'*Q*x + b'x + a
   *    </pre>
   *    Also the quadratic expression has to be convex, namely Q is a
   *    positive semidefinite matrix, and the quadratic expression needs
   *    to be non-negative for any x.
   * @throws std::runtime_error if the preconditions are not satisfied.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const symbolic::Expression& linear_expression1,
      const symbolic::Expression& linear_expression2,
      const symbolic::Expression& quadratic_expression, double tol = 0);

  /**
   * Adds a constraint that a symbolic expression @param v is in the rotated
   * Lorentz cone, i.e.,
   * \f[
   * v_0v_1 \ge v_2^2 + ... + v_{n-1}^2\\
   * v_0 \ge 0, v_1 \ge 0
   * \f]
   * @param v A linear expression of variables, \f$ v = A x + b\f$, where \f$ A,
   * b \f$ are given matrices of the correct size, \f$ x \f$ is the vector of
   * decision variables.
   * @retval binding The newly added rotated Lorentz cone constraint, together
   * with the bound variables.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& v);

  /**
   * Adds a rotated Lorentz cone constraint referencing potentially a subset
   * of decision variables, The linear expression @f$ z=Ax+b @f$ is in rotated
   * Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)?? + ... + z(n-1)??
   * -->
   * @f[
   * z_0z_1 \ge z_2^2 + ... + z_{n-1}^2
   * @f]
   * where @f$ A\in\mathbb{R}^{n\times m}, b\in\mathbb{R}^n@f$ are given
   * matrices.
   * @param A A matrix whose number of columns equals to the size of the
   * decision variables.
   * @param b A vector whose number of rows equals to the size fo the decision
   * variables.
   * @param vars The decision variables on which the constraint is imposed.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars) {
    return AddRotatedLorentzConeConstraint(A, b,
                                           ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a rotated Lorentz cone constraint referencing potentially a subset
   * of decision variables, The linear expression @f$ z=Ax+b @f$ is in rotated
   * Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)?? + ... + z(n-1)??
   * -->
   * @f[
   * z_0z_1 \ge z_2^2 + ... + z_{n-1}^2
   * @f]
   * where @f$ A\in\mathbb{R}^{n\times m}, b\in\mathbb{R}^n@f$ are given
   * matrices.
   * @param A A matrix whose number of columns equals to the size of the
   * decision variables.
   * @param b A vector whose number of rows equals to the size fo the decision
   * variables.
   * @param vars The decision variables on which the constraint is imposed.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Impose that a vector @f$ x\in\mathbb{R}^m @f$ is in rotated Lorentz cone.
   * Namely
   * @f[
   * x_0 x_1 \ge x_2^2 + ... + x_{m-1}^2\\
   * x_0 \ge 0, x_1 \ge 0
   * @f]
   * <!-->
   * x(0)*x(1) >= x(2)^2 + ... x(m-1)^2
   * x(0) >= 0, x(1) >= 0
   * <-->
   * @param vars The stacked column of vars lies in the rotated Lorentz cone.
   * @return The newly added rotated Lorentz cone constraint.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const VariableRefList& vars) {
    return AddRotatedLorentzConeConstraint(ConcatenateVariableRefList(vars));
  }

  /**
   * Impose that a vector @f$ x\in\mathbb{R}^m @f$ is in rotated Lorentz cone.
   * Namely
   * @f[
   * x_0 x_1 \ge x_2^2 + ... + x_{m-1}^2\\
   * x_0 \ge 0, x_1 \ge 0
   * @f]
   * <!-->
   * x(0)*x(1) >= x(2)^2 + ... x(m-1)^2
   * x(0) >= 0, x(1) >= 0
   * <-->
   * @param vars The stacked column of vars lies in the rotated Lorentz cone.
   * @return The newly added rotated Lorentz cone constraint.
   */
  template <int rows>
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::MatrixBase<VectorDecisionVariable<rows>>& vars) {
    Eigen::Matrix<double, rows, rows> A(vars.rows(), vars.rows());
    A.setIdentity();
    Eigen::Matrix<double, rows, 1> b(vars.rows());
    b.setZero();
    return AddRotatedLorentzConeConstraint(A, b, vars);
  }

  /**
   * Adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   */
  Binding<LinearComplementarityConstraint> AddConstraint(
      const Binding<LinearComplementarityConstraint>& binding);

  /**
   * Adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   */
  Binding<LinearComplementarityConstraint> AddLinearComplementarityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& M,
      const Eigen::Ref<const Eigen::VectorXd>& q, const VariableRefList& vars) {
    return AddLinearComplementarityConstraint(M, q,
                                              ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   */
  Binding<LinearComplementarityConstraint> AddLinearComplementarityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& M,
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a polynomial constraint to the program referencing a subset
   * of the decision variables (defined in the vars parameter).
   */
  Binding<Constraint> AddPolynomialConstraint(
      const VectorXPoly& polynomials,
      const std::vector<Polynomiald::VarType>& poly_vars,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const VariableRefList& vars) {
    return AddPolynomialConstraint(polynomials, poly_vars, lb, ub,
                                   ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a polynomial constraint to the program referencing a subset
   * of the decision variables (defined in the vars parameter).
   */
  Binding<Constraint> AddPolynomialConstraint(
      const VectorXPoly& polynomials,
      const std::vector<Polynomiald::VarType>& poly_vars,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   */
  Binding<PositiveSemidefiniteConstraint> AddConstraint(
      const Binding<PositiveSemidefiniteConstraint>& binding);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   */
  Binding<PositiveSemidefiniteConstraint> AddConstraint(
      std::shared_ptr<PositiveSemidefiniteConstraint> con,
      const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   *
   * @throws std::runtime_error in Debug mode if @p symmetric_matrix_var is not
   * symmetric.
   * @param symmetric_matrix_var A symmetric MatrixDecisionVariable object.
   */
  Binding<PositiveSemidefiniteConstraint> AddPositiveSemidefiniteConstraint(
      const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix of symbolic
   * expressions @p e. We create a new symmetric matrix of variables M being
   * positive semidefinite, with the linear equality constraint e == M.
   * @tparam Derived An Eigen Matrix of symbolic expressions.
   * @param e Imposes constraint "e is positive semidefinite".
   * @pre{1. e is symmetric.
   *      2. e(i, j) is linear for all i, j
   *      }
   * @return The newly added positive semidefinite constraint, with the bound
   * variable M that are also newly added.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Expression>::value,
      Binding<PositiveSemidefiniteConstraint>>::type
  AddPositiveSemidefiniteConstraint(const Eigen::MatrixBase<Derived>& e) {
    DRAKE_DEMAND(e.rows() == e.cols());
    DRAKE_ASSERT(e == e.transpose());
    const int e_rows = Derived::RowsAtCompileTime;
    MatrixDecisionVariable<e_rows, e_rows> M{};
    if (e_rows == Eigen::Dynamic) {
      M = NewSymmetricContinuousVariables(e.rows());
    } else {
      M = NewSymmetricContinuousVariables<e_rows>();
    }
    // Adds the linear equality constraint that M = e.
    AddLinearEqualityConstraint(
        e - M, Eigen::Matrix<double, e_rows, e_rows>::Zero(e.rows(), e.rows()),
        true);
    const int M_flat_size =
        e_rows == Eigen::Dynamic ? Eigen::Dynamic : e_rows * e_rows;
    const Eigen::Map<Eigen::Matrix<symbolic::Variable, M_flat_size, 1>> M_flat(
        &M(0, 0), e.size());
    return AddPositiveSemidefiniteConstraint(M);
  }

  /**
   * Adds a linear matrix inequality constraint to the program.
   */
  Binding<LinearMatrixInequalityConstraint> AddConstraint(
      const Binding<LinearMatrixInequalityConstraint>& binding);

  /**
   * Adds a linear matrix inequality constraint to the program.
   */
  Binding<LinearMatrixInequalityConstraint> AddLinearMatrixInequalityConstraint(
      const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
      const VariableRefList& vars) {
    return AddLinearMatrixInequalityConstraint(
        F, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a linear matrix inequality constraint to the program.
   */
  Binding<LinearMatrixInequalityConstraint> AddLinearMatrixInequalityConstraint(
      const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds the constraint that a symmetric matrix is diagonally dominant with
   * non-negative diagonal entries.
   * A symmetric matrix X is diagonally dominant with non-negative diagonal
   * entries if
   * X(i, i) >= ?????? |X(i, j)| ??? j ??? i
   * namely in each row, the diagonal entry is larger than the sum of the
   * absolute values of all other entries in the same row. A matrix being
   * diagonally dominant with non-negative diagonals is a sufficient (but not
   * necessary) condition of a matrix being positive semidefinite.
   * Internally we will create a matrix Y as slack variables, such that Y(i, j)
   * represents the absolute value |X(i, j)| ??? j ??? i. The diagonal entries
   * Y(i, i) = X(i, i)
   * The users can refer to "DSOS and SDSOS Optimization: More Tractable
   * Alternatives to Sum of Squares and Semidefinite Optimization" by Amir Ali
   * Ahmadi and Anirudha Majumdar, with arXiv link
   * https://arxiv.org/abs/1706.02586
   * @param X The matrix X. We will use 0.5(X+X???) as the "symmetric version" of
   * X.
   * @return Y The slack variable. Y(i, j) represents |X(i, j)| ??? j ??? i, with
   * the constraint Y(i, j) >= X(i, j) and Y(i, j) >= -X(i, j). Y is a symmetric
   * matrix. The diagonal entries Y(i, i) = X(i, i)
   */
  MatrixX<symbolic::Expression> AddPositiveDiagonallyDominantMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X);

  /**
   * Adds the constraint that a symmetric matrix is scaled diagonally dominant
   * (sdd). A matrix X is sdd if there exists a diagonal matrix D, such that
   * the product DXD is diagonally dominant with non-negative diagonal entries,
   * namely
   * d(i)X(i, i) ??? ?????? |d(j)X(i, j)| ??? j ??? i
   * where d(i) = D(i, i).
   * X being sdd is equivalent to the existence of symmetric matrices M???????? ???????????
   * i < j, such that all entries in M????? are 0, except M?????(i, i), M?????(i, j),
   * M?????(j, j). (M?????(i, i), M?????(j, j), M?????(i, j)) is in the rotated
   * Lorentz cone, and X = ????????? M?????
   * The users can refer to "DSOS and SDSOS Optimization: More Tractable
   * Alternatives to Sum of Squares and Semidefinite Optimization" by Amir Ali
   * Ahmadi and Anirudha Majumdar, with arXiv link
   * https://arxiv.org/abs/1706.02586.
   * @param X The matrix X. We will use 0.5(X+X???) as the "symmetric version" of
   * X.
   * @pre X(i, j) should be a linear expression of decision variables.
   * @return M A vector of vectors of 2 x 2 symmetric matrices M. For i < j,
   * M[i][j] is
   * <pre>
   * [M?????(i, i), M?????(i, j)]
   * [M?????(i, j), M?????(j, j)].
   * </pre>
   * Note that M[i][j](0, 1) = M?????(i, j) = (X(i, j) + X(j, i)) / 2
   * for i >= j, M[i][j] is the zero matrix.
   */
  std::vector<std::vector<Matrix2<symbolic::Expression>>>
  AddScaledDiagonallyDominantMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X);

  /**
   * Adds constraints that a given polynomial @p p is a sums-of-squares (SOS),
   * that is, @p p can be decomposed into `m???Qm`, where m is the @p
   * monomial_basis. It returns a pair of constraint bindings expressing:
   *
   *  - The coefficients matrix Q is PSD (positive semidefinite).
   *  - The coefficients matching conditions in linear equality constraint.
   */
  std::pair<Binding<PositiveSemidefiniteConstraint>,
            Binding<LinearEqualityConstraint>>
  AddSosConstraint(
      const symbolic::Polynomial& p,
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis);

  /**
   * Adds constraints that a given polynomial @p p is a sums-of-squares (SOS),
   * that is, @p p can be decomposed into `m???Qm`, where m is the monomial
   * basis of all indeterminates in the program with degree equal to half the
   * TotalDegree of @p p. It returns a pair of constraint bindings expressing:
   *
   *  - The coefficients matrix Q is PSD (positive semidefinite).
   *  - The coefficients matching conditions in linear equality constraint.
   */
  std::pair<Binding<PositiveSemidefiniteConstraint>,
            Binding<LinearEqualityConstraint>>
  AddSosConstraint(const symbolic::Polynomial& p);

  /**
   * Adds constraints that a given symbolic expression @p e is a
   * sums-of-squares (SOS), that is, @p p can be decomposed into `m???Qm`,
   * where m is the @p monomial_basis.  Note that it decomposes @p e into a
   * polynomial with respect to `indeterminates()` in this mathematical
   * program. It returns a pair of constraint bindings expressing:
   *
   *  - The coefficients matrix Q is PSD (positive semidefinite).
   *  - The coefficients matching conditions in linear equality constraint.
   */
  std::pair<Binding<PositiveSemidefiniteConstraint>,
            Binding<LinearEqualityConstraint>>
  AddSosConstraint(
      const symbolic::Expression& e,
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis);

  /**
   * Adds constraints that a given symbolic expression @p e is a sums-of-squares
   * (SOS), that is, @p e can be decomposed into `mTQm`. Note that it decomposes
   * @p e into a polynomial with respect to `indeterminates()` in this
   * mathematical program. It returns a pair of
   * constraint bindings expressing:
   *
   *  - The coefficients matrix Q is PSD (positive semidefinite).
   *  - The coefficients matching conditions in linear equality constraint.
   */
  std::pair<Binding<PositiveSemidefiniteConstraint>,
            Binding<LinearEqualityConstraint>>
  AddSosConstraint(const symbolic::Expression& e);

  // template <typename FunctionType>
  // void AddCost(std::function..);
  // void AddLinearCost(const Eigen::MatrixBase<Derived>& c, const vector<const
  // DecisionVariable&>& vars)
  // void addQuadraticCost ...

  /**
   * Gets the initial guess for a single variable.
   * @pre @p decision_variable has been registered in the optimization program.
   * @throws std::runtime_error if the pre condition is not satisfied.
   */
  double GetInitialGuess(const symbolic::Variable& decision_variable) const;

  /**
   * Gets the initial guess for some variables.
   * @pre Each variable in @p decision_variable_mat has been registered in the
   * optimization program.
   * @throws std::runtime_error if the pre condition is not satisfied.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
      Eigen::Matrix<double, Derived::RowsAtCompileTime,
                    Derived::ColsAtCompileTime>>::type
  GetInitialGuess(
      const Eigen::MatrixBase<Derived>& decision_variable_mat) const {
    Eigen::Matrix<double, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime>
        decision_variable_values(decision_variable_mat.rows(),
                                 decision_variable_mat.cols());
    for (int i = 0; i < decision_variable_mat.rows(); ++i) {
      for (int j = 0; j < decision_variable_mat.cols(); ++j) {
        decision_variable_values(i, j) =
            GetInitialGuess(decision_variable_mat(i, j));
      }
    }
    return decision_variable_values;
  }

  /**
   * Sets the initial guess for a single variable @p decision_variable.
   * @pre decision_variable is a registered decision variable in the program.
   * @throws std::runtime_error if precondition is not satisfied.
   */
  void SetInitialGuess(const symbolic::Variable& decision_variable,
                       double variable_guess_value);

  /**
   * Sets the initial guess for the decision variables stored in
   * @p decision_variable_mat to be @p x0. Variables begin with a default
   * initial guess of NaN to indicate that no guess is available.
   */
  template <typename DerivedA, typename DerivedB>
  void SetInitialGuess(const Eigen::MatrixBase<DerivedA>& decision_variable_mat,
                       const Eigen::MatrixBase<DerivedB>& x0) {
    DRAKE_ASSERT(decision_variable_mat.rows() == x0.rows());
    DRAKE_ASSERT(decision_variable_mat.cols() == x0.cols());
    for (int i = 0; i < decision_variable_mat.rows(); ++i) {
      for (int j = 0; j < decision_variable_mat.cols(); ++j) {
        SetInitialGuess(decision_variable_mat(i, j), x0(i, j));
      }
    }
  }

  /**
   * Set the initial guess for ALL decision variables.
   * Note that variables begin with a default initial guess of NaN to indicate
   * that no guess is available.
   * @param x0 A vector of appropriate size (num_vars() x 1).
   */
  template <typename Derived>
  void SetInitialGuessForAllVariables(const Eigen::MatrixBase<Derived>& x0) {
    DRAKE_ASSERT(x0.rows() == num_vars() && x0.cols() == 1);
    x_initial_guess_ = x0;
  }

  /**
   * Solve the MathematicalProgram.
   *
   * @return SolutionResult indicating if the solution was successful.
   */
  SolutionResult Solve();
  // TODO(naveenoid) : add argument for options

  //    template <typename Derived>
  //    bool solve(const Eigen::MatrixBase<Derived>& x0);

  //    getCostValue();
  //    getExitFlag();
  //    getInfeasibleConstraintNames();

  void PrintSolution() {
    for (int i = 0; i < num_vars(); ++i) {
      std::cout << decision_variables_(i).get_name() << " = "
                << GetSolution(decision_variables_(i)) << std::endl;
    }
  }

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option, double option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option, int option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option,
                       const std::string& option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  /**
   * Returns the solver options stored inside MathematicalProgram.
   */
  const SolverOptions& solver_options() const { return solver_options_; }

  const std::unordered_map<std::string, double>& GetSolverOptionsDouble(
      const SolverId& solver_id) const {
    return solver_options_.GetOptionsDouble(solver_id);
  }

  const std::unordered_map<std::string, int>& GetSolverOptionsInt(
      const SolverId& solver_id) const {
    return solver_options_.GetOptionsInt(solver_id);
  }

  const std::unordered_map<std::string, std::string>& GetSolverOptionsStr(
      const SolverId& solver_id) const {
    return solver_options_.GetOptionsStr(solver_id);
  }

  /**
   * Returns the ID of the solver that was used to solve this program.
   * Returns empty if Solve() has not been called.
   */
  optional<SolverId> GetSolverId() const { return solver_id_; }

  /**
   * Getter for optimal cost at the solution.
   * If the solver finds an optimal solution, then we return the cost evaluated
   * at this solution.
   * If the program is unbounded, then the optimal cost is -???.
   * If the program is globally infeasible, then the optimal cost is +???.
   * If the program is locally infeasible, then the solver (e.g. SNOPT) might
   * return some finite value as the optimal cost.
   * Otherwise, the optimal cost is NaN.
   */
  double GetOptimalCost() const { return optimal_cost_; }

  /**
   * Getter for lower bound on optimal cost. Defaults to -Infinity
   * if a lower bound has not been found.
   */
  double GetLowerBoundCost() const { return lower_bound_cost_; }

  /**
   * Getter for all callbacks.
   */
  const std::vector<Binding<VisualizationCallback>>& visualization_callbacks()
      const {
    return visualization_callbacks_;
  }

  /**
   * Getter for all generic costs.
   */
  const std::vector<Binding<Cost>>& generic_costs() const {
    return generic_costs_;
  }  // e.g. for snopt_user_fun

  /**
   * Getter for all generic constraints
   */
  const std::vector<Binding<Constraint>>& generic_constraints() const {
    return generic_constraints_;
  }  // e.g. for snopt_user_fun

  /**
   * Getter for linear equality constraints.
   */
  const std::vector<Binding<LinearEqualityConstraint>>&
  linear_equality_constraints() const {
    return linear_equality_constraints_;
  }

  /** Getter for linear costs. */
  const std::vector<Binding<LinearCost>>& linear_costs() const {
    return linear_costs_;
  }

  /** Getter for quadratic costs. */
  const std::vector<Binding<QuadraticCost>>& quadratic_costs() const {
    return quadratic_costs_;
  }

  /** Getter for linear constraints. */
  const std::vector<Binding<LinearConstraint>>& linear_constraints() const {
    return linear_constraints_;
  }

  /** Getter for Lorentz cone constraint */
  const std::vector<Binding<LorentzConeConstraint>>& lorentz_cone_constraints()
      const {
    return lorentz_cone_constraint_;
  }

  /** Getter for rotated Lorentz cone constraint */
  const std::vector<Binding<RotatedLorentzConeConstraint>>&
  rotated_lorentz_cone_constraints() const {
    return rotated_lorentz_cone_constraint_;
  }

  /** Getter for positive semidefinite constraint */
  const std::vector<Binding<PositiveSemidefiniteConstraint>>&
  positive_semidefinite_constraints() const {
    return positive_semidefinite_constraint_;
  }

  /** Getter for linear matrix inequality constraint */
  const std::vector<Binding<LinearMatrixInequalityConstraint>>&
  linear_matrix_inequality_constraints() const {
    return linear_matrix_inequality_constraint_;
  }

  /**
   * Getter returning all costs (for now linear costs appended to
   * generic costs, then quadratic costs appended to
   * generic costs).
   */
  std::vector<Binding<Cost>> GetAllCosts() const {
    auto costlist = generic_costs_;
    costlist.insert(costlist.end(), linear_costs_.begin(), linear_costs_.end());
    costlist.insert(costlist.end(), quadratic_costs_.begin(),
                    quadratic_costs_.end());
    return costlist;
  }

  /**
   * Getter returning all linear constraints (both linear equality and
   * inequality constraints).
   */
  std::vector<Binding<LinearConstraint>> GetAllLinearConstraints() const {
    std::vector<Binding<LinearConstraint>> conlist = linear_constraints_;
    conlist.insert(conlist.end(), linear_equality_constraints_.begin(),
                   linear_equality_constraints_.end());
    return conlist;
  }

  /** Getter for all bounding box constraints */
  const std::vector<Binding<BoundingBoxConstraint>>& bounding_box_constraints()
      const {
    return bbox_constraints_;
  }

  /** Getter for all linear complementarity constraints.*/
  const std::vector<Binding<LinearComplementarityConstraint>>&
  linear_complementarity_constraints() const {
    return linear_complementarity_constraints_;
  }

  // Base class for solver-specific data.  A solver implementation may derive
  // a helper class from this for use with getSolverData.
  struct SolverData {
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SolverData)
    SolverData() = default;
    virtual ~SolverData() = default;
  };

  // Call from solver implementations to get a persistently-stored
  // helper structure of type T (derived from SolverData).  If no
  // data of type T is already stored then a new one will be created
  // and stored, replacing data from any other solver in this problem
  // instance.
  template <typename T>
  std::shared_ptr<T> GetSolverData() {
    auto p = std::dynamic_pointer_cast<T>(solver_data_);
    if (!p) {
      p = std::make_shared<T>();
      solver_data_ = p;
    }
    return p;
  }

  /** Getter for number of variables in the optimization program */
  int num_vars() const { return decision_variables_.rows(); }

  /** Getter for the initial guess */
  const Eigen::VectorXd& initial_guess() const { return x_initial_guess_; }

  /** Returns the index of the decision variable. Internally the solvers thinks
   * all variables are stored in an array, and it accesses each individual
   * variable using its index. This index is used when adding constraints
   * and costs for each solver.
   * @pre{@p var is a decision variable in the mathematical program, otherwise
   * this function throws a runtime error.}
   */
  int FindDecisionVariableIndex(const symbolic::Variable& var) const;

  /**
   * Returns the indices of the decision variables. Internally the solvers
   * thinks all variables are stored in an array, and it accesses each
   * individual
   * variable using its index. This index is used when adding constraints
   * and costs for each solver.
   * @pre{@p vars are decision variables in the mathematical program, otherwise
   * this function throws a runtime error.}
   */
  std::vector<int> FindDecisionVariableIndices(
      const Eigen::Ref<const VectorXDecisionVariable>& vars) const;

  /** Gets the number of indeterminates in the optimization program */
  int num_indeterminates() const { return indeterminates_.rows(); }

  /** Returns the index of the indeterminate. Internally a solver
   * thinks all indeterminates are stored in an array, and it accesses each
   * individual indeterminate using its index. This index is used when adding
   * constraints and costs for each solver.
   * @pre @p var is a indeterminate in the mathematical program,
   * otherwise this function throws a runtime error.
   */
  size_t FindIndeterminateIndex(const symbolic::Variable& var) const;

  /**
   * Gets the solution of an Eigen matrix of decision variables.
   * @tparam Derived An Eigen matrix containing Variable.
   * @param var The decision variables.
   * @return The value of the decision variable after solving the problem.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
      Eigen::Matrix<double, Derived::RowsAtCompileTime,
                    Derived::ColsAtCompileTime>>::type
  GetSolution(const Eigen::MatrixBase<Derived>& var) const {
    Eigen::Matrix<double, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime>
        value(var.rows(), var.cols());
    for (int i = 0; i < var.rows(); ++i) {
      for (int j = 0; j < var.cols(); ++j) {
        value(i, j) = GetSolution(var(i, j));
      }
    }
    return value;
  }

  /**
   * Gets the value of a single decision variable.
   */
  double GetSolution(const symbolic::Variable& var) const;

  /**
   * Gets the solution of an Eigen matrix of decision variables.
   * @tparam Derived An Eigen matrix containing Variable.
   * @param var The decision variables.
   * @param result The result returned from the solver. @note This function
   * doesn't use the decision variable values stored inside
   * solvers::MathematicalProgram.
   * @return The value of the decision variable after solving the problem.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
      Eigen::Matrix<double, Derived::RowsAtCompileTime,
                    Derived::ColsAtCompileTime>>::type
  GetSolution(const Eigen::MatrixBase<Derived>& var,
              const MathematicalProgramResult& result) const {
    Eigen::Matrix<double, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime>
        value(var.rows(), var.cols());
    for (int i = 0; i < var.rows(); ++i) {
      for (int j = 0; j < var.cols(); ++j) {
        value(i, j) = GetSolution(var(i, j), result);
      }
    }
    return value;
  }

  /**
   * Gets the value of a single decision variable.
   * @param var The symbolic variable as a decision variable of the program.
   * @param result The result returned from calling the solver.
   * @throws std::invalid_argument if result.get_x_vals().rows() !=
   * num_vars().
   */
  double GetSolution(const symbolic::Variable& var,
                     const MathematicalProgramResult& result) const;

  /**
   * Replaces the variables in an expression with the solutions to the
   * variables, returns the expression after substitution.
   * @throws std::runtime_error if some variables in the expression @p e are NOT
   * decision variables or indeterminates in the optimization program.
   * @note If the expression @p e contains both decision variables and
   * indeterminates of the optimization program, then the decision variables
   * will be substituted by its solutions in double values, but not the
   * indeterminates.
   */
  symbolic::Expression SubstituteSolution(const symbolic::Expression& e) const;

  /**
   * Replaces the decision variables in a polynomial with the solutions to the
   * variables, returns the polynomial after substitution.
   * @throws std::runtime_error if some decision variables in the polynomial
   * @p p are NOT decision variables in the optimization program.
   * @note If the polynomial @p p contains both decision variables and
   * indeterminates of the optimization program, then the decision variables
   * will be substituted by its solutions in double values, but not the
   * indeterminates.
   */
  symbolic::Polynomial SubstituteSolution(const symbolic::Polynomial& p) const;

  /**
   * Evaluates the value of some binding, for some input value for all
   * decision variables.
   * @param binding A Binding whose variables are decision variables in this
   * program.
   * @param prog_var_vals The value of all the decision variables in this
   * program.
   * @throws std::logic_error if the size does not match.
   */
  template <typename C, typename DerivedX>
  typename std::enable_if<is_eigen_vector<DerivedX>::value,
                          VectorX<typename DerivedX::Scalar>>::type
  EvalBinding(const Binding<C>& binding,
              const Eigen::MatrixBase<DerivedX>& prog_var_vals) const {
    using Scalar = typename DerivedX::Scalar;
    if (prog_var_vals.rows() != num_vars()) {
      std::ostringstream oss;
      oss << "The input binding variable is not in the right size. Expects "
          << num_vars() << " rows, but it actually has " << prog_var_vals.rows()
          << " rows.\n";
      throw std::logic_error(oss.str());
    }
    VectorX<Scalar> binding_x(binding.GetNumElements());
    VectorX<Scalar> binding_y(binding.evaluator()->num_outputs());
    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      binding_x(i) =
          prog_var_vals(FindDecisionVariableIndex(binding.variables()(i)));
    }
    binding.evaluator()->Eval(binding_x, &binding_y);
    return binding_y;
  }

  /** Evaluates all visualization callbacks registered with the
   * MathematicalProgram.
   *
   * @param prog_var_vals The value of all the decision variables in this
   * program.
   * @throws std::logic_error if the size does not match.
   */
  void EvalVisualizationCallbacks(
      const Eigen::Ref<const Eigen::VectorXd>& prog_var_vals) const {
    if (prog_var_vals.rows() != num_vars()) {
      std::ostringstream oss;
      oss << "The input binding variable is not in the right size. Expects "
          << num_vars() << " rows, but it actually has " << prog_var_vals.rows()
          << " rows.\n";
      throw std::logic_error(oss.str());
    }

    Eigen::VectorXd this_x;

    for (auto const& binding : visualization_callbacks_) {
      auto const& obj = binding.evaluator();

      const int num_v_variables = binding.GetNumElements();
      this_x.resize(num_v_variables);
      for (int j = 0; j < num_v_variables; ++j) {
        this_x(j) =
            prog_var_vals(FindDecisionVariableIndex(binding.variables()(j)));
      }

      obj->EvalCallback(this_x);
    }
  }

  /**
   * Evaluates the evaluator in @p binding at the solution value.
   * @return The value of @p binding at the solution value.
   */
  template <typename C>
  Eigen::VectorXd EvalBindingAtSolution(const Binding<C>& binding) const {
    return EvalBinding(binding, x_values_);
  }

  /**
   * Evaluates the evaluator in @p binding at the initial guess.
   * @return The value of @p binding at the initial guess.
   */
  template <typename C>
  Eigen::VectorXd EvalBindingAtInitialGuess(const Binding<C>& binding) const {
    return EvalBinding(binding, x_initial_guess_);
  }

  /** Getter for all decision variables in the program. */
  const VectorXDecisionVariable& decision_variables() const {
    return decision_variables_;
  }

  /** Getter for the decision variable with index @p i in the program. */
  const symbolic::Variable& decision_variable(int i) const {
    return decision_variables_(i);
  }

  /** Getter for all indeterminates in the program. */
  const VectorXIndeterminate& indeterminates() const { return indeterminates_; }

  /** Getter for the indeterminate with index @p i in the program. */
  const symbolic::Variable& indeterminate(int i) const {
    return indeterminates_(i);
  }

  /**
   * Solver reports its result back to MathematicalProgram, by passing the
   * solver_result, which contains the solver result.
   * @note This method should only be called by each solver, after it solves the
   * optimization problem stored in MathematicalProgram. The user should NOT
   * call this method.
   */
  // This method should be called by the derived classes of
  // MathematicalProgramSolverInterface, which is not a friend class of
  // MathematicalProgram, as we do not want to leak any of the internal details
  // of MathematicalProgram.
  void SetSolverResult(const SolverResult& solver_result);

  /// Getter for the required capability on the solver, given the
  /// cost/constraint/variable types in the program.
  const ProgramAttributes& required_capabilities() const {
    return required_capabilities_;
  }

 private:
  static void AppendNanToEnd(int new_var_size, Eigen::VectorXd* vector);
  // maps the ID of a symbolic variable to the index of the variable stored in
  // the optimization program.
  std::unordered_map<symbolic::Variable::Id, int> decision_variable_index_{};

  VectorXDecisionVariable decision_variables_;

  std::unordered_map<symbolic::Variable::Id, int> indeterminates_index_;
  VectorXIndeterminate indeterminates_;

  std::vector<Binding<VisualizationCallback>> visualization_callbacks_;

  std::vector<Binding<Cost>> generic_costs_;
  std::vector<Binding<QuadraticCost>> quadratic_costs_;
  std::vector<Binding<LinearCost>> linear_costs_;
  // TODO(naveenoid) : quadratic_constraints_

  // note: linear_constraints_ does not include linear_equality_constraints_
  std::vector<Binding<Constraint>> generic_constraints_;
  std::vector<Binding<LinearConstraint>> linear_constraints_;
  std::vector<Binding<LinearEqualityConstraint>> linear_equality_constraints_;
  std::vector<Binding<BoundingBoxConstraint>> bbox_constraints_;
  std::vector<Binding<LorentzConeConstraint>> lorentz_cone_constraint_;
  std::vector<Binding<RotatedLorentzConeConstraint>>
      rotated_lorentz_cone_constraint_;
  std::vector<Binding<PositiveSemidefiniteConstraint>>
      positive_semidefinite_constraint_;
  std::vector<Binding<LinearMatrixInequalityConstraint>>
      linear_matrix_inequality_constraint_;

  // Invariant:  The bindings in this list must be non-overlapping, when calling
  // Linear Complementarity solver like Moby. If this constraint is solved
  // through a nonlinear optimization solver (like SNOPT) instead, then we allow
  // the bindings to be overlapping.
  // TODO(ggould-tri) can this constraint be relaxed?
  std::vector<Binding<LinearComplementarityConstraint>>
      linear_complementarity_constraints_;

  Eigen::VectorXd x_initial_guess_;
  Eigen::VectorXd x_values_;
  std::shared_ptr<SolverData> solver_data_;
  optional<SolverId> solver_id_;
  double optimal_cost_{};
  // The lower bound of the objective found by the solver, during the
  // optimization process.
  double lower_bound_cost_{};

  // The actual per-solver customization options.
  SolverOptions solver_options_;

  ProgramAttributes required_capabilities_{};

  std::unique_ptr<MathematicalProgramSolverInterface> ipopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> nlopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> snopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> moby_lcp_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> linear_system_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface>
      equality_constrained_qp_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> gurobi_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> mosek_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> osqp_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> scs_solver_;

  template <typename T>
  void NewVariables_impl(
      VarType type, const T& names, bool is_symmetric,
      Eigen::Ref<MatrixXDecisionVariable> decision_variable_matrix) {
    switch (type) {
      case VarType::CONTINUOUS:
        break;
      case VarType::BINARY:
        required_capabilities_.insert(ProgramAttribute::kBinaryVariable);
        break;
      case VarType::INTEGER:
        throw std::runtime_error(
            "MathematicalProgram does not support integer variables yet.");
      case VarType::BOOLEAN:
        throw std::runtime_error(
            "MathematicalProgram does not support Boolean variables.");
    }
    int rows = decision_variable_matrix.rows();
    int cols = decision_variable_matrix.cols();
    DRAKE_ASSERT(!is_symmetric || rows == cols);
    int num_new_vars = 0;
    if (!is_symmetric) {
      num_new_vars = rows * cols;
    } else {
      num_new_vars = rows * (rows + 1) / 2;
    }
    DRAKE_ASSERT(static_cast<int>(names.size()) == num_new_vars);
    decision_variables_.conservativeResize(num_vars() + num_new_vars,
                                           Eigen::NoChange);
    AppendNanToEnd(num_new_vars, &x_values_);
    int row_index = 0;
    int col_index = 0;
    for (int i = 0; i < num_new_vars; ++i) {
      decision_variables_(num_vars() - num_new_vars + i) =
          symbolic::Variable(names[i], type);
      const int new_var_index = num_vars() - num_new_vars + i;
      decision_variable_index_.insert(std::pair<int, int>(
          decision_variables_(new_var_index).get_id(), new_var_index));
      decision_variable_matrix(row_index, col_index) =
          decision_variables_(num_vars() - num_new_vars + i);
      // If the matrix is not symmetric, then store the variable in column
      // major.
      if (!is_symmetric) {
        if (row_index + 1 < rows) {
          ++row_index;
        } else {
          ++col_index;
          row_index = 0;
        }
      } else {
        // If the matrix is symmetric, then the decision variables are the lower
        // triangular part of the symmetric matrix, also stored in column major.
        if (row_index != col_index) {
          decision_variable_matrix(col_index, row_index) =
              decision_variable_matrix(row_index, col_index);
        }
        if (row_index + 1 < rows) {
          ++row_index;
        } else {
          ++col_index;
          row_index = col_index;
        }
      }
    }

    AppendNanToEnd(num_new_vars, &x_initial_guess_);
  }

  MatrixXDecisionVariable NewVariables(VarType type, int rows, int cols,
                                       bool is_symmetric,
                                       const std::vector<std::string>& names);

  template <typename T>
  void NewIndeterminates_impl(
      const T& names, Eigen::Ref<MatrixXIndeterminate> indeterminates_matrix) {
    int rows = indeterminates_matrix.rows();
    int cols = indeterminates_matrix.cols();
    int num_new_vars = rows * cols;

    DRAKE_ASSERT(static_cast<int>(names.size()) == num_new_vars);
    indeterminates_.conservativeResize(indeterminates_.rows() + num_new_vars,
                                       Eigen::NoChange);
    int row_index = 0;
    int col_index = 0;
    for (int i = 0; i < num_new_vars; ++i) {
      indeterminates_(indeterminates_.rows() - num_new_vars + i) =
          symbolic::Variable(names[i]);

      const int new_var_index = indeterminates_.rows() - num_new_vars + i;
      indeterminates_index_.insert(std::pair<size_t, size_t>(
          indeterminates_(new_var_index).get_id(), new_var_index));
      indeterminates_matrix(row_index, col_index) =
          indeterminates_(indeterminates_.rows() - num_new_vars + i);

      // store the indeterminate in column major.
      if (row_index + 1 < rows) {
        ++row_index;
      } else {
        ++col_index;
        row_index = 0;
      }
    }
  }

  /*
   * Given a matrix of decision variables, checks if every entry in the
   * matrix is a decision variable in the program; throws a runtime
   * error if any variable is not a decision variable in the program.
   * @tparam Derived An Eigen::Matrix type of symbolic Variable.
   * @param vars A matrix of variables.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value>::type
  CheckIsDecisionVariable(const Eigen::MatrixBase<Derived>& vars) const {
    for (int i = 0; i < vars.rows(); ++i) {
      for (int j = 0; j < vars.cols(); ++j) {
        if (decision_variable_index_.find(vars(i, j).get_id()) ==
            decision_variable_index_.end()) {
          std::ostringstream oss;
          oss << vars(i, j)
              << " is not a decision variable of the mathematical program.\n";
          throw std::runtime_error(oss.str());
        }
      }
    }
  }

  /*
   * Ensure a binding is valid *before* adding it to the program.
   * @pre The binding has not yet been registered.
   * @pre The decision variables have been registered.
   * @throws std::runtime_error if the binding is invalid.
   */
  template <typename C>
  void CheckBinding(const Binding<C>& binding) const {
    // TODO(eric.cousineau): In addition to identifiers, hash bindings by
    // their constraints and their variables, to prevent duplicates.
    // TODO(eric.cousineau): Once bindings have identifiers (perhaps
    // retrofitting `description`), ensure that they have unique names.
    CheckIsDecisionVariable(binding.variables());
  }

  // Adds a constraint represented by a set of symbolic formulas to the
  // program.
  //
  // Precondition: ??? f ??? formulas, is_relational(f).
  Binding<Constraint> AddConstraint(
      const std::set<symbolic::Formula>& formulas);

  // Adds a linear-equality constraint represented by a set of symbolic formulas
  // to the program.
  //
  // Precondition: ??? f ??? formulas, is_equal_to(f).
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const std::set<symbolic::Formula>& formulas);

  /**
   * Adds new variables to MathematicalProgram.
   */
  template <int Rows, int Cols>
  MatrixDecisionVariable<Rows, Cols> NewVariables(
      VarType type, const typename NewVariableNames<Rows, Cols>::type& names,
      int rows, int cols) {
    DRAKE_DEMAND(rows >= 0 && cols >= 0);
    MatrixDecisionVariable<Rows, Cols> decision_variable_matrix;
    decision_variable_matrix.resize(rows, cols);
    NewVariables_impl(type, names, false, decision_variable_matrix);
    return decision_variable_matrix;
  }

  /**
   * Adds symmetric matrix variables to optimization program. Only the lower
   * triangular part of the matrix is used as decision variables.
   * @param names The names of the stacked columns of the lower triangular part
   * of the matrix.
   */
  template <int Rows>
  MatrixDecisionVariable<Rows, Rows> NewSymmetricVariables(
      VarType type, const typename NewSymmetricVariableNames<Rows>::type& names,
      int rows = Rows) {
    MatrixDecisionVariable<Rows, Rows> decision_variable_matrix(rows, rows);
    NewVariables_impl(type, names, true, decision_variable_matrix);
    return decision_variable_matrix;
  }
};

}  // namespace solvers
}  // namespace drake
