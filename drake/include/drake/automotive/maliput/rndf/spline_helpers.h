#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"

#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace rndf {

/// A linear interpolator for arbitrary inverse functions.
/// Helpful for path-length parameterization with ignition::math::Splines.
/// Given a function F, its domain D, and codomain CD, this class gives a linear
/// interpolant of F's inverse that maps CD to D.
class InverseFunctionInterpolator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseFunctionInterpolator)

  /// Constructor that takes the function, its domain interval and an error
  /// boundary for the interpolation.
  /// The valid range of @p function's codomain is determined as follows. The
  /// minimum value is @p xmin applied to @p function. The maximum value is
  /// @p xmax applied to @p function. This is valid because @p function is
  /// assumed to be monotonically increasing with x.
  ///
  /// @param[in] function an arbitrary continuous function for which to
  /// interpolate an inverse. It should be monotonically increasing with x.
  /// @param[in] xmin @p function 's domain interval low bound.
  /// @param[in] xmax @p function 's domain interval upper bound.
  /// @param[in] error_boundary a positive constraint on the maximum error
  /// allowed when approximating the inverse function.
  /// @throws std::runtime_error When @p error_boundary is not positive.
  /// @throws std::runtime_error When @p xmin is equal or greater than @p xmax.
  /// @throws std::runtime_error When evaluating @p function throws.
  explicit InverseFunctionInterpolator(std::function<double(double)> function,
                                       double xmin, double xmax,
                                       double error_boundary);

  /// Interpolates @f$ x^{(derivative_order)}(y) @f$, that is, the inverse of
  /// the given function.
  /// @param[in] derivative_order a non-negative integer describing
  /// the order of the inverse function derivative to interpolate. Any value
  /// bigger than 1 will make the function return 0.0 as this is a linear
  /// interpolant.
  /// @param[in] y the range value to interpolate at, constrained by the direct
  /// function image, which is inside the codomain of the direct function.
  /// @return interpolated @p derivative_order derivative
  /// @f$ x^{(derivative_order)}(y) @f$ .
  /// @throws std::exception when @p derivative_order is a negative integer.
  /// @throws std::exception when @p y is larger than the maximum range of the
  /// function's codomain as described in the constructor's documentation.
  /// @throws std::runtime_error When @p y is smaller than the minimum range of
  /// the function's codomain as described in the constructor's documentation.
  double InterpolateMthDerivative(int derivative_order, double y);

 private:
  // A segment of a single-variable function graph.
  struct FunctionGraphSegment {
    double xmin;  //< Lower x bound.
    double xmax;  //< Upper x bound.
    double ymin;  //< Lower y bound.
    double ymax;  //< Upper y bound.
  };

  // A partition of a single-variable function graph, as a segment plus a set
  // of sub partitions.
  struct FunctionGraphPartition {
    // Segment covered.
    FunctionGraphSegment segment;
    // Sub-partitions covered.
    std::vector<std::unique_ptr<FunctionGraphPartition>> partitions;
  };

  const std::function<double(double)> function_;  //< Base function.
  const double error_boundary_{};         //< Error boundary for interpolation.
  const int partition_tree_degree_{};     //< Function partition tree degree.
  const int partition_tree_max_depth_{};  //< Function partition tree max depth.
  std::unique_ptr<FunctionGraphPartition>
      partition_tree_;  //< Function partition tree.
};

/// An extension for ignition::math::Splines that reparameterizes
/// them by path length.
class ArcLengthParameterizedSpline {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ArcLengthParameterizedSpline)

  /// Constructor that takes a spline and an error bound
  /// for path length parameterization approximations.
  /// @param[in] spline the spline to be parameterized by path length.
  /// @param[in] error_boundary a positive constraint on the
  /// maximum error allowed when approximating the path length
  /// parameterization.
  /// @throws std::runtime_error When @p spline is nullptr.
  /// @throws std::runtime_error When @p error_boundary is not a positive
  /// number.
  explicit ArcLengthParameterizedSpline(
      std::unique_ptr<ignition::math::Spline> spline, double error_boundary);

  /// Interpolates @f$ Q(s) @f$, that is, the spline parameterized
  /// by path length.
  /// @param[in] derivative_order a non-negative integer describing
  /// the order of the function derivative to interpolate. Since cubic
  /// interpolation is done, any derivative order greater than 3 will be zero.
  /// @param[in] s path length to interpolate at, constrained
  /// by the curve dimensions [0, path_length].
  /// @return the @p derivative_order derivative
  ///         @f$ Q^{(derivative_order)}(s) @f$.
  /// @throws std::runtime_error If @p derivative_order is a negative integer.
  ignition::math::Vector3d InterpolateMthDerivative(int derivative_order,
                                                    double s);

  /// @return a mutable pointer to the underlying spline.
  inline ignition::math::Spline* BaseSpline() { return this->q_t_.get(); }

  /// Finds the closest point on the spline to a point in space.
  ///
  /// It will iterate over the range of s of the spline which is 0 to 1 in terms
  /// of ignition::math::Spline's parameter at constant path length @p step
  /// increments. On each iteration we get the distance and save the path
  /// length coordinate that gives the minimum distance.
  ///
  /// @param point the point in space from which we want to get the s path
  /// length value whose image of the direct function (the spline) within 0 and
  /// the total length of the direct function.
  /// @param step the path length increment to iterate over the complete spline
  /// path length.
  /// @return the path length value that applied to the
  /// ignition::math::Spline, which was provided at construction time, has a
  /// minimum distance to @p point.
  double FindClosestPointTo(const ignition::math::Vector3d& point,
                            double step) const;

 private:
  std::unique_ptr<ignition::math::Spline> q_t_;  //< Parameterized curve Q(t).
  std::unique_ptr<InverseFunctionInterpolator>
      F_ts_;  //< Inverse path length function t(s).
};

/// Provides the equivalent set of points in cubic Bezier base from two pairs
/// of points and tangents at the extents of a spline.
/// @param p0 A vector that describes the starting position of the curve.
/// @param t0 A vector that describes the tangent at @p p0.
/// @param p1 A vector that describes the ending position of the curve.
/// @param t1 A vector that describes the tangent at @p p1.
/// @return A vector containing four Bezier control points. The first and last
/// points are the extent points of the Bezier curve and the other two are the
/// tangent controlling waypoints.
std::vector<ignition::math::Vector3d> SplineToBezier(
    const ignition::math::Vector3d& p0, const ignition::math::Vector3d& t0,
    const ignition::math::Vector3d& p1, const ignition::math::Vector3d& t1);

/// Provides the equivalent set of points in cubic spline base from four cubic
/// Bezier control points.
/// @param p0 A vector that describes the starting position of the curve.
/// @param p1 A vector that describes the first control point of the curve.
/// @param p2 A vector that describes the second control point of the curve.
/// @param p3 A vector that describes the last control point of the curve.
/// @return A vector containing four spline control points. The points are
/// returned in the following order:
/// 1. index 0 --> curve position at the beginning.
/// 2. index 1 --> curve tangent at the beginning.
/// 3. index 2 --> curve position at the ending.
/// 4. index 3 --> curve tangent at the ending.
std::vector<ignition::math::Vector3d> BezierToSpline(
    const ignition::math::Vector3d& p0, const ignition::math::Vector3d& p1,
    const ignition::math::Vector3d& p2, const ignition::math::Vector3d& p3);

/// Provides a conditionally convex and monotonic Bezier curve given a vector of
/// control points @p control_points.
/// First it computes the intersection of the lines represented by point and
/// tangent at the beginning and at the end of the curve. From here we have a
/// a first branch in the behavior, if there is no intersection, we assume that
/// all these curves are 2-D curves over the z = 0.0 api::GeoPosition frame.
/// Then, we create two intermediate control points for them that will provide a
/// S shape to match the curve. The change in convexity is set to be in the mid
/// point of the extents of the curve. In case there is an intersection between
/// the two lines, we find it and this will be the critical point. From this
/// step, there are two types of geometries we can generate but for both, the
/// control points at the extents remain the same. For those control points in
/// between we use the following equations to compute them:
///
/// @p control_points[1] = @p control_points[0] +
///                        (??1.0) * @p scale * (critical_point -
///                                             control_points[0])
/// @p control_points[2] = @p control_points[3] +
///                        (???1.0) * @p scale * (critical_point -
///                                             control_points[3])
///
/// When the curve preserves convexity, @p scale is multiplied by (+1.0) in both
/// cases. However, when it is not, opposite signs are used.
/// @param control_points A vector containing four Bezier control points. The
/// first and last points are the extent points of the Bezier curve and the
/// other two are the tangent controlling waypoints.
/// @param scale A scale factor with a range value between 0.0 and 1.0.
/// @return A vector containing four Bezier control points. The first and last
/// points are the extent points of the Bezier curve and the other two are the
/// tangent controlling waypoints.
/// @throws std::runtime_error When the size of @p control_points is different
/// from 4.
/// @throws std::runtime_error When @p scale is bigger than 1.0.
/// @throws std::runtime_error When @p scale is smaller than 0.0.
std::vector<ignition::math::Vector3d> MakeBezierCurveMonotonic(
    const std::vector<ignition::math::Vector3d>& control_points, double scale);

/// Creates a ignition::math::Spline from a set of @p positions. These positions
/// are the control points where the curve must go through. The final curve is
/// based from a PChip algorithm, which makes the interpolation safe in terms of
/// piecewise convexity and monotonicity.
/// @param positions A vector of position where the spline should go through. It
/// should have more than two points. In addition, two consecutive points that
/// have a length of zero will throw an exception since it's not yet supported.
/// @return A ignition's Spline containing as knots the positions vector and as
/// tangent's the PChip's interpolated value.
/// @throws std::runtime_error When positions' size is less than three.
/// @throws std::runtime_error When two consecutive @p positions' items have a
/// distance of zero.
std::unique_ptr<ignition::math::Spline> CreatePChipBasedSpline(
    const std::vector<ignition::math::Vector3d>& positions);

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
