#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_forces.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

/// A %Joint models the kinematical relationship which characterizes the
/// possible relative motion between two bodies.
/// The two bodies connected by a %Joint object are referred to as the
/// _parent_ and _child_ bodies. Although the terms _parent_ and _child_ are
/// sometimes used synonymously to describe the relationship between inboard and
/// outboard bodies in multibody models, this usage is wholly unrelated and
/// implies nothing about the inboard-outboard relationship between the bodies.
/// A %Joint is a model of a physical kinematic constraint between two bodies,
/// a constraint that in the real physical system does not even allude to the
/// ordering of the bodies.
///
/// In Drake we define a frame F rigidly attached to the parent body P with pose
/// `X_PF` and a frame M rigidly attached to the child body B with pose `X_BM`.
/// A %Joint object specifies a kinematic relation between frames F and M,
/// which in turn imposes a kinematic relation between bodies P and B.
///
/// Typical joints include the ball joint, to allow unrestricted rotations about
/// a given point, the revolute joint, that constraints two bodies to rotate
/// about a given common axis, etc.
///
/// Consider the following example to build a simple pendulum system:
///
/// @code
/// MultibodyTree<double> model;
/// // ... Code here to setup quantities below as mass, com, etc. ...
/// const Body<double>& pendulum =
///   model.AddBody<RigidBody>(SpatialInertia<double>(mass, com, unit_inertia));
/// // We will connect the pendulum body to the world using a RevoluteJoint.
/// // In this simple case the parent body P is the model's world body and frame
/// // F IS the world frame.
/// // Additionally, we need to specify the pose of frame M on the pendulum's
/// // body frame B.
/// // Say we declared and initialized X_BM...
/// const RevoluteJoint<double>& elbow =
///   model.AddJoint<RevoluteJoint>(
///     "Elbow",                /* joint name */
///     model.world_body(),     /* parent body */
///     {},                     /* frame F IS the world frame W */
///     pendulum,               /* child body, the pendulum */
///     X_BM,                   /* pose of frame M in the body frame B */
///     Vector3d::UnitZ());     /* revolute axis in this case */
/// @endcode
///
/// @warning Do not ever attempt to instantiate and manipulate %Joint objects
/// on the stack; it will fail. Add joints to your model using the provided API
/// MultibodyTree::AddJoint() as in the example above.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Joint : public MultibodyTreeElement<Joint<T>, JointIndex>  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Joint)

  /// Creates a joint between two Frame objects which imposes a given kinematic
  /// relation between frame F attached on the parent body P and frame M
  /// attached on the child body B. The joint will be initialized to the model
  /// instance from @p frame_on_child (this is the typical convention for joints
  /// between the world and a model, or between two models (e.g. an arm to a
  /// gripper)).  See this class's documentation for further details.
  ///
  /// @param[in] name
  ///   A string with a name identifying `this` joint.
  /// @param[in] frame_on_parent
  ///   The frame F attached on the parent body connected by this joint.
  /// @param[in] frame_on_child
  ///   The frame M attached on the child body connected by this joint.
  /// @param[in] lower_limit
  ///   A vector storing the position lower limit for each generalized position.
  ///   It must have the same size as `upper_limit`.
  ///   A value equal to -??? implies no lower limit.
  /// @param[in] upper_limit
  ///   A vector storing the position upper limit for each generalized position.
  ///   It must have the same size as `lower_limit`.
  ///   A value equal to +??? implies no upper limit.
  Joint(const std::string& name,
        const Frame<T>& frame_on_parent, const Frame<T>& frame_on_child,
        const VectorX<double>& lower_limits,
        const VectorX<double>& upper_limits)
      : MultibodyTreeElement<Joint<T>, JointIndex>(
            frame_on_child.model_instance()),
        name_(name),
        frame_on_parent_(frame_on_parent), frame_on_child_(frame_on_child),
        lower_limits_(lower_limits), upper_limits_(upper_limits) {
    // Notice `this` joint references `frame_on_parent` and `frame_on_child` and
    // therefore they must outlive it.
    DRAKE_DEMAND(lower_limits.size() == upper_limits.size());
    // Verify that lower_limit <= upper_limit, elementwise.
    DRAKE_DEMAND((lower_limits.array() <= upper_limits.array()).all());
  }

  virtual ~Joint() {}

  /// Returns the name of this joint.
  const std::string& name() const { return name_; }

  /// Returns a const reference to the parent body P.
  const Body<T>& parent_body() const {
    return frame_on_parent_.body();
  }

  /// Returns a const reference to the child body B.
  const Body<T>& child_body() const {
    return frame_on_child_.body();
  }

  /// Returns a const reference to the frame F attached on the parent body P.
  const Frame<T>& frame_on_parent() const {
    return frame_on_parent_;
  }

  /// Returns a const reference to the frame M attached on the child body B.
  const Frame<T>& frame_on_child() const {
    return frame_on_child_;
  }

  /// Returns the number of degrees of freedom for `this` joint.
  /// E.g., one for a revolute joint and three for a ball joint.
  DRAKE_DEPRECATED("Please use num_velocities().")
  int num_dofs() const {
    return num_velocities();
  }

  /// Returns the index to the first generalized velocity for this joint
  /// within the vector v of generalized velocities for the full multibody
  /// system.
  int velocity_start() const {
    return do_get_velocity_start();
  }

  /// Returns the number of generalized velocities describing this joint.
  int num_velocities() const {
    DRAKE_ASSERT(0 <= do_get_num_velocities() && do_get_num_velocities() <= 6);
    return do_get_num_velocities();
  }

  /// Returns the index to the first generalized position for this joint
  /// within the vector q of generalized positions for the full multibody
  /// system.
  int position_start() const {
    return do_get_position_start();
  }

  /// Returns the number of generalized positions describing this joint.
  int num_positions() const {
    DRAKE_ASSERT(0 <= do_get_num_positions() && do_get_num_positions() <= 7);
    return do_get_num_positions();
  }

  /// Returns a vector of size num_positions() storing the lower limits for each
  /// generalized position for `this` joint.
  /// A limit with value -??? implies no lower limit for the corresponding
  /// position.
  /// Joint limits are returned in order with the limit for position with index
  /// position_start() in the first entry and with the limit for position with
  /// index position_start() + num_positions() - 1 in the last entry.
  const VectorX<double>& lower_limits() const {
    return lower_limits_;
  }

  /// Returns a vector of size num_positions() storing the upper limits for each
  /// generalized position for `this` joint.
  /// A limit with value +??? implies no upper limit for the corresponding
  /// position.
  /// Joint limits are returned in order with the limit for position with index
  /// position_start() in the first entry and with the limit for position with
  /// index position_start() + num_positions() - 1 in the last entry.
  const VectorX<double>& upper_limits() const {
    return upper_limits_;
  }

  /// Returns the position coordinate for joints with a single degree of
  /// freedom.
  /// @throws std::exception if the joint does not have a single degree of
  /// freedom.
  const T& GetOnePosition(const systems::Context<T>& context) const {
    DRAKE_THROW_UNLESS(num_positions() == 1);
    return DoGetOnePosition(context);
  }

  /// Returns the velocity coordinate for joints with a single degree of
  /// freedom.
  /// @throws std::exception if the joint does not have a single degree of
  /// freedom.
  const T& GetOneVelocity(const systems::Context<T>& context) const {
    DRAKE_THROW_UNLESS(num_velocities() == 1);
    return DoGetOneVelocity(context);
  }

  /// Adds into `forces` a force along the one of the joint's degrees of
  /// freedom indicated by index `joint_dof`.
  /// The meaning for this degree of freedom and even its dimensional units
  /// depend on the specific joint sub-class. For a RevoluteJoint for instance,
  /// `joint_dof` can only be 0 since revolute joints's motion subspace only has
  /// one degree of freedom, while the units of `joint_tau` are those of torque
  /// (N???m in the MKS system of units). For multi-dof joints please refer to
  /// the documentation provided by specific joint sub-classes regarding the
  /// meaning of `joint_dof`.
  ///
  /// @param[in] context
  ///   The context storing the state and parameters for the model to which
  ///   `this` joint belongs.
  /// @param[in] joint_dof
  ///   Index specifying one of the degrees of freedom for this joint. The index
  ///   must be in the range `0 <= joint_dof < num_velocities()` or otherwise
  ///   this method will abort.
  /// @param[in] joint_tau
  ///   Generalized force corresponding to the degree of freedom indicated by
  ///   `joint_dof` to be added into `forces`.
  /// @param[out] forces
  ///   On return, this method will add force `joint_tau` for the degree of
  ///   freedom `joint_dof` into the output `forces`. This method aborts if
  ///   `forces` is `nullptr` or if `forces` doest not have the right sizes to
  ///   accommodate a set of forces for the model to which this joint belongs.
  // NVI to DoAddInOneForce().
  void AddInOneForce(
      const systems::Context<T>& context,
      int joint_dof,
      const T& joint_tau,
      MultibodyForces<T>* forces) const {
    DRAKE_DEMAND(forces != nullptr);
    DRAKE_DEMAND(0 <= joint_dof && joint_dof < num_velocities());
    DRAKE_DEMAND(forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    DoAddInOneForce(context, joint_dof, joint_tau, forces);
  }

  /// Adds into `forces` the force due to damping within `this` joint.
  ///
  /// @param[in] context
  ///   The context storing the state and parameters for the model to which
  ///   `this` joint belongs.
  /// @param[out] forces
  ///   On return, this method will add the force due to damping within `this`
  ///   joint. This method aborts if `forces` is `nullptr` or if `forces` does
  ///   not have the right sizes to accommodate a set of forces for the model
  ///   to which this joint belongs.
  // NVI to DoAddInOneForce().
  void AddInDamping(
      const systems::Context<T> &context, MultibodyForces<T>* forces) const {
    DRAKE_DEMAND(forces != nullptr);
    DRAKE_DEMAND(forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    DoAddInDamping(context, forces);
  }

  // Hide the following section from Doxygen.
#ifndef DRAKE_DOXYGEN_CXX
  // NVI to DoCloneToScalar() templated on the scalar type of the new clone to
  // be created. This method is intended to be called by
  // MultibodyTree::CloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> CloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const {
    std::unique_ptr<Joint<ToScalar>> joint_clone = DoCloneToScalar(tree_clone);

    std::unique_ptr<typename Joint<ToScalar>::JointImplementation>
        implementation_clone =
        this->get_implementation().template CloneToScalar<ToScalar>(tree_clone);
    joint_clone->OwnImplementation(std::move(implementation_clone));

    return std::move(joint_clone);
  }
#endif
  // End of hidden Doxygen section.

 protected:
  /// (Advanced) Structure containing all the information needed to build the
  /// MultibodyTree implementation for a %Joint. At MultibodyTree::Finalize() a
  /// %Joint creates a BluePrint of its implementation with MakeModelBlueprint()
  /// so that MultibodyTree can build an implementation for it.
  struct BluePrint {
    std::vector<std::unique_ptr<Mobilizer<T>>> mobilizers_;
    // TODO(amcastro-tri): add force elements, constraints, bodies.
  };

  /// (Advanced) A Joint is implemented in terms of MultibodyTree elements such
  /// as bodies, mobilizers, force elements and constraints. This object
  /// contains the internal details of the MultibodyTree implementation for a
  /// joint. The implementation does not own the MBT elements, it just keeps
  /// references to them.
  /// This is intentionally made a protected member so that derived classes have
  /// access to its definition.
  struct JointImplementation {
    /// Default constructor to create an empty implementation. Used by
    /// Joint::CloneToScalar().
    JointImplementation() {}

    /// This constructor creates an implementation for `this` joint from the
    /// blueprint provided.
    explicit JointImplementation(const BluePrint& blue_print) {
      DRAKE_DEMAND(static_cast<int>(blue_print.mobilizers_.size()) != 0);
      for (const auto& mobilizer : blue_print.mobilizers_) {
        mobilizers_.push_back(mobilizer.get());
      }
    }

    /// Returns the number of mobilizers in this implementation.
    int num_mobilizers() const {
      return static_cast<int>(mobilizers_.size());
    }

    // Hide the following section from Doxygen.
#ifndef DRAKE_DOXYGEN_CXX
    // Helper method to be called within Joint::CloneToScalar() to clone its
    // implementation to the appropriate scalar type.
    template <typename ToScalar>
    std::unique_ptr<typename Joint<ToScalar>::JointImplementation>
    CloneToScalar(const MultibodyTree<ToScalar>& tree_clone) const {
      auto implementation_clone =
          std::make_unique<typename Joint<ToScalar>::JointImplementation>();
      for (const Mobilizer<T>* mobilizer : mobilizers_) {
        const Mobilizer<ToScalar>* mobilizer_clone =
            &tree_clone.get_variant(*mobilizer);
        implementation_clone->mobilizers_.push_back(mobilizer_clone);
      }
      return std::move(implementation_clone);
    }
#endif
    // End of hidden Doxygen section.

    /// References (raw pointers) to the mobilizers that make part of this
    /// implementation.
    std::vector<const Mobilizer<T>*> mobilizers_;
    // TODO(amcastro-tri): add force elements, constraints, bodies, etc.
  };

  /// Implementation to the NVI velocity_start(), see velocity_start() for
  /// details.
  virtual int do_get_velocity_start() const = 0;

  /// Implementation to the NVI num_velocities(), see num_velocities() for
  /// details.
  virtual int do_get_num_velocities() const = 0;

  /// Implementation to the NVI position_start(), see position_start() for
  /// details.
  virtual int do_get_position_start() const = 0;

  /// Implementation to the NVI num_positions(), see num_positions() for
  /// details.
  virtual int do_get_num_positions() const = 0;

  /// Implementation to the NVI GetOnePosition() that must only be implemented
  /// by those joint subclasses that have a single degree of freedom.
  /// The default implementation for all other joints is to abort with an
  /// appropriate message.
  /// Revolute and prismatic are examples of joints that will want to implement
  /// this method.
  virtual const T& DoGetOnePosition(const systems::Context<T>&) const {
    DRAKE_ABORT_MSG("This method can only be called on single-dof joints.");
  }

  /// Implementation to the NVI GetOneVelocity() that must only be implemented
  /// by those joint subclasses that have a single degree of freedom.
  /// The default implementation for all other joints is to abort with an
  /// appropriate message.
  /// Revolute and prismatic are examples of joints that will want to implement
  /// this method.
  virtual const T& DoGetOneVelocity(const systems::Context<T>&) const {
    DRAKE_ABORT_MSG("This method can only be called on single-dof joints.");
  }

  /// Adds into `forces` a force along the one of the joint's degrees of
  /// freedom given by `joint_dof`.
  /// How forces are added to a MultibodyTree model depends on the underlying
  /// implementation of a particular joint and therefore specific %Joint
  /// subclasses must provide a definition for this method. For instance, a
  /// revolute joint could be modeled with a single generalized coordinate for
  /// the angular rotation (implemented through a RevoluteMobilizer) or it could
  /// be modeled using a constraint that only allows rotation about the joint's
  /// axis but that constrains the motion in the other five degrees of freedom.
  /// This method is only called by the public NVI AddInOneForce() and therefore
  /// input arguments were checked to be valid.
  /// @see The public NVI AddInOneForce() for details.
  virtual void DoAddInOneForce(
      const systems::Context<T>& context,
      int joint_dof,
      const T& joint_tau,
      MultibodyForces<T>* forces) const = 0;

  /// Adds into MultibodyForces the forces due to damping within `this` joint.
  /// How forces are added to a MultibodyTree model depends on the underlying
  /// implementation of a particular joint (for instance, mobilizer vs.
  /// constraint) and therefore specific %Joint subclasses must provide a
  /// definition for this method.
  /// The default implementation is a no-op for joints with no damping.
  virtual void DoAddInDamping(
      const systems::Context<T>&, MultibodyForces<T>*) const {}

  // Implements MultibodyTreeElement::DoSetTopology(). Joints have no topology
  // though we could require them to have one in the future.
  void DoSetTopology(const MultibodyTreeTopology&) {}

  /// @name Methods to make a clone templated on different scalar types.
  /// @{
  /// Clones this %Joint (templated on T) to a joint templated on `double`.
  virtual std::unique_ptr<Joint<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const = 0;

  /// Clones this %Joint (templated on T) to a joint templated on AutoDiffXd.
  virtual std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const = 0;
  /// @}

  /// This method must be implemented by derived classes in order to provide
  /// JointImplementationBuilder a BluePrint of their internal implementation
  /// JointImplementation.
  virtual std::unique_ptr<BluePrint> MakeImplementationBlueprint() const = 0;

  /// Returns a const reference to the internal implementation of `this` joint.
  /// @warning The MultibodyTree model must have already been finalized, or
  /// this method will abort.
  const JointImplementation& get_implementation() const {
    // The MultibodyTree must have been finalized for the implementation to be
    // valid.
    DRAKE_DEMAND(this->get_parent_tree().topology_is_valid());
    return *implementation_;
  }

 private:
  // Make all other Joint<U> objects a friend of Joint<T> so they can make
  // Joint<ToScalar>::JointImplementation from CloneToScalar<ToScalar>().
  template <typename> friend class Joint;

  // JointImplementationBuilder is a friend so that it can access the
  // Joint<T>::BluePrint and protected method MakeImplementationBlueprint().
  friend class internal::JointImplementationBuilder<T>;

  // When an implementation is created, either by
  // internal::JointImplementationBuilder or by Joint::CloneToScalar(), this
  // method is called to pass ownership of an implementation to the Joint.
  void OwnImplementation(std::unique_ptr<JointImplementation> implementation) {
    implementation_ = std::move(implementation);
  }

  std::string name_;
  const Frame<T>& frame_on_parent_;
  const Frame<T>& frame_on_child_;

  // Joint limits. These vectors have zero size for joints with no limits.
  VectorX<double> lower_limits_;
  VectorX<double> upper_limits_;

  // The Joint<T> implementation:
  std::unique_ptr<JointImplementation> implementation_;
};

}  // namespace multibody
}  // namespace drake
