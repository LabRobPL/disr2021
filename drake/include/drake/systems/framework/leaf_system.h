#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_output_port.h"
#include "drake/systems/framework/model_values.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_constraint.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_scalar_converter.h"
#include "drake/systems/framework/system_symbolic_inspector.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/value_checker.h"

namespace drake {
namespace systems {

/** @cond */
// Private helper functions for LeafSystem.
namespace leaf_system_detail {

// Returns the next sample time for the given @p attribute.
template <typename T>
static T GetNextSampleTime(
    const PeriodicEventData& attribute,
    const T& current_time_sec) {
  const double period = attribute.period_sec();
  DRAKE_ASSERT(period > 0);
  const double offset = attribute.offset_sec();
  DRAKE_ASSERT(offset >= 0);

  // If the first sample time hasn't arrived yet, then that is the next
  // sample time.
  if (current_time_sec < offset) {
    return offset;
  }

  // Compute the index in the sequence of samples for the next time to sample,
  // which should be greater than the present time.
  using std::ceil;
  const T offset_time = current_time_sec - offset;
  const T next_k = ceil(offset_time / period);
  T next_t = offset + next_k * period;
  if (next_t <= current_time_sec) {
    next_t = offset + (next_k + 1) * period;
  }
  DRAKE_ASSERT(next_t > current_time_sec);
  return next_t;
}

}  // namespace leaf_system_detail
/** @endcond */


/// A superclass template that extends System with some convenience utilities
/// that are not applicable to Diagrams.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class LeafSystem : public System<T> {
 public:
  // LeafSystem objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafSystem)

  ~LeafSystem() override {}

  /// Allocates a CompositeEventCollection object for this system.
  /// @sa System::AllocateCompositeEventCollection().
  std::unique_ptr<CompositeEventCollection<T>>
      AllocateCompositeEventCollection() const final {
    return std::make_unique<LeafCompositeEventCollection<T>>();
  }

  /// Shadows System<T>::AllocateContext to provide a more concrete return
  /// type LeafContext<T>.
  std::unique_ptr<LeafContext<T>> AllocateContext() const {
    return dynamic_pointer_cast_or_throw<LeafContext<T>>(
        System<T>::AllocateContext());
  }

  // =========================================================================
  // Implementations of System<T> methods.

  /// @cond
  // The three methods below are hidden from doxygen, as described in
  // documentation for their corresponding methods in System.
  std::unique_ptr<EventCollection<PublishEvent<T>>>
  AllocateForcedPublishEventCollection() const override {
    return LeafEventCollection<PublishEvent<T>>::MakeForcedEventCollection();
  }

  std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
  AllocateForcedDiscreteUpdateEventCollection() const override {
    return LeafEventCollection<
        DiscreteUpdateEvent<T>>::MakeForcedEventCollection();
  }

  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
  AllocateForcedUnrestrictedUpdateEventCollection() const override {
    return LeafEventCollection<
        UnrestrictedUpdateEvent<T>>::MakeForcedEventCollection();
  }
  /// @endcond

  std::unique_ptr<ContextBase> DoAllocateContext() const final {
    std::unique_ptr<LeafContext<T>> context = DoMakeLeafContext();
    this->InitializeContextBase(&*context);

    // Reserve parameters via delegation to subclass.
    context->init_parameters(this->AllocateParameters());

    // Reserve state via delegation to subclass.
    context->init_continuous_state(this->AllocateContinuousState());
    context->init_discrete_state(this->AllocateDiscreteState());
    context->init_abstract_state(this->AllocateAbstractState());

    // At this point this LeafContext is complete except possibly for
    // inter-Context dependencies involving port connections to peers or
    // parent. We can now perform some final sanity checks.

    // The numeric vectors used for parameters and state must be contiguous,
    // i.e., valid BasicVectors. In general, a Context's numeric vectors can be
    // any kind of VectorBase including scatter-gather implementations like
    // Supervector. But for a LeafContext, we only allow BasicVectors, which are
    // guaranteed to have a contiguous storage layout.

    // If xc is not BasicVector, the dynamic_cast will yield nullptr, and the
    // invariant-checker will complain.
    const VectorBase<T>* const xc = &context->get_continuous_state_vector();
    detail::CheckBasicVectorInvariants(dynamic_cast<const BasicVector<T>*>(xc));

    // The discrete state must all be valid BasicVectors.
    for (const BasicVector<T>* group :
        context->get_state().get_discrete_state().get_data()) {
      detail::CheckBasicVectorInvariants(group);
    }

    // The numeric parameters must all be valid BasicVectors.
    const int num_numeric_parameters =
        context->num_numeric_parameter_groups();
    for (int i = 0; i < num_numeric_parameters; ++i) {
      const BasicVector<T>& group = context->get_numeric_parameter(i);
      detail::CheckBasicVectorInvariants(&group);
    }

    // Allow derived LeafSystem to validate allocated Context.
    DoValidateAllocatedLeafContext(*context);

    // TODO(sherm1) Remove this line and the corresponding one in
    // Diagram to enable caching by default in Drake.
    context->DisableCaching();

    return context;
  }

  /// Default implementation: sets all continuous state to the model vector
  /// given in DeclareContinuousState (or zero if no model vector was given) and
  /// discrete states to zero. Overrides must not change the number of state
  /// variables.
  // TODO(sherm/russt): Initialize the discrete state from the model vector
  // pending resolution of #7058.
  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {
    unused(context);
    DRAKE_DEMAND(state != nullptr);
    ContinuousState<T>& xc = state->get_mutable_continuous_state();
    if (model_continuous_state_vector_ != nullptr) {
      xc.SetFromVector(model_continuous_state_vector_->get_value());
    } else {
      xc.SetFromVector(VectorX<T>::Zero(xc.size()));
    }
    DiscreteValues<T>& xd = state->get_mutable_discrete_state();
    for (int i = 0; i < xd.num_groups(); i++) {
      BasicVector<T>& s = xd.get_mutable_vector(i);
      s.SetFromVector(VectorX<T>::Zero(s.size()));
    }
    AbstractValues& xa = state->get_mutable_abstract_state();
    xa.CopyFrom(AbstractValues(model_abstract_states_.CloneAllModels()));
  }

  /// Default implementation: sets all numeric parameters to the model vector
  /// given to DeclareNumericParameter, or else if no model was provided sets
  /// the numeric parameter to one.  It sets all abstract parameters to the
  /// model value given to DeclareAbstractParameter.  Overrides must not change
  /// the number of parameters.
  void SetDefaultParameters(const Context<T>& context,
                            Parameters<T>* parameters) const override {
    unused(context);
    for (int i = 0; i < parameters->num_numeric_parameter_groups(); i++) {
      BasicVector<T>& p = parameters->get_mutable_numeric_parameter(i);
      auto model_vector = model_numeric_parameters_.CloneVectorModel<T>(i);
      if (model_vector != nullptr) {
        p.SetFrom(*model_vector);
      } else {
        p.SetFromVector(VectorX<T>::Constant(p.size(), 1.0));
      }
    }
    for (int i = 0; i < parameters->num_abstract_parameters(); i++) {
      AbstractValue& p = parameters->get_mutable_abstract_parameter(i);
      auto model_value = model_abstract_parameters_.CloneModel(i);
      p.SetFrom(*model_value);
    }
  }

  /// Returns the AllocateContinuousState value, which must not be nullptr.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    return AllocateContinuousState();
  }

  /// Returns the AllocateDiscreteState value, which must not be nullptr.
  std::unique_ptr<DiscreteValues<T>> AllocateDiscreteVariables()
      const override {
    return AllocateDiscreteState();
  }

  std::multimap<int, int> GetDirectFeedthroughs() const final {
    // A helper object that is latch-initialized the first time it is needed,
    // but not before.  The optional<> wrapper represents whether or not the
    // latch-init has been attempted; the unique_ptr's non-nullness represents
    // whether or not symbolic form is supported.
    optional<std::unique_ptr<SystemSymbolicInspector>> inspector;

    // This predicate answers a feedthrough query using symbolic form, or
    // returns "true" if symbolic form is unavailable.  It is lazy, in that it
    // will not create the symbolic form until the first time it is invoked.
    auto inspect_symbolic_feedthrough = [this, &inspector](int u, int v) {
      // The very first time we are called, latch-initialize the inspector.
      if (!inspector) { inspector = MakeSystemSymbolicInspector(); }

      // If we have an inspector, delegate to it.  Otherwise, be conservative.
      if (SystemSymbolicInspector* inspector_value = inspector.value().get()) {
        return inspector_value->IsConnectedInputToOutput(u, v);
      } else {
        return true;
      }
    };

    // Iterate all input-output pairs, populating the map with the "true" terms.
    std::multimap<int, int> pairs;
    for (int u = 0; u < this->get_num_input_ports(); ++u) {
      for (int v = 0; v < this->get_num_output_ports(); ++v) {
        // Ask our subclass whether it wants to directly express feedthrough.
        const optional<bool> overridden_feedthrough =
            DoHasDirectFeedthrough(u, v);
        // If our subclass didn't provide an answer, use symbolic form instead.
        const bool direct_feedthrough =
            overridden_feedthrough ? overridden_feedthrough.value() :
            inspect_symbolic_feedthrough(u, v);
        if (direct_feedthrough) {
          pairs.emplace(u, v);
        }
      }
    }
    return pairs;
  };

  int get_num_continuous_states() const final {
    int total = num_generalized_positions_ +
                num_generalized_velocities_+
                num_misc_continuous_states_;
    return total;
  }

 protected:
  // Promote so we don't need "this->" in defaults which show up in Doxygen.
  using SystemBase::all_sources_ticket;

  /// Default constructor that declares no inputs, outputs, state, parameters,
  /// events, nor scalar-type conversion support (AutoDiff, etc.).  To enable
  /// AutoDiff support, use the SystemScalarConverter-based constructor.
  LeafSystem() : LeafSystem(SystemScalarConverter{}) {}

  /// Constructor that declares no inputs, outputs, state, parameters, or
  /// events, but allows subclasses to declare scalar-type conversion support
  /// (AutoDiff, etc.).
  ///
  /// The scalar-type conversion support will use @p converter.
  /// To enable scalar-type conversion support, pass a `SystemTypeTag<S>{}`
  /// where `S` must be the exact class of `this` being constructed.
  ///
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  explicit LeafSystem(SystemScalarConverter converter)
      : System<T>(std::move(converter)) {
    this->set_forced_publish_events(
        LeafEventCollection<PublishEvent<T>>::MakeForcedEventCollection());
    this->set_forced_discrete_update_events(
        LeafEventCollection<
            DiscreteUpdateEvent<T>>::MakeForcedEventCollection());
    this->set_forced_unrestricted_update_events(
        LeafEventCollection<
            UnrestrictedUpdateEvent<T>>::MakeForcedEventCollection());
  }

  /// Provides a new instance of the leaf context for this system. Derived
  /// leaf systems with custom derived leaf system contexts should override this
  /// to provide a context of the appropriate type. The returned context should
  /// be "empty"; invoked by AllocateContext(), the caller will take the
  /// responsibility to initialize the core LeafContext data. The default
  /// implementation provides a default-constructed `LeafContext<T>`.
  virtual std::unique_ptr<LeafContext<T>> DoMakeLeafContext() const {
    return std::make_unique<LeafContext<T>>();
  }

  /// Derived classes that impose restrictions on what resources are permitted
  /// should check those restrictions by implementing this. For example, a
  /// derived class might require a single input and single output. Note that
  /// the supplied Context will be complete except that input and output
  /// dependencies on peer and parent subcontexts will not yet have been set up,
  /// so you may not consider them for validation.
  /// The default implementation does nothing.
  virtual void DoValidateAllocatedLeafContext(
      const LeafContext<T>& context) const {
    unused(context);
  }

  // =========================================================================
  // Implementations of System<T> methods.

  T DoCalcWitnessValue(const Context<T>& context,
                       const WitnessFunction<T>& witness_func) const final {
    DRAKE_DEMAND(this == &witness_func.get_system());
    return witness_func.CalcWitnessValue(context);
  }

  void AddTriggeredWitnessFunctionToCompositeEventCollection(
      Event<T>* event,
      CompositeEventCollection<T>* events) const final {
    DRAKE_DEMAND(event);
    DRAKE_DEMAND(event->get_event_data());
    DRAKE_DEMAND(dynamic_cast<const WitnessTriggeredEventData<T>*>(
        event->get_event_data()));
    DRAKE_DEMAND(events);
    event->AddToComposite(events);
  }

  /// Computes the next update time based on the configured periodic events, for
  /// scalar types that are arithmetic, or aborts for scalar types that are not
  /// arithmetic. Subclasses that require aperiodic events should override.
  /// @post `time` is set to a value greater than or equal to
  ///       `context.get_time()` on return.
  /// @warning If you override this method, think carefully before setting
  ///          `time` to `context.get_time()` on return, which can inadvertently
  ///          cause simulations of systems derived from %LeafSystem to loop
  ///          interminably. Such a loop will occur if, for example, the
  ///          event(s) does not modify the state.
  void DoCalcNextUpdateTime(const Context<T>& context,
                            CompositeEventCollection<T>* events,
                            T* time) const override {
    T min_time = std::numeric_limits<double>::infinity();
    // No periodic events.
    if (periodic_events_.empty()) {
      // No discrete update.
      *time = min_time;
      return;
    }

    // Find the minimum next sample time across all registered events, and
    // the set of registered events that will occur at that time.
    std::vector<const Event<T>*> next_events;
    for (const auto& event_pair : periodic_events_) {
      const PeriodicEventData& event_data =
          event_pair.first;
      const Event<T>* const event = event_pair.second.get();
      const T t = leaf_system_detail::GetNextSampleTime(
          event_data, context.get_time());
      if (t < min_time) {
        min_time = t;
        next_events = {event};
      } else if (t == min_time) {
        next_events.push_back(event);
      }
    }

    // Write out the events that fire at min_time.
    *time = min_time;
    for (const Event<T>* event : next_events) {
      event->AddToComposite(events);
    }
  }

  /// Emits a graphviz fragment for this System. Leaf systems are visualized as
  /// records. For instance, a leaf system with 2 inputs and 1 output is:
  ///
  /// @verbatim
  /// 123456 [shape= record, label="name | {<u0> 0 |<y0> 0} | {<u1> 1 | }"];
  /// @endverbatim
  ///
  /// which looks like:
  ///
  /// @verbatim
  /// +------------+----+
  /// | name  | u0 | u1 |
  /// |       | y0 |    |
  /// +-------+----+----+
  /// @endverbatim
  void GetGraphvizFragment(int max_depth,
                           std::stringstream* dot) const override {
    unused(max_depth);

    // Use the this pointer as a unique ID for the node in the dotfile.
    const int64_t id = this->GetGraphvizId();
    std::string name = this->get_name();
    if (name.empty()) {
      name = this->GetMemoryObjectName();
    }

    // Open the attributes and label.
    *dot << id << " [shape=record, label=\"" << name << "|{";

    // Append input ports to the label.
    *dot << "{";
    for (int i = 0; i < this->get_num_input_ports(); ++i) {
      if (i != 0) *dot << "|";
      *dot << "<u" << i << ">" << this->get_input_port(i).get_name();
    }
    *dot << "}";

    // Append output ports to the label.
    *dot << " | {";
    for (int i = 0; i < this->get_num_output_ports(); ++i) {
      if (i != 0) *dot << "|";
      *dot << "<y" << i << ">" << this->get_output_port(i).get_name();
    }
    *dot << "}";

    // Close the label and attributes.
    *dot << "}\"];" << std::endl;
  }

  void GetGraphvizInputPortToken(const InputPort<T>& port,
                                 int max_depth,
                                 std::stringstream *dot) const final {
    unused(max_depth);
    DRAKE_DEMAND(port.get_system() == this);
    *dot << this->GetGraphvizId() << ":u" << port.get_index();
  }

  void GetGraphvizOutputPortToken(const OutputPort<T>& port,
                                  int max_depth,
                                  std::stringstream *dot) const final {
    unused(max_depth);
    DRAKE_DEMAND(&port.get_system() == this);
    *dot << this->GetGraphvizId() << ":y" << port.get_index();
  }

  // =========================================================================
  // New methods for subclasses to override

  /// Returns a ContinuousState used to implement both CreateDefaultContext and
  /// AllocateTimeDerivatives. Allocates the state configured with
  /// DeclareContinuousState, or none by default. Systems with continuous state
  /// variables may override, but must ensure the ContinuousState vector is
  /// a subclass of BasicVector.
  virtual std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const {
    if (model_continuous_state_vector_ != nullptr) {
      return std::make_unique<ContinuousState<T>>(
          model_continuous_state_vector_->Clone(), num_generalized_positions_,
          num_generalized_velocities_, num_misc_continuous_states_);
    }
    return std::make_unique<ContinuousState<T>>();
  }

  /// Reserves the discrete state as required by CreateDefaultContext. By
  /// default, reserves no state. Systems with discrete state should override.
  virtual std::unique_ptr<DiscreteValues<T>> AllocateDiscreteState() const {
    if (model_discrete_state_vector_ != nullptr) {
      return std::make_unique<DiscreteValues<T>>(
          model_discrete_state_vector_->Clone());
    }
    return std::make_unique<DiscreteValues<T>>();
  }

  /// Reserves the abstract state as required by CreateDefaultContext. By
  /// default, it clones the abstract states declared through
  /// DeclareAbstractState() calls. Derived systems should override for
  /// different behaviors.
  virtual std::unique_ptr<AbstractValues> AllocateAbstractState() const {
    return std::make_unique<AbstractValues>(
        std::move(model_abstract_states_.CloneAllModels()));
  }

  /// Reserves the parameters as required by CreateDefaultContext.  The default
  /// implementation in this class clones the model_vector for all parameters
  /// declared via DeclareNumericParameter(), as well as the model value for all
  /// parameters declared via DeclareAbstractParameter().  Subclasses can
  /// override this method if the default behavior is not sufficient.
  virtual std::unique_ptr<Parameters<T>> AllocateParameters() const {
    std::vector<std::unique_ptr<BasicVector<T>>> numeric_params;
    numeric_params.reserve(model_numeric_parameters_.size());
    for (int i = 0; i < model_numeric_parameters_.size(); ++i) {
      auto param = model_numeric_parameters_.CloneVectorModel<T>(i);
      DRAKE_ASSERT(param != nullptr);
      numeric_params.emplace_back(std::move(param));
    }
    std::vector<std::unique_ptr<AbstractValue>> abstract_params;
    abstract_params.reserve(model_abstract_parameters_.size());
    for (int i = 0; i < model_abstract_parameters_.size(); ++i) {
      auto param = model_abstract_parameters_.CloneModel(i);
      DRAKE_ASSERT(param != nullptr);
      abstract_params.emplace_back(std::move(param));
    }
    return std::make_unique<Parameters<T>>(std::move(numeric_params),
                                           std::move(abstract_params));
  }

  /// Returns true if there is direct-feedthrough from the given @p input_port
  /// to the given @p output_port, false if there is not direct-feedthrough, or
  /// nullopt if unknown (in which case SystemSymbolicInspector will attempt to
  /// measure the feedthrough using symbolic form).
  ///
  /// By default, %LeafSystem assumes there is direct feedthrough of values
  /// from every input to every output.
  /// This is a conservative assumption that ensures we detect and can prevent
  /// the formation of algebraic loops (implicit computations) in system
  /// Diagrams. Systems which do not have direct feedthrough may override that
  /// assumption in two ways:
  ///
  /// - Override DoToSymbolic, allowing %LeafSystem to infer the sparsity
  ///   from the symbolic equations. This method is typically preferred for
  ///   systems that have a symbolic form, but should be avoided in certain
  ///   corner cases where fully descriptive symbolic analysis is impossible,
  ///   e.g., when the symbolic form depends on C++ native conditionals. For
  ///   additional discussion, consult the documentation for
  ///   SystemSymbolicInspector.
  ///
  /// - Override this function directly, reporting manual sparsity. This method
  ///   is recommended when DoToSymbolic has not been implemented, or when
  ///   creating the symbolic form is too computationally expensive, or when its
  ///   output is not fully descriptive, as discussed above. Manually configured
  ///   sparsity must be conservative: if there is any Context for which an
  ///   input port is direct-feedthrough to an output port, this function must
  ///   return either true or nullopt for those two ports.
  virtual optional<bool> DoHasDirectFeedthrough(
      int input_port, int output_port) const {
    unused(input_port, output_port);
    return nullopt;
  }

  // =========================================================================
  // New methods for subclasses to use

  /// Declares a numeric parameter using the given @p model_vector.  This is
  /// the best way to declare LeafSystem numeric parameters.  LeafSystem's
  /// default implementation of AllocateParameters uses model_vector.Clone(),
  /// and the default implementation of SetDefaultParameters() will reset
  /// parameters to their model vectors.  If the @p model_vector declares any
  /// VectorBase::CalcInequalityConstraint() constraints, they will be
  /// re-declared as inequality constraints on this system (see
  /// DeclareInequalityConstraint()).  Returns the index of the new parameter.
  int DeclareNumericParameter(const BasicVector<T>& model_vector) {
    const NumericParameterIndex index(model_numeric_parameters_.size());
    model_numeric_parameters_.AddVectorModel(index, model_vector.Clone());
    MaybeDeclareVectorBaseInequalityConstraint(
        "parameter " + std::to_string(index), model_vector,
        [index](const Context<T>& context) -> const VectorBase<T>& {
          const BasicVector<T>& result = context.get_numeric_parameter(index);
          return result;
        });
    this->AddNumericParameter(index);
    return index;
  }

  /// Extracts the numeric parameters of type U from the @p context at @p index.
  /// Asserts if the context is not a LeafContext, or if it does not have a
  /// vector-valued parameter of type U at @p index.
  template <template <typename> class U = BasicVector>
  const U<T>& GetNumericParameter(const Context<T>& context, int index) const {
    static_assert(std::is_base_of<BasicVector<T>, U<T>>::value,
                  "U must be a subclass of BasicVector.");
    const auto& leaf_context =
        dynamic_cast<const systems::LeafContext<T>&>(context);
    const auto* const params =
        dynamic_cast<const U<T>*>(&leaf_context.get_numeric_parameter(index));
    DRAKE_ASSERT(params != nullptr);
    return *params;
  }

  /// Extracts the numeric parameters of type U from the @p context at @p index.
  /// Asserts if the context is not a LeafContext, or if it does not have a
  /// vector-valued parameter of type U at @p index.
  template <template <typename> class U = BasicVector>
  U<T>& GetMutableNumericParameter(Context<T>* context, int index) const {
    static_assert(std::is_base_of<BasicVector<T>, U<T>>::value,
                  "U must be a subclass of BasicVector.");
    auto* leaf_context = dynamic_cast<systems::LeafContext<T>*>(context);
    DRAKE_ASSERT(leaf_context != nullptr);
    auto* const params = dynamic_cast<U<T>*>(
        &leaf_context->get_mutable_numeric_parameter(index));
    DRAKE_ASSERT(params != nullptr);
    return *params;
  }

  /// Declares an abstract parameter using the given @p model_value.  This is
  /// the best way to declare LeafSystem abstract parameters.  LeafSystem's
  /// default implementation of AllocateParameters uses model_value.Clone(), and
  /// the default implementation of SetDefaultParameters() will reset parameters
  /// to their model values.  Returns the index of the new parameter.
  int DeclareAbstractParameter(const AbstractValue& model_value) {
    const AbstractParameterIndex index(model_abstract_parameters_.size());
    model_abstract_parameters_.AddModel(index, model_value.Clone());
    this->AddAbstractParameter(index);
    return index;
  }

  /// Declares that this System has a simple, fixed-period event specified with
  /// no custom callback function, and its attribute field contains an
  /// Event<T>::PeriodicAttribute constructed from the specified @p period_sec
  /// and @p offset_sec. The first tick will occur at t = @p offset_sec, and it
  /// will recur at every @p period_sec thereafter. Note that the periodic
  /// events returned by system::CalcNextUpdateTime() will happen at a time
  /// strictly after the querying time. E.g. if there is a periodic event with
  /// offset = 0 and period = 5, when calling CalcNextUpdateTime() at t = 0,
  /// the returned event will happen at t = 5 not t = 0.
  ///
  /// @tparam EventType A class derived from Event (e.g., PublishEvent,
  /// DiscreteUpdateEvent, UnrestrictedUpdateEvent, etc.)
  template <typename EventType>
  void DeclarePeriodicEvent(double period_sec, double offset_sec) {
    static_assert(std::is_base_of<Event<T>, EventType>::value,
                  "EventType must be a subclass of Event<T>.");
    EventType event(TriggerType::kPeriodic);
    PeriodicEventData periodic_data;
    periodic_data.set_period_sec(period_sec);
    periodic_data.set_offset_sec(offset_sec);
    event.set_event_data(std::make_unique<PeriodicEventData>(periodic_data));
    periodic_events_.push_back(std::make_pair(periodic_data, event.Clone()));
  }

  /// Declares that this System has a simple, fixed-period event specified by
  /// @p event. The first tick will occur at t = @p offset_sec, and it
  /// will recur at every @p period_sec thereafter. Note that the periodic
  /// events returned by system::CalcNextUpdateTime() will happen at a time
  /// strictly after the querying time. E.g. if there is a periodic event with
  /// offset = 0 and period = 5, when calling CalcNextUpdateTime() at t = 0,
  /// the returned event will happen at t = 5 not t = 0.
  ///
  /// A deep copy of @p event will be made and maintained by `this`. The
  /// trigger type in the clone will be set to kPeriodic, unless it is already
  /// set in the source @p event in which case it must be kPeriodic already.
  /// The @p event's attribute field is preserved.
  ///
  /// @tparam EventType A class derived from Event (e.g., PublishEvent,
  /// DiscreteUpdateEvent, UnrestrictedUpdateEvent, etc.)
  template <typename EventType>
  void DeclarePeriodicEvent(double period_sec, double offset_sec,
                            const EventType& event) {
    DRAKE_DEMAND(event.get_trigger_type() == TriggerType::kUnknown ||
                 event.get_trigger_type() == TriggerType::kPeriodic);
    PeriodicEventData periodic_data;
    periodic_data.set_period_sec(period_sec);
    periodic_data.set_offset_sec(offset_sec);
    auto event_copy = event.Clone();
    event_copy->set_trigger_type(TriggerType::kPeriodic);
    periodic_events_.emplace_back(
        std::make_pair(periodic_data, std::move(event_copy)));
  }

  /// Declares a periodic discrete update event with period = @p period_sec and
  /// offset = @p offset_sec. The event does not have a custom callback
  /// function, and its trigger will be set to Event::TriggerType::kPeriodic.
  /// Its attribute will be an Event<T>::PeriodicAttribute of @p offset_sec and
  /// @p period_sec.
  void DeclarePeriodicDiscreteUpdate(double period_sec, double offset_sec = 0) {
    DeclarePeriodicEvent<DiscreteUpdateEvent<T>>(period_sec, offset_sec);
  }

  /// Declares a periodic unrestricted update event with period = @p period_sec
  /// and offset = @p offset_sec. The event does not have a custom callback
  /// function, and its trigger will be set to Event::TriggerType::kPeriodic.
  /// Its attribute will be an Event<T>::PeriodicAttribute of @p offset_sec and
  /// @p period_sec.
  void DeclarePeriodicUnrestrictedUpdate(double period_sec,
                                         double offset_sec = 0) {
    DeclarePeriodicEvent<UnrestrictedUpdateEvent<T>>(period_sec, offset_sec);
  }

  /// Declares a periodic publish event with period = @p period_sec
  /// and offset = @p offset_sec. The event does not have a custom callback
  /// function, and its trigger will be set to Event::TriggerType::kPeriodic.
  /// Its attribute will be an Event<T>::PeriodicAttribute of @p offset_sec and
  /// @p period_sec.
  void DeclarePeriodicPublish(double period_sec, double offset_sec = 0) {
    DeclarePeriodicEvent<PublishEvent<T>>(period_sec, offset_sec);
  }

  /// Declares a per-step event using @p event, which is deep copied (the
  /// copy is maintained by `this`). @p event's associated trigger type must be
  /// unknown or already set to Event::TriggerType::kPerStep. Aborts otherwise.
  template <typename EventType>
  void DeclarePerStepEvent(const EventType& event) {
    DRAKE_DEMAND(event.get_trigger_type() == TriggerType::kUnknown ||
        event.get_trigger_type() == TriggerType::kPerStep);
    event.AddToComposite(TriggerType::kPerStep, &per_step_events_);
  }

  /// Declares an initialization event by deep copying @p event and storing it
  /// internally. @p event's associated trigger type must be unknown or already
  /// set to Event::TriggerType::kInitialization. Aborts otherwise.
  template <typename EventType>
  void DeclareInitializationEvent(const EventType& event) {
    DRAKE_DEMAND(event.get_trigger_type() == TriggerType::kUnknown ||
        event.get_trigger_type() == TriggerType::kInitialization);
    event.AddToComposite(TriggerType::kInitialization, &initialization_events_);
  }

  /// Declares that this System should reserve continuous state with
  /// @p num_state_variables state variables, which have no second-order
  /// structure. Has no effect if AllocateContinuousState is overridden.
  void DeclareContinuousState(int num_state_variables) {
    const int num_q = 0, num_v = 0;
    DeclareContinuousState(num_q, num_v, num_state_variables);
  }

  /// Declares that this System should reserve continuous state with @p num_q
  /// generalized positions, @p num_v generalized velocities, and @p num_z
  /// miscellaneous state variables.  Has no effect if AllocateContinuousState
  /// is overridden.
  void DeclareContinuousState(int num_q, int num_v, int num_z) {
    const int n = num_q + num_v + num_z;
    DeclareContinuousState(BasicVector<T>(VectorX<T>::Zero(n)), num_q, num_v,
                           num_z);
  }

  /// Declares that this System should reserve continuous state with
  /// @p model_vector.size() miscellaneous state variables, stored in a
  /// vector Cloned from @p model_vector.  Has no effect if
  /// AllocateContinuousState is overridden.
  void DeclareContinuousState(const BasicVector<T>& model_vector) {
    const int num_q = 0, num_v = 0;
    const int num_z = model_vector.size();
    DeclareContinuousState(model_vector, num_q, num_v, num_z);
  }

  /// Declares that this System should reserve continuous state with @p num_q
  /// generalized positions, @p num_v generalized velocities, and @p num_z
  /// miscellaneous state variables, stored in a vector Cloned from
  /// @p model_vector. Aborts if @p model_vector has the wrong size. Has no
  /// effect if AllocateContinuousState is overridden. If the @p model_vector
  /// declares any VectorBase::CalcInequalityConstraint() constraints, they
  /// will be re-declared as inequality constraints on this system (see
  /// DeclareInequalityConstraint()).
  void DeclareContinuousState(const BasicVector<T>& model_vector, int num_q,
                              int num_v, int num_z) {
    DRAKE_DEMAND(model_vector.size() == num_q + num_v + num_z);
    model_continuous_state_vector_ = model_vector.Clone();
    num_generalized_positions_ = num_q;
    num_generalized_velocities_ = num_v;
    num_misc_continuous_states_ = num_z;
    MaybeDeclareVectorBaseInequalityConstraint(
        "continuous state", model_vector,
        [](const Context<T>& context) -> const VectorBase<T>& {
          const ContinuousState<T>& state = context.get_continuous_state();
          return state.get_vector();
        });
  }

  /// Declares that this System should reserve continuous state with @p num_q
  /// generalized positions, @p num_v generalized velocities, and @p num_z
  /// miscellaneous state variables, stored in the a vector Cloned from
  /// @p model_vector. Aborts if @p model_vector is nullptr or has the wrong
  /// size. Has no effect if AllocateContinuousState is overridden.
  DRAKE_DEPRECATED("Use the const-reference model_vector overload instead")
  void DeclareContinuousState(std::unique_ptr<BasicVector<T>> model_vector,
                              int num_q, int num_v, int num_z) {
    DRAKE_DEMAND(model_vector != nullptr);
    DeclareContinuousState(*model_vector, num_q, num_v, num_z);
  }

  /// Declares that this System should reserve discrete state with
  /// @p num_state_variables state variables. Has no effect if
  /// AllocateDiscreteState is overridden.
  // TODO(sherm1) Repeated calls to this should allocate additional discrete
  // state groups. Currently there is only one.
  void DeclareDiscreteState(int num_state_variables) {
    const DiscreteStateIndex index(0);  // Only one implemented currently.
    model_discrete_state_vector_ =
        std::make_unique<BasicVector<T>>(num_state_variables);
    this->AddDiscreteStateGroup(index);
  }

  /// Declares an abstract state.
  /// @param abstract_state The abstract state, its ownership is transferred.
  /// @return index of the declared abstract state.
  int DeclareAbstractState(std::unique_ptr<AbstractValue> abstract_state) {
    const AbstractStateIndex index(model_abstract_states_.size());
    model_abstract_states_.AddModel(index, std::move(abstract_state));
    this->AddAbstractState(index);
    return index;
  }

  // =========================================================================
  /// @name                    Declare input ports
  /// Methods in this section are used by derived classes to declare their
  /// output ports, which may be vector valued or abstract valued.
  ///
  /// You should normally provide a meaningful name for any input port you
  /// create. Names must be unique for this system (passing in a duplicate
  /// name will throw std::logic_error). However, if you specify
  /// kUseDefaultName as the name, then a default name of e.g. "u2", where 2
  /// is the input port number will be provided. An empty name is not
  /// permitted.
  //@{

  /// Declares a vector-valued input port using the given @p model_vector.
  /// This is the best way to declare LeafSystem input ports that require
  /// subclasses of BasicVector.  The port's size and type will be the same as
  /// model_vector. If the port is intended to model a random noise or
  /// disturbance input, @p random_type can (optionally) be used to label it
  /// as such.  If the @p model_vector declares any
  /// VectorBase::CalcInequalityConstraint() constraints, they will be
  /// re-declared as inequality constraints on this system (see
  /// DeclareInequalityConstraint()).
  ///
  /// @see System::DeclareInputPort() for more information.
  const InputPort<T>& DeclareVectorInputPort(
      std::string name, const BasicVector<T>& model_vector,
      optional<RandomDistribution> random_type = nullopt) {
    const int size = model_vector.size();
    const int index = this->get_num_input_ports();
    model_input_values_.AddVectorModel(index, model_vector.Clone());
    MaybeDeclareVectorBaseInequalityConstraint(
        "input " + std::to_string(index), model_vector,
        [this, index](const Context<T>& context) -> const VectorBase<T>& {
          const BasicVector<T>* input = this->EvalVectorInput(context, index);
          DRAKE_DEMAND(input != nullptr);
          return *input;
        });
    return this->DeclareInputPort(NextInputPortName(std::move(name)),
                                  kVectorValued, size, random_type);
  }

  // Avoid shadowing out the no-arg DeclareAbstractInputPort().  (This line
  // should be removed when the deprecated base class methods disappear.)
  using System<T>::DeclareAbstractInputPort;

  /// Declares an abstract-valued input port using the given @p model_value.
  /// This is the best way to declare LeafSystem abstract input ports.
  ///
  /// Any port connected to this input, and any call to FixInputPort for this
  /// input, must provide for values whose type matches this @p model_value.
  ///
  /// @see System::DeclareInputPort() for more information.
  const InputPort<T>& DeclareAbstractInputPort(
      std::string name, const AbstractValue& model_value) {
    const int next_index = this->get_num_input_ports();
    model_input_values_.AddModel(next_index, model_value.Clone());
    return this->DeclareInputPort(NextInputPortName(std::move(name)),
                                  kAbstractValued, 0 /* size */);
  }
  //@}

  // =========================================================================
  /// @name          To-be-deprecated input port declarations
  /// Methods in this section leave out the name parameter and are the same
  /// as invoking the corresponding method with `kUseDefaultName` as the name.
  /// We intend to make specifying the name required and will deprecate these
  /// soon. Don't use them.
  //@{

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  const InputPort<T>& DeclareVectorInputPort(
      const BasicVector<T>& model_vector,
      optional<RandomDistribution> random_type = nullopt) {
    return DeclareVectorInputPort(kUseDefaultName, model_vector, random_type);
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  const InputPort<T>& DeclareAbstractInputPort(
      const AbstractValue& model_value) {
    return DeclareAbstractInputPort(kUseDefaultName, model_value);
  }
  //@}

  // =========================================================================
  /// @name                    Declare output ports
  /// @anchor DeclareLeafOutputPort_documentation
  ///
  /// Methods in this section are used by derived classes to declare their
  /// output ports, which may be vector valued or abstract valued. Every output
  /// port must have an _allocator_ function and
  /// a _calculator_ function. The allocator returns an object suitable for
  /// holding a value of the output port. The calculator uses the contents of
  /// a given Context to produce the output port's value, which is placed in
  /// an object of the type returned by the allocator.
  ///
  /// Although the allocator and calculator functions ultimately satisfy generic
  /// function signatures defined in LeafOutputPort, we provide a variety
  /// of `DeclareVectorOutputPort()` and `DeclareAbstractOutputPort()`
  /// signatures here for convenient specification, with mapping to the generic
  /// form handled invisibly. In particular, allocators are most easily defined
  /// by providing a model value that can be used to construct an
  /// allocator that copies the model when a new value object is needed.
  /// Alternatively a method can be provided that constructs a value object when
  /// invoked (those methods are conventionally, but not necessarily, named
  /// `MakeSomething()` where `Something` is replaced by the output port value
  /// type).
  ///
  /// Because output port values are ultimately stored in AbstractValue objects,
  /// the underlying types must be suitable. For vector ports, that means the
  /// type must be BasicVector or a class derived from BasicVector. For abstract
  /// ports, the type must be copy constructible or cloneable. For
  /// methods below that are not given an explicit model value or construction
  /// ("make") method, the underlying type must be default constructible.
  /// @see drake::systems::Value for more about abstract values.
  ///
  /// A list of prerequisites may be provided for the calculator function to
  /// avoid unnecessary recomputation. If no prerequisites are provided, the
  /// default is to assume the output port value is dependent on all possible
  /// sources. See @ref DeclareCacheEntry_documentation "DeclareCacheEntry"
  /// for more information about prerequisites.
  ///
  /// Output ports must have a name that is unique within the owning subsystem.
  /// Users can provide meaningful names or specify the name as
  /// `kUseDefaultName` in which case a name like "y3" is
  /// automatically provided, where the number is the output port index. An
  /// empty name is not permitted.
  //@{

  /// Declares a vector-valued output port by specifying (1) a model vector of
  /// type BasicVectorSubtype derived from BasicVector and initialized to the
  /// correct size and desired initial value, and (2) a calculator function that
  /// is a class member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputVector(const Context<T>&,
  ///                                 BasicVectorSubtype*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>`. Template
  /// arguments will be deduced and do not need to be specified.
  template <class MySystem, typename BasicVectorSubtype>
  const OutputPort<T>& DeclareVectorOutputPort(
      std::string name,
      const BasicVectorSubtype& model_vector,
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    static_assert(std::is_base_of<BasicVector<T>, BasicVectorSubtype>::value,
                  "Expected vector type derived from BasicVector.");
    // We need to obtain a `this` pointer of the right derived type to capture
    // in the calculator functor, so that it will be able to invoke the given
    // mmember function `calc()`.
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    // Currently all vector ports in Drake require a fixed size that is known
    // at the time the port is declared.
    auto& port = CreateVectorLeafOutputPort(
        NextOutputPortName(std::move(name)), model_vector.size(),
        // Allocator function just clones the given model vector.
        MakeAllocCallback<BasicVector<T>>(model_vector),
        // Calculator function downcasts to specific vector type and invokes
        // the given member function.
        [this_ptr, calc](const Context<T>& context, BasicVector<T>* result) {
          auto typed_result = dynamic_cast<BasicVectorSubtype*>(result);
          DRAKE_DEMAND(typed_result != nullptr);
          (this_ptr->*calc)(context, typed_result);
        },
        std::move(prerequisites_of_calc));
    // Caution: "name" is empty now.
    MaybeDeclareVectorBaseInequalityConstraint(
        "output " + std::to_string(int{port.get_index()}), model_vector,
        [&port, storage = std::shared_ptr<AbstractValue>{}](
            const Context<T>& context) mutable -> const VectorBase<T>& {
          // Because we must return a VectorBase by const reference, our lambda
          // object needs a member field to maintain storage for our result.
          // We must use a shared_ptr not because we share storage, but because
          // our lambda must be copyable.  This will go away once Eval works.
          storage = port.Allocate();
          // TODO(jwnimmer-tri) We should use port.Eval(), once it works.
          port.Calc(context, storage.get());
          return storage->GetValue<BasicVector<T>>();
        });
    return port;
  }

  /// Declares a vector-valued output port by specifying _only_ a calculator
  /// function that is a class member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputVector(const Context<T>&,
  ///                                 BasicVectorSubtype*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and
  /// `BasicVectorSubtype` is derived from `BasicVector<T>` and has a suitable
  /// default constructor that allocates a vector of the expected size. This
  /// will use `BasicVectorSubtype{}` (that is, the default constructor) to
  /// produce a model vector for the output port's value.
  /// Template arguments will be deduced and do not need to be specified.
  ///
  /// @note The default constructor will be called once immediately, and
  /// subsequent allocations will just copy the model value without invoking the
  /// constructor again. If you want the constructor invoked again at each
  /// allocation (not common), use one of the other signatures to explicitly
  /// provide a method for the allocator to call; that method can then invoke
  /// the `BasicVectorSubtype` default constructor.
  template <class MySystem, typename BasicVectorSubtype>
  const OutputPort<T>& DeclareVectorOutputPort(
      std::string name,
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    static_assert(
        std::is_default_constructible<BasicVectorSubtype>::value,
        "LeafSystem::DeclareVectorOutputPort(calc): the one-argument form of "
        "this method requires that the output type has a default constructor");
    // Invokes the previous method.
    return DeclareVectorOutputPort(NextOutputPortName(std::move(name)),
                                   BasicVectorSubtype{}, calc,
                                   std::move(prerequisites_of_calc));
  }

  /// (Advanced) Declares a vector-valued output port using the given
  /// `model_vector` and a function for calculating the port's value at runtime.
  /// The port's size will be model_vector.size(), and the default allocator for
  /// the port will be model_vector.Clone(). Note that this takes the calculator
  /// function in its most generic form; if you have a member function available
  /// use one of the other signatures.
  /// @see LeafOutputPort::CalcVectorCallback
  const OutputPort<T>& DeclareVectorOutputPort(
      std::string name,
      const BasicVector<T>& model_vector,
      typename LeafOutputPort<T>::CalcVectorCallback vector_calc_function,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    auto& port = CreateVectorLeafOutputPort(NextOutputPortName(std::move(name)),
        model_vector.size(), MakeAllocCallback(model_vector),
        std::move(vector_calc_function), std::move(prerequisites_of_calc));
    return port;
  }

  /// Declares an abstract-valued output port by specifying a model value of
  /// concrete type `OutputType` and a calculator function that is a class
  /// member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  /// @endcode
  /// where `MySystem` must be a class derived from `LeafSystem<T>`.
  /// `OutputType` must be such that `Value<OutputType>` is permitted.
  /// Template arguments will be deduced and do not need to be specified.
  /// @see drake::systems::Value
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      std::string name, const OutputType& model_value,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);

    auto& port = CreateAbstractLeafOutputPort(
        NextOutputPortName(std::move(name)), MakeAllocCallback(model_value),
        [this_ptr, calc](const Context<T>& context, AbstractValue* result) {
          OutputType& typed_result = result->GetMutableValue<OutputType>();
          (this_ptr->*calc)(context, &typed_result);
        },
        std::move(prerequisites_of_calc));
    return port;
  }

  /// Declares an abstract-valued output port by specifying only a calculator
  /// function that is a class member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>`. `OutputType`
  /// is a concrete type such that `Value<OutputType>` is permitted, and
  /// must be default constructible, so that we can create a model value using
  /// `Value<OutputType>{}` (value initialized so numerical types will be
  /// zeroed in the model).
  /// Template arguments will be deduced and do not need to be specified.
  ///
  /// @note The default constructor will be called once immediately, and
  /// subsequent allocations will just copy the model value without invoking the
  /// constructor again. If you want the constructor invoked again at each
  /// allocation (not common), use one of the other signatures to explicitly
  /// provide a method for the allocator to call; that method can then invoke
  /// the `OutputType` default constructor.
  /// @see drake::systems::Value
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      std::string name,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    static_assert(
        std::is_default_constructible<OutputType>::value,
        "LeafSystem::DeclareAbstractOutputPort(calc): the one-argument form of "
        "this method requires that the output type has a default constructor");
    // Note that value initialization {} is required here.
    return DeclareAbstractOutputPort(NextOutputPortName(std::move(name)),
                                     OutputType{}, calc,
                                     std::move(prerequisites_of_calc));
  }

  /// Declares an abstract-valued output port by specifying member functions to
  /// use both for the allocator and calculator. The signatures are:
  /// @code
  /// OutputType MySystem::MakeOutputValue() const;
  /// void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and `OutputType`
  /// may be any concrete type such that `Value<OutputType>` is permitted.
  /// See alternate signature if your allocator method needs a Context.
  /// Template arguments will be deduced and do not need to be specified.
  /// @see drake::systems::Value
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      std::string name, OutputType (MySystem::*make)() const,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);

    auto& port = CreateAbstractLeafOutputPort(
        NextOutputPortName(std::move(name)),
        [this_ptr, make]() { return AbstractValue::Make((this_ptr->*make)()); },
        [this_ptr, calc](const Context<T>& context, AbstractValue* result) {
          OutputType& typed_result = result->GetMutableValue<OutputType>();
          (this_ptr->*calc)(context, &typed_result);
        },
        std::move(prerequisites_of_calc));
    return port;
  }

  /// (Advanced) Declares an abstract-valued output port using the given
  /// allocator and calculator functions provided in their most generic forms.
  /// If you have a member function available use one of the other signatures.
  /// @see LeafOutputPort::AllocCallback, LeafOutputPort::CalcCallback
  const OutputPort<T>& DeclareAbstractOutputPort(
      std::string name,
      typename LeafOutputPort<T>::AllocCallback alloc_function,
      typename LeafOutputPort<T>::CalcCallback calc_function,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    auto& port = CreateAbstractLeafOutputPort(
        NextOutputPortName(std::move(name)), std::move(alloc_function),
        std::move(calc_function), std::move(prerequisites_of_calc));
    return port;
  }
  //@}

  // =========================================================================
  /// @name          To-be-deprecated output port declarations
  /// Methods in this section leave out the name parameter and are the same
  /// as invoking the corresponding method with `kUseDefaultName` as the name.
  /// We intend to make specifying the name required and will deprecate these
  /// soon. Don't use them.
  //@{

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  template <class MySystem, typename BasicVectorSubtype>
  const OutputPort<T>& DeclareVectorOutputPort(
      const BasicVectorSubtype& model_vector,
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareVectorOutputPort(kUseDefaultName, model_vector, calc,
                                   std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  template <class MySystem, typename BasicVectorSubtype>
  const OutputPort<T>& DeclareVectorOutputPort(
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareVectorOutputPort(kUseDefaultName, calc,
                                   std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  const OutputPort<T>& DeclareVectorOutputPort(
      const BasicVector<T>& model_vector,
      typename LeafOutputPort<T>::CalcVectorCallback vector_calc_function,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareVectorOutputPort(kUseDefaultName, model_vector,
                                   std::move(vector_calc_function),
                                   std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447. Note that the deprecated method is not available for
  /// `OutputType` std::string as that would create an ambiguity. In that
  /// case the name is required.
  template <class MySystem, typename OutputType>
  std::enable_if_t<!std::is_same<OutputType, std::string>::value,
                   const OutputPort<T>&>
  DeclareAbstractOutputPort(const OutputType& model_value,
                            void (MySystem::*calc)(const Context<T>&,
                                                   OutputType*) const,
                            std::set<DependencyTicket> prerequisites_of_calc = {
                                all_sources_ticket()}) {
    return DeclareAbstractOutputPort(kUseDefaultName, model_value, calc,
                                     std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareAbstractOutputPort<MySystem, OutputType>(
        kUseDefaultName, calc, std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      OutputType (MySystem::*make)() const,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareAbstractOutputPort(kUseDefaultName, make, calc,
                                     std::move(prerequisites_of_calc));
  }

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  const OutputPort<T>& DeclareAbstractOutputPort(
      typename LeafOutputPort<T>::AllocCallback alloc_function,
      typename LeafOutputPort<T>::CalcCallback calc_function,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareAbstractOutputPort(kUseDefaultName, std::move(alloc_function),
                                     std::move(calc_function),
                                     std::move(prerequisites_of_calc));
  }
  //@}

  // =========================================================================
  /// @name                    Declare witness functions
  /// Methods in this section are used by derived classes to declare any
  /// witness functions useful for ensuring that integration ends a step upon
  /// entering particular times or states.
  ///
  /// In contrast to other declaration methods (e.g., DeclareVectorOutputPort(),
  /// for which the System class creates and stores the objects and returns
  /// references to them, the witness function declaration functions return
  /// heap-allocated objects that the subclass of leaf system owns. This
  /// facilitates returning pointers to these objects in
  /// System::DoGetWitnessFunctions().
  //@{

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, and calculator function; and
  /// with no event object.
  /// @note Constructing a witness function with no corresponding event forces
  ///       Simulator's integration of an ODE to end a step at the witness
  ///       isolation time. For example, isolating a function's minimum or
  ///       maximum values can be realized with a witness that triggers on a
  ///       sign change of the function's time derivative, ensuring that the
  ///       actual extreme value is present in the discretized trajectory.
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const) const {
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc);
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, and calculator function; and
  /// with no event object.
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      std::function<T(const Context<T>&)> calc) const {
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc);
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, calculator function, and
  /// publish event callback function for when this triggers.
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      void (MySystem::*publish_callback)(
          const Context<T>&, const PublishEvent<T>&) const) const {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived system.");
    auto fn = [this, publish_callback](
        const Context<T>& context, const PublishEvent<T>& publish_event) {
      auto system_ptr = dynamic_cast<const MySystem*>(this);
      DRAKE_DEMAND(system_ptr);
      return (system_ptr->*publish_callback)(context, publish_event);
    };
    PublishEvent<T> publish_event(fn);
    publish_event.set_trigger_type(TriggerType::kWitness);
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc, publish_event.Clone());
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, calculator function, and
  /// discrete update event callback function for when this triggers.
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      void (MySystem::*du_callback)(const Context<T>&,
          const DiscreteUpdateEvent<T>&, DiscreteValues<T>*) const) const {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived system.");
    auto fn = [this, du_callback](const Context<T>& context,
        const DiscreteUpdateEvent<T>& du_event, DiscreteValues<T>* values) {
      auto system_ptr = dynamic_cast<const MySystem*>(this);
      DRAKE_DEMAND(system_ptr);
      return (system_ptr->*du_callback)(context, du_event, values);
    };
    DiscreteUpdateEvent<T> du_event(fn);
    du_event.set_trigger_type(TriggerType::kWitness);
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc, du_event.Clone());
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, calculator function, and
  /// unrestricted update event callback function for when this triggers.
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      void (MySystem::*uu_callback)(const Context<T>&,
          const UnrestrictedUpdateEvent<T>&, State<T>*) const) const {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived system.");
    auto fn = [this, uu_callback](const Context<T>& context,
        const UnrestrictedUpdateEvent<T>& uu_event, State<T>* state) {
      auto system_ptr = dynamic_cast<const MySystem*>(this);
      DRAKE_DEMAND(system_ptr);
      return (system_ptr->*uu_callback)(context, uu_event, state);
    };
    UnrestrictedUpdateEvent<T> uu_event(fn);
    uu_event.set_trigger_type(TriggerType::kWitness);
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc, uu_event.Clone());
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, and calculator
  /// function, and with an object corresponding to the event that is to be
  /// dispatched when this witness function triggers. Example types of event
  /// objects are publish, discrete variable update, unrestricted update events.
  /// A clone of the event will be owned by the newly constructed
  /// WitnessFunction.
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      const Event<T>& e) const {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived system.");
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc, e.Clone());
  }

  /// Constructs the witness function with the given description (used primarily
  /// for debugging and logging), direction type, and calculator
  /// function, and with an object corresponding to the event that is to be
  /// dispatched when this witness function triggers. Example types of event
  /// objects are publish, discrete variable update, unrestricted update events.
  /// A clone of the event will be owned by the newly constructed
  /// WitnessFunction.
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      std::function<T(const Context<T>&)> calc,
      const Event<T>& e) const {
    return std::make_unique<WitnessFunction<T>>(
        this, description, direction_type, calc, e.Clone());
  }
  //@}

  /// Declares a system constraint of the form
  ///   f(context) = 0
  /// by specifying a member function to use to calculate the (VectorX)
  /// constraint value with a signature:
  /// @code
  /// void MySystem::CalcConstraint(const Context<T>&, VectorX<T>*) const;
  /// @endcode
  ///
  /// @param count is the dimension of the VectorX output.
  /// @param description should be a human-readable phrase.
  /// @returns The index of the constraint.
  /// Template arguments will be deduced and do not need to be specified.
  ///
  /// @see SystemConstraint<T> for more information about the meaning of
  /// these constraints.
  template <class MySystem>
  SystemConstraintIndex DeclareEqualityConstraint(
      void (MySystem::*calc)(const Context<T>&, VectorX<T>*) const,
      int count, const std::string& description) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    return DeclareEqualityConstraint(
        [this_ptr, calc](const Context<T>& context, VectorX<T>* value) {
          DRAKE_DEMAND(value != nullptr);
          (this_ptr->*calc)(context, value);
        },
        count, description);
  }

  /// Declares a system constraint of the form
  ///   f(context) = 0
  /// by specifying a std::function to use to calculate the (Vector) constraint
  /// value with a signature:
  /// @code
  /// void CalcConstraint(const Context<T>&, VectorX<T>*);
  /// @endcode
  ///
  /// @param count is the dimension of the VectorX output.
  /// @param description should be a human-readable phrase.
  /// @returns The index of the constraint.
  ///
  /// @see SystemConstraint<T> for more information about the meaning of
  /// these constraints.
  SystemConstraintIndex DeclareEqualityConstraint(
      typename SystemConstraint<T>::CalcCallback calc, int count,
      const std::string& description) {
    return this->AddConstraint(std::make_unique<SystemConstraint<T>>(
        calc, count, SystemConstraintType::kEquality, description));
  }

  /// Declares a system constraint of the form
  ///   f(context) ??? 0
  /// by specifying a member function to use to calculate the (VectorX)
  /// constraint value with a signature:
  /// @code
  /// void MySystem::CalcConstraint(const Context<T>&, VectorX<T>*) const;
  /// @endcode
  ///
  /// @param count is the dimension of the VectorX output.
  /// @param description should be a human-readable phrase.
  /// @returns The index of the constraint.
  /// Template arguments will be deduced and do not need to be specified.
  ///
  /// @see SystemConstraint<T> for more information about the meaning of
  /// these constraints.
  template <class MySystem>
  SystemConstraintIndex DeclareInequalityConstraint(
      void (MySystem::*calc)(const Context<T>&, VectorX<T>*) const,
      int count, const std::string& description) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    return DeclareInequalityConstraint(
        [this_ptr, calc](const Context<T>& context, VectorX<T>* value) {
          DRAKE_DEMAND(value != nullptr);
          (this_ptr->*calc)(context, value);
        },
        count, description);
  }

  /// Declares a system constraint of the form
  ///   f(context) ??? 0
  /// by specifying a std::function to use to calculate the (Vector) constraint
  /// value with a signature:
  /// @code
  /// void CalcConstraint(const Context<T>&, VectorX<T>*);
  /// @endcode
  ///
  /// @param count is the dimension of the VectorX output.
  /// @param description should be a human-readable phrase.
  /// @returns The index of the constraint.
  ///
  /// @see SystemConstraint<T> for more information about the meaning of
  /// these constraints.
  SystemConstraintIndex DeclareInequalityConstraint(
      typename SystemConstraint<T>::CalcCallback calc, int count,
      const std::string& description) {
    return this->AddConstraint(std::make_unique<SystemConstraint<T>>(
        calc, count, SystemConstraintType::kInequality, description));
  }

  /// Derived-class event handler for all simultaneous publish events
  /// in @p events. Override this in your derived LeafSystem if your derived
  /// LeafSystem requires a behavior other than the default behavior, which
  /// traverses events in the arbitrary order they appear in @p events, and
  /// for each event that has a callback function, it will invoke the callback
  /// with @p context and that event. This can be used for tasks that need
  /// read-only access to the context, such as sending messages, producing
  /// console output, debugging, logging, saving the trajectory to a file, etc.
  ///
  /// This method is called only from the virtual DispatchPublishHandler, which
  /// is only called from the public non-virtual Publish(), which will have
  /// already error-checked @p context so you may assume that it is valid.
  ///
  /// @param[in] context Const current context.
  /// @param[in] events All the publish events that need handling.
  virtual void DoPublish(
      const Context<T>& context,
      const std::vector<const PublishEvent<T>*>& events) const {
    for (const PublishEvent<T>* event : events) {
      event->handle(context);
    }
  }

  /// Derived-class event handler for all simultaneous discrete update
  /// events. This method updates the @p discrete_state on discrete update
  /// events. The default implementation traverses events in the arbitrary
  /// order they appear in @p events, and for each event that has a callback
  /// function, it will invoke it with @p context, that event, and
  /// @p discrete_state. Note that the same @p discrete_state is passed to
  /// subsequent callbacks. Override this in your derived LeafSystem if your
  /// derived LeafSystem requires a behavior other than the default.
  ///
  /// This method is called only from the virtual
  /// DispatchDiscreteVariableUpdateHandler(), which is only called from
  /// the public non-virtual CalcDiscreteVariableUpdates(), which will already
  /// have error-checked the parameters so you don't have to. In particular,
  /// implementations may assume that @p context is valid; that
  /// @p discrete_state is non-null, and that the referenced object has the
  /// same constituent structure as was produced by AllocateDiscreteVariables().
  ///
  /// @param[in] context The "before" state.
  /// @param[in] events All the discrete update events that need handling.
  /// @param[in,out] discrete_state The current state of the system on input;
  /// the desired state of the system on return.
  virtual void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>& events,
      DiscreteValues<T>* discrete_state) const {
    for (const DiscreteUpdateEvent<T>* event : events) {
      event->handle(context, discrete_state);
    }
  }

  /// Derived-class event handler for all simultaneous unrestricted
  /// update events. This function updates the @p state *in an unrestricted
  /// fashion* on unrestricted update events. Override this function if you
  /// need your System to update abstract variables or generally make changes
  /// to state that cannot be made using CalcDiscreteVariableUpdates() or
  /// via integration of continuous variables.
  ///
  /// The default implementation traverses events in the arbitrary order they
  /// appear in @p events, and for each event that has a callback function,
  /// it will invoke it with @p context, that event, and @p state. Note that
  /// the same @p state is passed to subsequent callbacks. Override this if
  /// your derived LeafSystem requires a behavior other than the default.
  ///
  /// This method is called only from the virtual
  /// DispatchUnrestrictedUpdateHandler(), which is only called from the
  /// non-virtual public CalcUnrestrictedUpdate(), which will already have
  /// error-checked the parameters so you don't have to. In particular,
  /// implementations may assume that the @p context is valid; that @p state
  /// is non-null, and that the referenced object has the same constituent
  /// structure as the state in @p context.
  ///
  /// @param[in]     context The "before" state that is to be used to calculate
  ///                        the returned state update.
  /// @param[in]     events All the unrestricted update events that need
  ///                       handling.
  /// @param[in,out] state   The current state of the system on input; the
  ///                        desired state of the system on return.
  // TODO(sherm1) Shouldn't require preloading of the output state; better to
  //              note just the changes since usually only a small subset will
  //              be changed by this method.
  virtual void DoCalcUnrestrictedUpdate(
      const Context<T>& context,
      const std::vector<const UnrestrictedUpdateEvent<T>*>& events,
      State<T>* state) const {
    for (const UnrestrictedUpdateEvent<T>* event : events) {
      event->handle(context, state);
    }
  }

 private:
  using SystemBase::NextInputPortName;
  using SystemBase::NextOutputPortName;

  // Either clones the model_value, or else for vector ports allocates a
  // BasicVector, or else for abstract ports throws an exception.
  std::unique_ptr<AbstractValue> DoAllocateInput(
      const InputPort<T>& input_port) const final {
    std::unique_ptr<AbstractValue> model_result =
        model_input_values_.CloneModel(input_port.get_index());
    if (model_result) {
      return model_result;
    }
    if (input_port.get_data_type() == kVectorValued) {
      return std::make_unique<Value<BasicVector<T>>>(input_port.size());
    }
    throw std::logic_error(fmt::format(
        "System::AllocateInputAbstract(): a System with abstract input ports "
        "must pass a model_value to DeclareAbstractInputPort; the port[{}] "
        "named '{}' did not do so (System {})",
        input_port.get_index(), input_port.get_name(),
        this->GetSystemPathname()));
  }

  std::map<PeriodicEventData, std::vector<const Event<T>*>,
      PeriodicEventDataComparator> DoGetPeriodicEvents() const override {
    std::map<PeriodicEventData, std::vector<const Event<T>*>,
        PeriodicEventDataComparator> periodic_events_map;
    for (const auto& i : periodic_events_) {
      periodic_events_map[i.first].push_back(i.second.get());
    }
    return periodic_events_map;
  }

  // Calls DoPublish.
  // Assumes @param events is an instance of LeafEventCollection, throws
  // std::bad_cast otherwise.
  // Assumes @param events is not empty. Aborts otherwise.
  void DispatchPublishHandler(
      const Context<T>& context,
      const EventCollection<PublishEvent<T>>& events) const final {
    const LeafEventCollection<PublishEvent<T>>& leaf_events =
       dynamic_cast<const LeafEventCollection<PublishEvent<T>>&>(events);
    // Only call DoPublish if there are publish events.
    DRAKE_DEMAND(leaf_events.HasEvents());
    this->DoPublish(context, leaf_events.get_events());
  }

  // Calls DoCalcDiscreteVariableUpdates.
  // Assumes @param events is an instance of LeafEventCollection, throws
  // std::bad_cast otherwise.
  // Assumes @param events is not empty. Aborts otherwise.
  void DispatchDiscreteVariableUpdateHandler(
      const Context<T>& context,
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state) const final {
    const LeafEventCollection<DiscreteUpdateEvent<T>>& leaf_events =
        dynamic_cast<const LeafEventCollection<DiscreteUpdateEvent<T>>&>(
            events);
    // TODO(siyuan): should have a API level CopyFrom for DiscreteValues.
    discrete_state->CopyFrom(context.get_discrete_state());
    // Only call DoCalcDiscreteVariableUpdates if there are discrete update
    // events.
    DRAKE_DEMAND(leaf_events.HasEvents());
    this->DoCalcDiscreteVariableUpdates(context, leaf_events.get_events(),
        discrete_state);
  }

  // Calls DoCalcUnrestrictedUpdate.
  // Assumes @param events is an instance of LeafEventCollection, throws
  // std::bad_cast otherwise.
  // Assumes @param events is not empty. Aborts otherwise.
  void DispatchUnrestrictedUpdateHandler(
      const Context<T>& context,
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state) const final {
    const LeafEventCollection<UnrestrictedUpdateEvent<T>>& leaf_events =
        dynamic_cast<const LeafEventCollection<UnrestrictedUpdateEvent<T>>&>(
            events);
    // Only call DoCalcUnrestrictedUpdate if there are unrestricted update
    // events.
    DRAKE_DEMAND(leaf_events.HasEvents());
    this->DoCalcUnrestrictedUpdate(context, leaf_events.get_events(), state);
  }

  void DoGetPerStepEvents(
      const Context<T>&,
      CompositeEventCollection<T>* events) const override {
    events->SetFrom(per_step_events_);
  }

  void DoGetInitializationEvents(
      const Context<T>&,
      CompositeEventCollection<T>* events) const override {
    events->SetFrom(initialization_events_);
  }

  // Returns a SystemSymbolicInspector for this system, or nullptr if one
  // cannot be constructed because this System has no symbolic representation.
  std::unique_ptr<SystemSymbolicInspector> MakeSystemSymbolicInspector() const {
    // We use different implementations when T = Expression or not.
    return MakeSystemSymbolicInspectorImpl(*this);
  }

  // When T == Expression, we don't need to use scalar conversion.
  static std::unique_ptr<SystemSymbolicInspector>
  MakeSystemSymbolicInspectorImpl(const System<symbolic::Expression>& system) {
    return std::make_unique<SystemSymbolicInspector>(system);
  }

  // When T != Expression, attempt to use scalar conversion.
  template <typename T1>
  static
  typename std::enable_if_t<
    !std::is_same<T1, symbolic::Expression>::value,
    std::unique_ptr<SystemSymbolicInspector>>
  MakeSystemSymbolicInspectorImpl(const System<T1>& system) {
    using symbolic::Expression;
    std::unique_ptr<System<Expression>> converted = system.ToSymbolicMaybe();
    if (converted) {
      return MakeSystemSymbolicInspectorImpl(*converted);
    } else {
      return nullptr;
    }
  }

  // Creates a new cached, vector-valued LeafOutputPort in this LeafSystem and
  // returns a reference to it.
  LeafOutputPort<T>& CreateVectorLeafOutputPort(
      std::string name,
      int fixed_size,
      typename LeafOutputPort<T>::AllocCallback vector_allocator,
      typename LeafOutputPort<T>::CalcVectorCallback vector_calculator,
      std::set<DependencyTicket> calc_prerequisites) {
    // Construct a suitable type-erased cache calculator from the given
    // BasicVector<T> calculator function.
    auto cache_calc_function = [vector_calculator](
        const ContextBase& context_base, AbstractValue* abstract) {
      auto& context = dynamic_cast<const Context<T>&>(context_base);

      // The abstract value must be a Value<BasicVector<T>>, even if the
      // underlying object is a more-derived vector type.
      auto value = dynamic_cast<Value<BasicVector<T>>*>(abstract);

      // TODO(sherm1) Make this error message more informative by capturing
      // system and port index info.
      if (value == nullptr) {
        throw std::logic_error(fmt::format(
            "An output port calculation required a {} object for its result "
            "but got a {} object instead.",
            NiceTypeName::Get<Value<BasicVector<T>>>(),
            abstract->GetNiceTypeName()));
      }
      vector_calculator(context, &value->get_mutable_value());
    };

    // The allocator function is identical between output port and cache.
    return CreateCachedLeafOutputPort(
        std::move(name), fixed_size, std::move(vector_allocator),
        std::move(cache_calc_function), std::move(calc_prerequisites));
  }

  // Creates a new cached, abstract-valued LeafOutputPort in this LeafSystem and
  // returns a reference to it.
  LeafOutputPort<T>& CreateAbstractLeafOutputPort(
      std::string name,
      typename LeafOutputPort<T>::AllocCallback allocator,
      typename LeafOutputPort<T>::CalcCallback calculator,
      std::set<DependencyTicket> calc_prerequisites) {
    // Construct a suitable type-erased cache calculator from the given
    // type-T calculator function.
    auto cache_calc_function = [calculator](
        const ContextBase& context_base, AbstractValue* result) {
      const Context<T>& context = dynamic_cast<const Context<T>&>(context_base);
      return calculator(context, result);
    };

    return CreateCachedLeafOutputPort(
        std::move(name), 0 /* size */, std::move(allocator),
        std::move(cache_calc_function), std::move(calc_prerequisites));
  }

  // Creates a new cached LeafOutputPort in this LeafSystem and returns a
  // reference to it. Pass fixed_size == 0 for abstract ports, or non-zero
  // for vector ports. Prerequisites list must not be empty.
  LeafOutputPort<T>& CreateCachedLeafOutputPort(
      std::string name, int fixed_size,
      typename CacheEntry::AllocCallback allocator,
      typename CacheEntry::CalcCallback calculator,
      std::set<DependencyTicket> calc_prerequisites) {
    DRAKE_DEMAND(!calc_prerequisites.empty());
    // Create a cache entry for this output port.
    const OutputPortIndex oport_index(this->get_num_output_ports());
    const CacheEntry& cache_entry = this->DeclareCacheEntry(
        "output port " + std::to_string(oport_index) + "(" + name + ") cache",
        std::move(allocator), std::move(calculator),
        std::move(calc_prerequisites));

    // Create and install the port. Note that it has a separate ticket from
    // the cache entry; the port's tracker will be subscribed to the cache
    // entry's tracker when a Context is created.
    // TODO(sherm1) Use implicit_cast when available (from abseil).
    auto port = std::make_unique<LeafOutputPort<T>>(
        this,  // implicit_cast<const System<T>*>(this)
        this,  // implicit_cast<const SystemBase*>(this)
        std::move(name),
        oport_index, this->assign_next_dependency_ticket(),
        fixed_size == 0 ? kAbstractValued : kVectorValued, fixed_size,
        &cache_entry);
    LeafOutputPort<T>* const port_ptr = port.get();
    this->AddOutputPort(std::move(port));
    return *port_ptr;
  }

  // Creates an abstract output port allocator function from an arbitrary type
  // model value.
  template <typename OutputType>
  static typename LeafOutputPort<T>::AllocCallback MakeAllocCallback(
      const OutputType& model_value) {
    // The given model value may have *either* a copy constructor or a Clone()
    // method, since it just has to be suitable for containing in an
    // AbstractValue. We need to create a functor that is copy constructible,
    // so need to wrap the model value to give it a copy constructor. Drake's
    // copyable_unique_ptr does just that, so is suitable for capture by the
    // allocator functor here.
    copyable_unique_ptr<AbstractValue> owned_model(
        new Value<OutputType>(model_value));
    return [model = std::move(owned_model)]() {
      return model->Clone();
    };
  }

  // If @p model_vector's CalcInequalityConstraint provides any constraints,
  // then declares inequality constraints on `this` using a calc function that
  // obtains a VectorBase from a Context using @p get_vector_from_context and
  // then delegates to the VectorBase::CalcInequalityConstraint.  Note that the
  // model vector is only used to determine how many constraints will appear;
  // it is not part of the ongoing constraint computations.
  void MaybeDeclareVectorBaseInequalityConstraint(
      const std::string& kind,
      const VectorBase<T>& model_vector,
      const std::function<const VectorBase<T>&(const Context<T>&)>&
        get_vector_from_context) {
    VectorX<T> dummy_value;
    model_vector.CalcInequalityConstraint(&dummy_value);
    const int count = dummy_value.size();
    if (count == 0) {
      return;
    }
    this->DeclareInequalityConstraint(
        [get_vector_from_context](const Context<T>& con, VectorX<T>* value) {
          get_vector_from_context(con).CalcInequalityConstraint(value);
        },
        count,
        kind + " of type " + NiceTypeName::Get(model_vector));
  }

  // Periodic Update or Publish events registered on this system.
  std::vector<std::pair<PeriodicEventData,
                        std::unique_ptr<Event<T>>>>
      periodic_events_;

  // Update or Publish events registered on this system for every simulator
  // major time step.
  LeafCompositeEventCollection<T> per_step_events_;

  // Update or Publish events that need to be handled at system initialization.
  LeafCompositeEventCollection<T> initialization_events_;

  // A model continuous state to be used during Context allocation.
  std::unique_ptr<BasicVector<T>> model_continuous_state_vector_;
  int num_generalized_positions_{0};
  int num_generalized_velocities_{0};
  int num_misc_continuous_states_{0};

  // A model discrete state to be used during Context allocation.
  std::unique_ptr<BasicVector<T>> model_discrete_state_vector_;

  // A model abstract state to be used during Context allocation.
  detail::ModelValues model_abstract_states_;

  // Model inputs to be used in AllocateInput{Vector,Abstract}.
  detail::ModelValues model_input_values_;

  // Model numeric parameters to be used during Context allocation.
  detail::ModelValues model_numeric_parameters_;

  // Model abstract parameters to be used during Context allocation.
  detail::ModelValues model_abstract_parameters_;
};

}  // namespace systems
}  // namespace drake
