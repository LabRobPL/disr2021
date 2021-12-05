#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <memory>
#include <vector>

#include "drake/automotive/gen/maliput_railcar_state.h"
#include "drake/lcmt_maliput_railcar_state_t.hpp"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

namespace drake {
namespace automotive {

/**
 * Translates between LCM message objects and VectorBase objects for the
 * MaliputRailcarState type.
 */
class MaliputRailcarStateTranslator final
    : public drake::systems::lcm::LcmAndVectorBaseTranslator {
 public:
  MaliputRailcarStateTranslator()
      : LcmAndVectorBaseTranslator(
            MaliputRailcarStateIndices::kNumCoordinates) {}
  std::unique_ptr<drake::systems::BasicVector<double>> AllocateOutputVector()
      const final;
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   drake::systems::VectorBase<double>* vector_base) const final;
  void Serialize(double time,
                 const drake::systems::VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const final;
};

}  // namespace automotive
}  // namespace drake
