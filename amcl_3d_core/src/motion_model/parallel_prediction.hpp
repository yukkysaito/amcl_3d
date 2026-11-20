// Internal helper shared by the motion models.
#pragma once

#include <cstddef>
#include <cstdint>

#include "amcl_3d_core/particle_set.hpp"
#include "amcl_3d_core/random.hpp"
#include "parallel_for.hpp"

namespace amcl_3d
{
/// Replaces every particle pose p_i by move(p_i, stream_i) on `thread_num` threads (0: all).
/// Each particle draws its noise from its own stream (parallelStream() with one key drawn from
/// `rng`), so the result does not depend on the thread count.
template <typename MovePose>
void moveParticlesInParallel(
  ParticleSet & particles, RandomGenerator & rng, size_t thread_num, const MovePose & move)
{
  const uint64_t key = drawKey(rng);
  parallelFor(static_cast<int64_t>(particles.size()), thread_num, [&](int64_t i) {
    const auto index = static_cast<size_t>(i);
    RandomGenerator stream = parallelStream(key, index);
    particles.setPose(index, move(particles.pose(index), stream));
  });
}

}  // namespace amcl_3d
