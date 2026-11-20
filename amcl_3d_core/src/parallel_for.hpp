// Internal helper: the one place that maps the thread_num parameters to OpenMP.
#pragma once

#include <cstddef>
#include <cstdint>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace amcl_3d
{
/// Calls body(i) for i in [0, count) on `thread_num` threads (0: all hardware threads).
/// Static scheduling: each index is processed by one thread, so results written per index do not
/// depend on the thread count.
template <typename Body>
void parallelFor(int64_t count, size_t thread_num, const Body & body)
{
#ifdef _OPENMP
  const int omp_thread_num = thread_num > 0 ? static_cast<int>(thread_num) : omp_get_max_threads();
#else
  static_cast<void>(thread_num);
#endif
#pragma omp parallel for schedule(static) num_threads(omp_thread_num)
  for (int64_t i = 0; i < count; ++i) {
    body(i);
  }
}

}  // namespace amcl_3d
