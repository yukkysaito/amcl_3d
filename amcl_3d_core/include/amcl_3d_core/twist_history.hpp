#pragma once

#include <iterator>
#include <map>

#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Piecewise constant body twists over time, inserted in any order. A twist is valid from its
/// stamp until the next one; before the first twist the vehicle is assumed to stand still.
class TwistHistory
{
public:
  /// Inserts a twist valid from `stamp`; replaces a twist with the same stamp.
  void insert(const Time & stamp, const Twist & twist) { twists_[stamp.nanoseconds] = twist; }

  /// Calls piece(twist, dt) for the consecutive constant pieces covering [from, to).
  template <typename Piece>
  void integrate(const Time & from, const Time & to, const Piece & piece) const
  {
    auto next = twists_.upper_bound(from.nanoseconds);  // first twist starting after `from`
    Twist active = next == twists_.begin() ? Twist{} : std::prev(next)->second;
    Time t = from;
    while (t < to) {
      Time end = to;
      if (next != twists_.end() && next->first < to.nanoseconds) {
        end = Time{next->first};
      }
      piece(active, end - t);
      t = end;
      if (next != twists_.end() && next->first <= t.nanoseconds) {
        active = next->second;
        ++next;
      }
    }
  }

  /// Drops the twists not needed to integrate from `stamp` on (keeps the one active at `stamp`).
  void pruneBefore(const Time & stamp)
  {
    auto active = twists_.upper_bound(stamp.nanoseconds);
    if (active != twists_.begin()) {
      twists_.erase(twists_.begin(), std::prev(active));
    }
  }

  bool empty() const { return twists_.empty(); }
  size_t size() const { return twists_.size(); }
  /// Stamp of the newest twist; requires !empty().
  Time newestStamp() const { return Time{twists_.rbegin()->first}; }

private:
  std::map<int64_t, Twist> twists_;
};

}  // namespace amcl_3d
