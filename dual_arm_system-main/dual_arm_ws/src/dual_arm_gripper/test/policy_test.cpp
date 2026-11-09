#include "dual_arm_gripper/policy.hpp"
#include <cassert>
#include <limits>
int main() {
  using namespace dual_arm_gripper;
  assert(parse("123,attach,cap_1").has_value());
  assert(parse("124,release").has_value());
  for (const auto *bad : {"", "0,release", "-1,release", "2,attach,cap_9", "2,attach,cap_1,extra", "2,release,cap_1", "2,attach,../cap_1"}) assert(!parse(bad));
  assert(contact_ok(0.,0.,1.));
  assert(!contact_ok(.002,0.,1.));
  assert(!contact_ok(0.,.02,1.));
  assert(!contact_ok(0.,0.,0.));
  assert(!contact_ok(std::numeric_limits<double>::quiet_NaN(),0.,1.));
}
