/*
 * Injected only when building bp_for_demo (see CMakeLists.txt).
 * Supplies std headers some BulletPhysics .cpp rely on transitively on libc++.
 * Does not modify files under BulletPhysics/.
 */
#pragma once
#include <algorithm>
#include <string>
