#pragma once
#include <cstdint>
// All returned vertices use costmap coordinates: integer (x,y) is a cell centre in NavFn coordinates.
// Caller supplies capacity >= width*height+2. Return vertices, 0 on failure, -1 invalid args.
extern "C" int wave_plan(const uint8_t*, int, int, int, int, int, int,
                         double*, double*, int, int, uint32_t*);
extern "C" int navfn_plan(const uint8_t*, int, int, int, int, int, int,
                          double*, double*, int, double, int);

// Last NavFn call diagnostics: stage, propagation return, formal path length,
// requested-goal potential legal, resolved target x/y, expanded-limit diagnostic path length.
extern "C" int navfn_debug(int field);
