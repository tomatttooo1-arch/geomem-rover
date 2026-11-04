// DRV5 (2026-11-04): dpu_rx's PL pre-processing adapter (pl_prep.cpp) -- the parts host tests and the board smoke
// tool need besides make_pl_prep() (colour_prep.h).
#pragma once
#include <memory>
#include <string>

#include "../../hw/m7/pl/sw/pl_io.hpp"
#include "../../hw/prep/drv/prep_core.hpp"
#include "colour_prep.h"

namespace dpupipe {

// The core configuration dpu_rx uses for a src_w x src_h YUYV camera frame -> size x size letterbox tensor:
// content box from letterbox() (same as SoftPrep), source stride = w*2 rounded up to 16 (the driver copies rows into
// its own buffer), bypass when no resize is needed.
prep_drv::Config pl_prep_config(int src_w, int src_h, int size, int fix_point, int kset, int ups);

// make_pl_prep on a given bus (host mock / board smoke); bus == nullptr opens the hardware like make_pl_prep.
std::unique_ptr<ColourPrep> make_pl_prep_on(std::unique_ptr<plio::Bus> bus, int src_w, int src_h, int size,
                                            int fix_point, int kset, int ups, std::string* why);
// The driver inside a ColourPrep returned by make_pl_prep / make_pl_prep_on (nullptr for any other ColourPrep).
prep_drv::Core* pl_prep_core(ColourPrep* p);

}  // namespace dpupipe
