// syntax-check stub only (no glog headers on this host); NOT for building.
#pragma once
#include <iostream>
namespace google { struct LogMessageVoidify { void operator&(std::ostream&) {} }; }
#define LOG(s) std::cerr
#define LOG_IF(s, c) std::cerr
#define CHECK(c) std::cerr
#define CHECK_EQ(a, b) std::cerr
#define CHECK_NE(a, b) std::cerr
#define CHECK_LT(a, b) std::cerr
#define CHECK_LE(a, b) std::cerr
#define CHECK_GT(a, b) std::cerr
#define CHECK_GE(a, b) std::cerr
