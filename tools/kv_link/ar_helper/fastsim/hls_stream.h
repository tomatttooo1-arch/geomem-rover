// AR2NANO fast stand-in for the Vitis HLS hls_stream.h (C execution of a DATAFLOW region is
// sequential: the producer fills the whole stream before the consumer runs). Subset: named
// constructor, write, read, empty. Reading an empty stream aborts (the Xilinx C model warns and
// returns a default value; the AR core asserts both streams end empty, so neither happens).
#ifndef AR2NANO_FAST_HLS_STREAM_H
#define AR2NANO_FAST_HLS_STREAM_H
#include <cstdlib>
#include <vector>
namespace hls {
template <typename T> class stream {
public:
    stream() : head_(0) {}
    explicit stream(const char*) : head_(0) {}
    void write(const T& v) { q_.push_back(v); }
    T read() {
        if (head_ >= q_.size()) std::abort();
        T v = q_[head_++];
        if (head_ == q_.size()) { q_.clear(); head_ = 0; }
        return v;
    }
    bool empty() const { return head_ >= q_.size(); }
private:
    std::vector<T> q_;
    std::size_t head_;
};
}
#endif
