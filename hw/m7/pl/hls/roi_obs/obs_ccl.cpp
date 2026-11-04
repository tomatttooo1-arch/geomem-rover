#include "obs_ccl.hpp"
#ifndef __SYNTHESIS__
#include <cassert>
#endif
namespace roi_obs {
namespace {
static ap_uint<36> node(ap_uint<15> parent, ap_uint<17> area, ap_uint<4> rank) {
#pragma HLS INLINE
    ap_uint<36> n=0; n.range(14,0)=parent; n.range(31,15)=area;
    n.range(35,32)=rank; return n;
}
static ap_uint<15> parent_of(ap_uint<36> n) {
#pragma HLS INLINE
    return n.range(14,0);
}
static ap_uint<17> area_of(ap_uint<36> n) {
#pragma HLS INLINE
    return n.range(31,15);
}
static ap_uint<4> rank_of(ap_uint<36> n) {
#pragma HLS INLINE
    return n.range(35,32);
}
// Values are encoded in six bits. No state-indexed decode ROM: this switch
// describes one reused control FSM rather than multiple find/union engines.
enum State { SCAN, RUN_BEGIN, PREV_REQUEST, PREV_WAIT, PREV_CHECK,
    FIND_REQUEST, FIND_CHECK, OVERLAP_FOUND, UNION_CHILD_DONE,
    UNION_WINNER_DONE, FINISH_RUN, RUN_AREA_DONE, EMIT_GAP, EMIT_RUN,
    ROW_FLUSH, NEXT_ROW, MAX_REQUEST, MAX_CHECK, FLAT_BEGIN, FLAT_FOUND,
    FLAT_WRITTEN, P_REQUEST, P_WAIT, P_DELIVER, RMW_COMMIT, DRAIN, DONE };
}
void ccl_fast(hls::stream<ap_uint<16> >& depth,
              ap_uint<11> width, ap_uint<10> height,
              ap_uint<17> lower, ap_uint<17> upper,
              ap_uint<64> labels[16384],
              ap_uint<72> parents[16384],
              ap_uint<38> runs[1280], CclSummary& result) {
#pragma HLS INLINE off
    // Port discipline: one syntactic read and one write for parents; one read
    // and one write for run rows; one write for label plane. A packed parent
    // modification ALWAYS obtains a fresh 72-bit word, even when its other
    // lane was read by the preceding find/union. No false dependence pragma.
    ap_uint<21> total=ap_uint<21>(width)*height, consumed=0;
    bool error=(width==0 || width>1280 || height==0 || height>800 || total>65536);
    ap_uint<6> state=error?DRAIN:SCAN, read_return=FIND_CHECK, write_return=SCAN;
    ap_uint<11> x=0, run_left=0, run_right=0;
    ap_uint<10> y=0;
    ap_uint<17> row_base=0;
    bool in_run=false, row_finished=false, parity=false, connected=false;
    ap_uint<10> previous_count=0, current_count=0, first_previous=0, search_previous=0;
    ap_uint<11> run_read_address=0;
    ap_uint<38> run_read_word=0;
    ap_uint<16> births=0, scan_node=0;
    ap_uint<17> maximum=0;
    ap_uint<15> root_a=0, found_root=0, find_node=0, parent_index=0;
    ap_uint<36> node_a=0, found_record=0, first_record=0, pending_record=0;
    ap_uint<5> find_steps=0;
    ap_uint<6> find_return=OVERLAP_FOUND;
    // Fixed data response pipeline; control states guarantee t+3 consume.
    // Request-valid tracking is a C-simulation assertion, not hardware state.
    ap_uint<72> parent_rsp0=0, parent_rsp1=0, parent_rsp2=0;
#ifndef __SYNTHESIS__
    bool parent_valid0=false, parent_valid1=false, parent_valid2=false;
#endif
    ap_uint<17> output_position=0, segment_end=0;
    ap_uint<16> segment_label=0;
    ap_uint<64> label_pack=0;
    bool output_for_run=false;

    CCL_FSM: while (state!=DONE) {
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=parents type=inter direction=RAW dependent=true distance=2
#pragma HLS LOOP_TRIPCOUNT min=1 max=12000000
        bool parent_read=false, parent_write=false, run_read=false, run_write=false;
        bool label_write=false;
        ap_uint<14> parent_address=parent_index.range(14,1);
        ap_uint<72> parent_write_word=0;
        ap_uint<11> run_write_address=0;
        ap_uint<38> run_write_word=0;
        ap_uint<14> label_write_address=0;
        ap_uint<64> label_write_word=0;
        const ap_uint<36> read_node=parent_index[0]?ap_uint<36>(parent_rsp2.range(71,36)):
                                                                  ap_uint<36>(parent_rsp2.range(35,0));
        switch (state) {
        case SCAN: {
            const ap_uint<16> code=depth.read(); ++consumed;
            const bool foreground=(code>=101 && code<10000 &&
                                   ap_uint<17>(code)>=lower && ap_uint<17>(code)<upper);
            const bool last=(x+1==width);
            row_finished=last;
            if (foreground && !in_run) { run_left=x; in_run=true; }
            if (in_run && (!foreground || last)) {
                run_right=foreground?x:ap_uint<11>(x-1);
                in_run=false; state=RUN_BEGIN;
            } else if (last) state=ROW_FLUSH;
            x=last?ap_uint<11>(0):ap_uint<11>(x+1);
            break;
        }
        case RUN_BEGIN:
            connected=false; search_previous=first_previous; state=PREV_REQUEST; break;
        case PREV_REQUEST:
            if (search_previous==previous_count) state=FINISH_RUN;
            else {
                run_read_address=(parity?ap_uint<11>(0):ap_uint<11>(640))+search_previous;
                run_read=true; state=PREV_WAIT;
            }
            break;
        case PREV_WAIT: state=PREV_CHECK; break;
        case PREV_CHECK: {
            ap_uint<11> left=run_read_word.range(10,0);
            ap_uint<11> right=run_read_word.range(21,11);
            ap_uint<16> positive=run_read_word.range(37,22);
            if (ap_uint<12>(right)+1<run_left) {
                ++search_previous; first_previous=search_previous; state=PREV_REQUEST;
            } else if (ap_uint<12>(left)>ap_uint<12>(run_right)+1) state=FINISH_RUN;
            else {
                ++search_previous; find_node=ap_uint<15>(positive-1);
                find_steps=0; find_return=OVERLAP_FOUND; state=FIND_REQUEST;
            }
            break;
        }
        case FIND_REQUEST:
            parent_index=find_node; read_return=FIND_CHECK; state=P_REQUEST; break;
        case FIND_CHECK: {
#ifndef __SYNTHESIS__
            assert(parent_valid2);
#endif
            const ap_uint<36> n=read_node;
            if (find_steps==0) first_record=n;
            if (parent_of(n)==find_node) {
                found_root=find_node; found_record=n; state=find_return;
            } else if (find_steps==15) { error=true; state=DRAIN; }
            else { find_node=parent_of(n); ++find_steps; state=FIND_REQUEST; }
            break;
        }
        case OVERLAP_FOUND:
            if (!connected) {
                connected=true; root_a=found_root; node_a=found_record; state=PREV_REQUEST;
            } else if (root_a==found_root) state=PREV_REQUEST;
            else {
                ap_uint<15> winner=root_a, loser=found_root;
                ap_uint<36> win=node_a, lose=found_record;
                if (rank_of(node_a)<rank_of(found_record)) {
                    winner=found_root; loser=root_a; win=found_record; lose=node_a;
                }
                ap_uint<4> rank=rank_of(win);
                if (rank_of(win)==rank_of(lose)) ++rank;
                root_a=winner; node_a=node(winner,ap_uint<17>(area_of(win)+area_of(lose)),rank);
                parent_index=loser; pending_record=node(winner,area_of(lose),rank_of(lose));
                read_return=RMW_COMMIT; write_return=UNION_CHILD_DONE; state=P_REQUEST;
            }
            break;
        case UNION_CHILD_DONE:
            parent_index=root_a; pending_record=node_a; read_return=RMW_COMMIT;
            write_return=UNION_WINNER_DONE; state=P_REQUEST; break;
        case UNION_WINNER_DONE: state=PREV_REQUEST; break;
        case FINISH_RUN: {
            const ap_uint<17> length=ap_uint<17>(run_right)-run_left+1;
            if (!connected) {
                if (births==32768) { error=true; state=DRAIN; break; }
                root_a=ap_uint<15>(births); ++births;
                node_a=node(root_a,length,0);
            } else node_a=node(root_a,ap_uint<17>(area_of(node_a)+length),rank_of(node_a));
            parent_index=root_a; pending_record=node_a; read_return=RMW_COMMIT;
            write_return=RUN_AREA_DONE; state=P_REQUEST; break;
        }
        case RUN_AREA_DONE:
            if (current_count==640) { error=true; state=DRAIN; }
            else {
                run_write_address=(parity?ap_uint<11>(640):ap_uint<11>(0))+current_count;
                run_write_word.range(10,0)=run_left; run_write_word.range(21,11)=run_right;
                run_write_word.range(37,22)=ap_uint<16>(root_a)+1; run_write=true;
                ++current_count; segment_end=row_base+run_left; segment_label=0;
                output_for_run=true; state=EMIT_GAP;
            }
            break;
        case EMIT_GAP:
        case EMIT_RUN:
        case ROW_FLUSH: {
            // At most four pixels of one constant-label segment per tick.
            // This packs each lane once; no depth replay and no ROI pixel cache.
            if (state==ROW_FLUSH) { segment_end=row_base+width; segment_label=0; output_for_run=false; }
            if (output_position<segment_end) {
                ap_uint<3> lane=output_position.range(1,0);
                ap_uint<17> remaining=segment_end-output_position;
                ap_uint<3> capacity=4-lane;
                ap_uint<3> take=remaining<capacity?ap_uint<3>(remaining):capacity;
                ap_uint<64> next_pack=label_pack;
                for (unsigned k=0;k<4;++k) {
#pragma HLS UNROLL
                    if (k>=lane && k<lane+take) next_pack.range(16*k+15,16*k)=segment_label;
                }
                label_pack=next_pack;
                if (lane+take==4) {
                    label_write=true; label_write_address=output_position.range(15,2);
                    label_write_word=next_pack; label_pack=0;
                }
                output_position+=take;
            } else if (state==EMIT_GAP && output_for_run) {
                segment_end=row_base+run_right+1; segment_label=ap_uint<16>(root_a)+1; state=EMIT_RUN;
            } else if (state==EMIT_RUN) state=row_finished?ROW_FLUSH:SCAN;
            else state=NEXT_ROW;
            break;
        }
        case NEXT_ROW:
            previous_count=current_count; current_count=0; first_previous=0; parity=!parity;
            row_base+=width; ++y;
            if (y==height) {
                if (output_position.range(1,0)!=0) {
                    label_write=true; label_write_address=output_position.range(15,2);
                    label_write_word=label_pack;
                }
                scan_node=0; state=MAX_REQUEST;
            } else state=SCAN;
            break;
        case MAX_REQUEST:
            if (scan_node==births) { scan_node=0; state=FLAT_BEGIN; }
            else { parent_index=ap_uint<15>(scan_node); read_return=MAX_CHECK; state=P_REQUEST; }
            break;
        case MAX_CHECK: {
#ifndef __SYNTHESIS__
            assert(parent_valid2);
#endif
            const ap_uint<36> n=read_node;
            if (parent_of(n)==scan_node && area_of(n)>maximum) maximum=area_of(n);
            ++scan_node; state=MAX_REQUEST; break;
        }
        case FLAT_BEGIN:
            if (scan_node==births) state=DONE;
            else { find_node=ap_uint<15>(scan_node); find_steps=0; find_return=FLAT_FOUND; state=FIND_REQUEST; }
            break;
        case FLAT_FOUND:
            parent_index=ap_uint<15>(scan_node);
            pending_record=node(found_root,area_of(first_record),ap_uint<4>(area_of(found_record)==maximum));
            read_return=RMW_COMMIT; write_return=FLAT_WRITTEN; state=P_REQUEST; break;
        case FLAT_WRITTEN: ++scan_node; state=FLAT_BEGIN; break;
        case P_REQUEST: parent_read=true; state=P_WAIT; break;
        case P_WAIT: state=P_DELIVER; break;
        case P_DELIVER: state=read_return; break;
        case RMW_COMMIT:
#ifndef __SYNTHESIS__
            assert(parent_valid2);
#endif
            parent_write_word=parent_rsp2;
            if (parent_index[0]) parent_write_word.range(71,36)=pending_record;
            else parent_write_word.range(35,0)=pending_record;
            parent_write=true; state=write_return; break;
        case DRAIN:
            if (consumed<total) { (void)depth.read(); ++consumed; }
            else state=DONE;
            break;
        default: error=true; state=DRAIN; break;
        }
        // Each memory read/write has exactly one call site. Requests and
        // consumers are separated by explicit states for the URAM latency.
        // Request only in P_REQUEST; zero-fill other iterations instead of
        // issuing an unused read or introducing a RAM-data hold feedback.
        // Closest real write->read is COMMIT, CHILD_DONE, REQUEST: distance 2.
        parent_rsp2=parent_rsp1;
        parent_rsp1=parent_rsp0;
        parent_rsp0=parent_read ? parents[parent_address] : ap_uint<72>(0);
#ifndef __SYNTHESIS__
        parent_valid2=parent_valid1;
        parent_valid1=parent_valid0;
        parent_valid0=parent_read;
#endif
        if (parent_write) parents[parent_address]=parent_write_word;
        if (run_read) run_read_word=runs[run_read_address];
        if (run_write) runs[run_write_address]=run_write_word;
        if (label_write) labels[label_write_address]=label_write_word;
    }
    result.births=births; result.max_area=maximum; result.error=error;
}
}
