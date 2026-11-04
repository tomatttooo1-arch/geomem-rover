"""Host: judge the board TRT detections against the host ORT reference (PREREG.md section 2).

  d5 step  : python3.13 -m tools.yolo_trt_board.compare_board d5 <ref json> <board d5 json> <out json>
             -> d5 = max |score_trt - score_ref| over paired detections in the first 5 frames,
                T = max(0.05, 2*d5). Write d5 and T into PREREG's revision section, then run full.
  full step: python3.13 -m tools.yolo_trt_board.compare_board full <ref json> <board full json> <out json>
                 --gate T [--d5 <d5 judge json>] [--latency <latency_summary.json>] [--env <board_env.json>]
             -> B1-B4 on 39 frames, K1/K2/K3 instrument checks, report-only items.
A board file written with the fake TensorRT stand-in (host dry run) is never judged as real:
the verdict is "SELFTEST_ONLY" unless --allow-fake is given, and even then it is labelled.
"""
import argparse
import json
import sys

from tools.yolo_trt_board.matching import brief, compare, judge

ENGINE_SHA = "0a111806558c33df20c656818c7b486d8f9193f643c248e47962116cbc288fd6"


def load(ref_path, board_path, n_expected):
    ref, board = json.load(open(ref_path)), json.load(open(board_path))
    frames = board["frames"]
    problems = []
    if len(frames) != n_expected:
        problems.append("board has %d frames, expected %d" % (len(frames), n_expected))
    if frames != ref["frames"][:len(frames)]:
        problems.append("board frame list is not the reference's first %d frames" % len(frames))
    for n in frames:
        if board["frame_sha256"].get(n) != ref["frame_sha256"].get(n):
            problems.append("frame sha256 differs: %s" % n)
    return ref, board, frames, problems


def provenance(board, allow_fake, engine_sha=ENGINE_SHA):
    ex = board.get("execution", {})
    prov = {"fake_trt": bool(board.get("fake_trt")), "backend": ex.get("backend"), "precision": ex.get("precision"),
            "engine_sha256": board.get("engine_sha256"), "engine_sha_expected": engine_sha,
            "engine_sha_ok": board.get("engine_sha256") == engine_sha, "classes": ex.get("classes"),
            "conf": ex.get("conf"), "iou": ex.get("iou"), "max_det": ex.get("max_det")}
    prov["real_board_record"] = (not prov["fake_trt"]) and prov["backend"] == "trt" and prov["engine_sha_ok"]
    prov["config_ok"] = (prov["classes"] == "all" and prov["conf"] == 0.25 and prov["iou"] == 0.7
                         and prov["max_det"] == 100)
    prov["judgeable"] = prov["config_ok"] and (prov["real_board_record"] or allow_fake)
    return prov


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=("d5", "full"))
    ap.add_argument("ref"); ap.add_argument("board"); ap.add_argument("out")
    ap.add_argument("--gate", type=float, help="full: B3 gate T from PREREG's revision section")
    ap.add_argument("--d5", help="full: the d5 judge json; T must equal max(0.05, 2*d5)")
    ap.add_argument("--latency"); ap.add_argument("--env")
    ap.add_argument("--allow-fake", action="store_true", help="host dry run only")
    ap.add_argument("--engine-sha", default=ENGINE_SHA,
                    help="expected engine sha256 (default: the 09-27 engine; 2026-10-15 rebuild passes the new one)")
    a = ap.parse_args(argv)
    n_expected = 5 if a.step == "d5" else 39
    ref, board, frames, problems = load(a.ref, a.board, n_expected)
    prov = provenance(board, a.allow_fake, a.engine_sha)
    per = compare(frames, ref["detections"], board["detections"])
    res = {"step": a.step, "ref": a.ref, "board": a.board, "provenance": prov, "input_problems": problems}
    if a.step == "d5":
        pairs = [p for f in per for p in f["pairs"]]
        d5 = max((p["dscore"] for p in pairs), default=None)
        res["pairs"] = len(pairs)
        res["d5"] = d5
        res["T"] = None if d5 is None else max(0.05, 2 * d5)
        res["info_5_frames"] = brief(judge(per, score_gate=res["T"] if res["T"] is not None else 0.05,
                                           min_pos_frames=0, min_pairs=0))
        res["note"] = "d5 step: B1/B2 on 5 frames are information only; write d5 and T into PREREG, then run full"
        res["usable"] = d5 is not None and not problems and prov["judgeable"]
    else:
        if a.gate is None:
            sys.exit("--gate T (from PREREG's revision section) is required for the full step")
        T = a.gate
        if a.d5:
            d5j = json.load(open(a.d5))
            res["d5_from_file"] = d5j["d5"]
            res["gate_matches_d5_rule"] = abs(T - max(0.05, 2 * d5j["d5"])) < 1e-12
            first5 = compare(frames[:5], ref["detections"], board["detections"])
            p5 = [p["dscore"] for f in first5 for p in f["pairs"]]
            res["full_first5_max_dscore"] = max(p5, default=None)
            res["full_first5_equals_d5_run"] = res["full_first5_max_dscore"] == d5j["d5"]
        main_j = judge(per, score_gate=T)
        res["main"] = main_j
        k1 = judge(compare(frames, ref["detections"], ref["detections"]), score_gate=T)
        shifted = {n: [dict(d, box=[d["box"][0] + 20, d["box"][1], d["box"][2] + 20, d["box"][3]])
                       for d in board["detections"][n]] for n in frames}
        k2 = judge(compare(frames, ref["detections"], shifted), score_gate=T)
        bumped = {n: [dict(d, score=d["score"] + T + 0.05) for d in board["detections"][n]] for n in frames}
        k3 = judge(compare(frames, ref["detections"], bumped), score_gate=T)
        res["K1_ref_vs_ref"] = brief(k1)
        res["K2_shift20"] = brief(k2)
        res["K3_score_plus_T_plus_0.05"] = brief(k3)
        res["instrument_ok"] = (k1["PASS"] and k1["pairs"] == k1["ref_detections"]
                                and not k2["B2_iou_ge_gate"] and not k3["B3_dscore_le_gate"])
        res["report_only"] = {"count_mismatch_frames": main_j["count_mismatch_frames"],
                              "passes": board.get("passes"),
                              "passes_identical_to_pass1": board.get("passes_identical_to_pass1"),
                              "pass_mismatches": board.get("pass_mismatches")}
        if a.latency:
            res["latency"] = json.load(open(a.latency))
        if a.env:
            env = json.load(open(a.env))
            res["board_env"] = {k: env.get(k) for k in ("nvpmodel_q", "tensorrt", "torch", "l4t", "engine_io",
                                                       "detector_outputs_used", "gpu_before", "gpu_after", "load_s",
                                                       "cv2_threads", "torch_threads", "cuda_ctx", "uptime")}
        ok = (main_j["PASS"] and res["instrument_ok"] and not problems and prov["judgeable"]
              and res.get("gate_matches_d5_rule", True))
        if not prov["config_ok"] or problems or not res["instrument_ok"]:
            res["VERDICT"] = "INVALID"
        elif not main_j["B4_positive_control"]:
            res["VERDICT"] = "MEANINGLESS"
        elif prov["fake_trt"] or not prov["real_board_record"]:
            res["VERDICT"] = "SELFTEST_ONLY_" + ("PASS" if ok else "FAIL")
        else:
            res["VERDICT"] = "PASS" if ok else "FAIL"
    with open(a.out, "w") as f:
        json.dump(res, f, indent=1)
    short = {k: v for k, v in res.items() if k not in ("main", "latency", "board_env")}
    if "main" in res:
        short["main"] = brief(res["main"])
    print(json.dumps(short, indent=1))


if __name__ == "__main__":
    main()
