"""Compare same-input deployments; does not assess detector ground-truth accuracy."""
import argparse
from collections import Counter
import json
from pathlib import Path


def compare(left, right):
    a = [json.loads(line) for line in (Path(left)/"events.jsonl").read_text().splitlines()]
    b = [json.loads(line) for line in (Path(right)/"events.jsonl").read_text().splitlines()]
    if [(x["frame_id"], x["timestamp"]) for x in a] != [(x["frame_id"], x["timestamp"]) for x in b]:
        raise ValueError("different acquisition sequences; comparison invalid")
    if (Path(left)/"input.jsonl").read_bytes() != (Path(right)/"input.jsonl").read_bytes():
        raise ValueError("different input manifests; comparison invalid")
    box_error, score_error = 0., 0.
    reordered = []
    state_changes = []
    observation_changes = []
    position_error = 0.
    paired_detections, paired_tracks = 0, 0
    for x, y in zip(a, b):
        dx, dy = x["detections"], y["detections"]
        if [d["label"] for d in dx] != [d["label"] for d in dy]:
            reordered.append(x["frame_id"])
        else:
            for u, v in zip(dx, dy):
                paired_detections += 1
                box_error = max(box_error, *(abs(p-q) for p,q in zip(u["bbox"],v["bbox"])))
                score_error = max(score_error, abs(u["score"]-v["score"]))
        if x["observation_count"] != y["observation_count"]:
            observation_changes.append(x["frame_id"])
        for strategy in x["strategies"]:
            tx, ty = x["strategies"][strategy]["tracks"], y["strategies"][strategy]["tracks"]
            signature = lambda rows: Counter((r["id"],r["label"],r["color"],r["state"],r["reason"]) for r in rows)
            if signature(tx) != signature(ty):
                state_changes.append({"frame_id":x["frame_id"],"strategy":strategy})
            mx, my = {r["id"]:r for r in tx}, {r["id"]:r for r in ty}
            for key in mx.keys() & my.keys():
                paired_tracks += 1
                position_error = max(position_error, *(abs(p-q) for p,q in zip(mx[key]["position_m"],my[key]["position_m"])))
    return {"frames":len(a), "same_input_manifest":True,
            "detection_order_or_class_changed_frames":reordered,
            "paired_detections":paired_detections,
            "max_box_coordinate_error_px_for_matching_order":box_error if paired_detections else None,
            "max_score_error_for_matching_order":score_error if paired_detections else None,
            "observation_count_changed_frames":observation_changes,
            "track_state_or_reason_changes":state_changes,
            "paired_tracks":paired_tracks,
            "max_position_coordinate_error_m_for_matching_ids":position_error if paired_tracks else None,
            "scope":"deployment consistency only; matching order/IDs are not instance truth; no task accuracy or speed claim"}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("left");parser.add_argument("right");parser.add_argument("--out",required=True)
    args=parser.parse_args()
    result=compare(args.left,args.right)
    with open(args.out,"x") as out: json.dump(result,out,indent=2)
    print(json.dumps(result))


if __name__=="__main__": main()
