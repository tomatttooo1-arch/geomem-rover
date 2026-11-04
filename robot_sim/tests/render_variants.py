#!/usr/bin/env python3
"""Generate a registered 2x2 visual-input probe; never an online label source."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--source-revision",required=True)
    args=parser.parse_args()
    source=Path(__file__).resolve().parents[1]/"fpga_sim/worlds/room.sdf"
    if args.output.exists():raise SystemExit("Refusing to overwrite an existing variant run")
    args.output.mkdir(parents=True)
    cases=[]
    for high_view in [False,True]:
        for bright in [False,True]:
            tree=ET.parse(source); world=tree.find("world")
            camera=world.find(".//sensor[@type='rgbd_camera']")
            camera.find("pose").text=("0.18 0 0.95 0 0.25 0" if high_view
                                      else "0.18 0 0.55 0 0 0")
            # Sensor extrinsics change only. Robot body/scene props identical.
            if bright:
                scene=world.find("scene")
                if scene is None:scene=ET.SubElement(world,"scene")
                ambient=scene.find("ambient")
                if ambient is None:ambient=ET.SubElement(scene,"ambient")
                ambient.text="0.8 0.8 0.8 1"
                light=world.find("light[@name='sun']")
                light.find("diffuse").text="1 1 1 1"
                light.find("specular").text="0.2 0.2 0.2 1"
                light.find("direction").text="0.4 0.0 -1.0"
                attenuation=light.find("attenuation")
                attenuation.find("constant").text="1.0"
                attenuation.find("linear").text="0.0"
                attenuation.find("quadratic").text="0.0"
            key=("high" if high_view else "low")+"_"+("bright" if bright else "original")
            path=args.output/(key+".sdf")
            ET.indent(tree);tree.write(path,encoding="unicode",xml_declaration=True)
            cases.append({"name":key,"world":path.name,"camera_pose":camera.findtext("pose"),
                          "high_view":high_view,"bright_light":bright,
                          "sha256":hashlib.sha256(path.read_bytes()).hexdigest()})
    manifest={"source_revision":args.source_revision,"source_world_sha256":hashlib.sha256(source.read_bytes()).hexdigest(),
              "purpose":"2x2 view and illumination probe with unchanged props and detector threshold",
              "navigation_disabled":True,"camera_tf_valid_only_in_case_metadata":True,"cases":cases}
    (args.output/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print(json.dumps(manifest,indent=2))


if __name__=="__main__":
    main()
