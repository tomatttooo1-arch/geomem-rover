"""Repackage the TWO preregistered goldens, without rerunning reference code."""
import argparse, hashlib, json, struct
from pathlib import Path
def sha(b): return hashlib.sha256(b).hexdigest()
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for name in ('old-vectors','profile','out'): ap.add_argument('--'+name,type=Path,required=True)
    a=ap.parse_args()
    src=a.old_vectors
    old_in=(src/'input.bin').read_bytes(); old_go=(src/'expected.bin').read_bytes()
    assert sha(old_in)=='dacb87a5a13853cf66a44c8f93b71d1dc4209a1deb5d66a69a3cec4279d49116'
    assert sha(old_go)=='0492e0187111c723268aa72593e802da7ff24797b9f966567a2bbf67ed3aae13'
    assert old_in[:8]==b'FPGAFIN01' and old_go[:8]==b'FPGAFGO01'
    local_in=old_in[12:6432]; local_go=old_go[12:19288]
    field80=b''.join(struct.pack('<I',int.from_bytes(local_go[i:i+3],'little')) for i in range(12,19212,3))
    flags80=b''.join(struct.pack('<I',x) for x in local_go[19212:])
    c=a.profile/'classroom'
    map200=(c/'map.u8').read_bytes(); phi200=(c/'reference.field.u32le').read_bytes()
    changed=(c/'reference.changed.u8').read_bytes()
    assert sha(map200)=='ecb6a2a8a876fb0b6d28524a21f476efd8ef237f90d0dc072af0b318e531937f'
    assert sha(phi200)=='a32dbcb059fe42476dabe860dd3995bb74431891ffc1062954cc16191aded287'
    assert sha(changed)=='d029ad93978df7e6ee64f4f3b5e5be70df48ce3cbee6d56fd5651ff4bcfa7323'
    info=json.loads((c/'input.json').read_text())
    assert info['start']==[158,108] and info['goal']==[40,118] and len(changed)==6
    flags200=b''.join(struct.pack('<I',x) for x in changed+bytes([255])*250)
    body=local_go[:12]+field80+flags80+struct.pack('<III',0,6,1)+phi200+flags200
    ibody=struct.pack('<I',0)+local_in+struct.pack('<I5i',1,158,108,40,118,256)+map200
    a.out.mkdir(parents=True,exist_ok=False)
    (a.out/'input.bin').write_bytes(b'FPGADIN01'+struct.pack('<I',2)+ibody)
    (a.out/'expected.bin').write_bytes(b'FPGADGO01'+struct.pack('<I',2)+body)
    (a.out/'golden').mkdir(); (a.out/'golden'/'outputs.bin').write_bytes(body)
    manifest=dict(cases=['old80_all211_16rounds','classroom200_6rounds'],modes=[0,1],k_max=[64,256],
                  old_vectors=str(src.resolve()),profile=str(a.profile.resolve()),reference_execution='none: repackage frozen outputs only',
                  words=len(body)//4,bytes=len(body),output_sha256=sha(body),
                  sha256={p.name:sha(p.read_bytes()) for p in (a.out/'input.bin',a.out/'expected.bin')})
    (a.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest))
if __name__=='__main__': main()
