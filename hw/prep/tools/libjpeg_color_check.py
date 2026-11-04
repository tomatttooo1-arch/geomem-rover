import io, glob, numpy as np, PIL
from PIL import Image, features
print("PIL", PIL.__version__, "libjpeg_turbo", features.check_feature("libjpeg_turbo"), features.version_feature("libjpeg_turbo"))
def fix(x): return int(x*65536+0.5)
x=np.arange(256)-128
crr=(fix(1.402)*x+32768)>>16; cbb=(fix(1.772)*x+32768)>>16; crg=-fix(0.71414)*x; cbg=-fix(0.34414)*x+32768
tot=0; bad=0
for p in sorted(glob.glob("${FPGA_RUN}/DPUPIPE1/data/mix16/*.png"))[:8]:
    for q in (75,95):
        b=io.BytesIO(); Image.open(p).convert("RGB").save(b,"JPEG",quality=q,subsampling=1)  # 4:2:2
        d=b.getvalue()
        rgb=np.asarray(Image.open(io.BytesIO(d)).convert("RGB")).astype(int)
        im=Image.open(io.BytesIO(d)); im.draft("YCbCr",im.size); ycc=np.asarray(im).astype(int)
        assert im.mode=="YCbCr", im.mode
        Y,Cb,Cr=ycc[...,0],ycc[...,1],ycc[...,2]
        R=np.clip(Y+crr[Cr],0,255); G=np.clip(Y+((cbg[Cb]+crg[Cr])>>16),0,255); B=np.clip(Y+cbb[Cb],0,255)
        m=np.stack([R,G,B],-1)
        bad+=int((m!=rgb).sum()); tot+=m.size
print("pixels-bytes", tot, "mismatch", bad)
