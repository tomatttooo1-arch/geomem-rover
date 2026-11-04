# 阴性对照：把 trace 第 rec 条记录的期望镜像（after）第 off 字节翻一位
import struct,sys
src,dst,rec,boff=sys.argv[1],sys.argv[2],int(sys.argv[3]),int(sys.argv[4],0)
b=bytearray(open(src,'rb').read()); m,n,nb,p=struct.unpack('<4sIII',b[:16]); off=16
for r in range(rec): off+=8+2*nb
b[off+8+nb+boff]^=1; open(dst,'wb').write(b); print('flipped rec',rec,'op',struct.unpack('<I',b[off:off+4])[0],'byte',hex(boff))
