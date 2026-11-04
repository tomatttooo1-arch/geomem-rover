"""Small diagnostic helpers; proc accounting is separate from heap ownership."""
import json
from pathlib import Path
import socket
import time
import xml.etree.ElementTree as ET


def allocator_request(path, command='snapshot'):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
        client.settimeout(3.)
        client.connect(str(path))
        client.sendall(command.encode('ascii'))
        result=json.loads(client.recv(8192))
    if not result['ok']:
        raise RuntimeError('allocator snapshot failed')
    return result


def allocator_totals(path):
    root=ET.parse(path).getroot()
    # Root totals already sum all arenas. Do not sum them again with heap/*.
    totals={r.attrib['type']:int(r.attrib['size']) for r in root.findall('total')}
    system={r.attrib['type']:int(r.attrib['size']) for r in root.findall('system')}
    free=totals.get('fast',0)+totals.get('rest',0)
    return {'arenas':len(root.findall('heap')), 'arena_system_bytes':system.get('current',0),
            'arena_free_bytes':free,'mmap_bytes':totals.get('mmap',0),
            'in_use_proxy_bytes':system.get('current',0)-free+totals.get('mmap',0),
            'limitation':'allocator accounting proxy, including allocator metadata/tcache; not exact live objects'}


def proc_sample(pid):
    root=Path('/proc')/str(pid)
    fields={}
    for name in ('status','smaps_rollup'):
        for line in (root/name).read_text().splitlines():
            key,sep,value=line.partition(':')
            if sep and value.strip().endswith(' kB'):
                fields[key+'_kib']=int(value.split()[0])
    fields['schedstat']=(root/'schedstat').read_text().strip()
    fields['pid']=pid
    return fields


def telemetry(processes, directory, sample_number):
    result=[]
    for name,proc in processes:
        try:
            row={'name':name,**proc_sample(proc.pid)}
            if sample_number%30==0:
                (directory/f'{name}-{sample_number:06d}.smaps').write_text(Path(f'/proc/{proc.pid}/smaps').read_text())
            result.append(row)
        except (FileNotFoundError, ProcessLookupError):
            result.append({'name':name,'pid':proc.pid,'exited':True})
    # Container-local cgroup counters: throttling is not an algorithm duration.
    cpu={}
    for path in ('/sys/fs/cgroup/cpu.stat','/sys/fs/cgroup/cpu/cpu.stat','/proc/pressure/cpu'):
        if Path(path).exists():cpu[path]=Path(path).read_text()
    return {'monotonic_ns':time.monotonic_ns(),'processes':result,'cpu_counters':cpu}
