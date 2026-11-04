import tempfile
from pathlib import Path
import unittest
from tools.rtabmap_memory_audit import window_summary
from tools.rtabmap_memory_probe import allocator_totals


class MemoryAuditTests(unittest.TestCase):
    def test_release_step_does_not_leak_across_window(self):
        samples=[{'monotonic_ns':t,'processes':[{'name':'localization','Rss_kib':rss,'Pss_kib':rss}]} for t,rss in [(0,100),(10,100),(20,20),(30,20)]]
        self.assertEqual(window_summary(samples,'localization',0,20)['Rss_kib'],100)
        self.assertEqual(window_summary(samples,'localization',20,40)['Rss_kib'],20)
        self.assertEqual(window_summary(samples,'localization',100,200)['samples'],0)
    def test_arena_totals_not_double_counted_and_mmap_separate(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'malloc.xml';p.write_text('<malloc><heap nr="0"><total type="rest" size="40"/></heap><heap nr="1"><total type="rest" size="30"/></heap><total type="fast" size="10"/><total type="rest" size="70"/><total type="mmap" size="50"/><system type="current" size="200"/></malloc>')
            a=allocator_totals(p)
            self.assertEqual(a['arena_free_bytes'],80);self.assertEqual(a['in_use_proxy_bytes'],170);self.assertEqual(a['arenas'],2)


if __name__=='__main__':unittest.main()
