import json
from pathlib import Path
import struct
import tempfile
import unittest
from tools.planner_model.hls.batched_cosim import INPUT_SIZE,OUTPUT_SIZE,prepare,records


class FrozenBatchTest(unittest.TestCase):
    def fixture(self,root):
        v=root/'vectors';v.mkdir();(v/'golden_tb_out').mkdir()
        self.inputs=[bytes([i])*INPUT_SIZE for i in range(33)]
        self.outputs=[bytes([200-i])*OUTPUT_SIZE for i in range(33)]
        (v/'input.bin').write_bytes(b'FPGAFIN01'+struct.pack('<I',33)+b''.join(self.inputs))
        (v/'expected.bin').write_bytes(b'FPGAFGO01'+struct.pack('<I',33)+b''.join(self.outputs))
        (v/'golden_tb_out/outputs.bin').write_bytes(b''.join(self.outputs));return v

    def test_preserves_every_record_and_every_ordered_transition(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);v=self.fixture(root);out=root/'batches';prepare(v,out)
            m=json.loads((out/'manifest.json').read_text());observed={};pairs=set();calls=0
            for b in m['batches']:
                i=(out/b['folder']/'input.bin').read_bytes()[12:]
                o=(out/b['folder']/'expected.bin').read_bytes()[12:]
                self.assertEqual(i,b''.join(self.inputs[n] for n in b['indices']))
                self.assertEqual(o,b''.join(self.outputs[n] for n in b['indices']))
                for index in b['indices']:observed[index]=True
                pairs.update(zip(b['indices'],b['indices'][1:]));calls+=len(b['indices'])
            self.assertEqual(set(observed),set(range(33)))
            self.assertEqual(pairs,{(i,i+1) for i in range(32)});self.assertEqual(calls,38)

    def test_rejects_truncation_wrong_count_or_magic(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);v=self.fixture(root);p=v/'input.bin';original=p.read_bytes()
            for bad in [original[:-1],original+b'x',b'BADMAGIC'+original[8:],original[:8]+struct.pack('<I',32)+original[12:]]:
                p.write_bytes(bad)
                with self.assertRaises(ValueError):records(p,b'FPGAFIN01',INPUT_SIZE)

    def test_rejects_changed_frozen_golden(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);v=self.fixture(root);p=v/'golden_tb_out/outputs.bin';raw=bytearray(p.read_bytes());raw[20]^=1;p.write_bytes(raw)
            with self.assertRaises(ValueError):prepare(v,root/'out')
