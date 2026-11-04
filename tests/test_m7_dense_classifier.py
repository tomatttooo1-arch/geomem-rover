"""Known-answer controls for the supervised dense-run classifier (2026-09-26)."""

from pathlib import Path
import importlib.util
import json
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "classify_m7_dense_run", ROOT / "tools/classify_m7_dense_run.py")
CLS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CLS)
CASES = CLS.DENSE.EXPECTED_CASES


def case_line(index, steps_per_valid=10):
    name, valid = CASES[index]
    return ("DENSE_CASE index=%d name=%s pixels=4096 valid=%d unique=%d dda_steps=%d "
            "accepted=%d invalid=%d outside=0\n" %
            (index, name, valid, 0 if valid == 0 else 1, valid * steps_per_valid,
             valid, 4096 - valid))


def group(count, tb_pass):
    return "".join(case_line(i) for i in range(count)) + (
        "M7 dense frame testbench PASS\n" if tb_pass else "")


def progress(count):
    return "".join('// RTL Simulation : %d / 13 [100.00%%] @ "%d"\n' % (i, 1000 + i * 5000)
                   for i in range(count + 1))


def log(csim=13, csim_pass=True, capture=13, post=13, post_pass=True, rtl=13,
        cosim="PASS", post_group=None):
    text = "INFO: [SIM 211-2] *************** CSIM start ***************\n"
    text += group(csim, csim_pass)
    if csim_pass:
        text += "INFO: [SIM 211-1] CSim done with 0 errors.\n"
    else:
        return text + "ERROR: [SIM 211-100] 'csim_design' failed: nonzero return value.\n"
    text += "INFO: [SIM 211-3] *************** CSIM finish ***************\n"
    text += "INFO: [COSIM 212-302] Starting C TB testing ... \n" + group(capture, True)
    text += "INFO: [COSIM 212-333] Generating C post check test bench ...\n"
    text += "INFO: [COSIM 212-15] Starting XSIM ...\n" + progress(rtl)
    if post is None:
        return text
    text += "INFO: [COSIM 212-316] Starting C post checking ...\n"
    text += post_group if post_group is not None else group(post, post_pass)
    if cosim == "PASS":
        text += "INFO: [COSIM 212-1000] *** C/RTL co-simulation finished: PASS ***\n"
    elif cosim == "FAIL":
        text += ("ERROR: [COSIM 212-361] C TB post check failed, nonzero return value '1'.\n"
                 "ERROR: [COSIM 212-4] *** C/RTL co-simulation finished: FAIL ***\n")
    return text


def transactions(rows=13, first=82198):
    body = "".join("transaction %7d: %15d %15s\n" %
                   (i, first + i * 1000, "x" if i == rows - 1 else str(first + i * 1000 + 1))
                   for i in range(rows))
    return "                             latency        interval\n" + body


def make_run(root, text, exit_code=0, signal=None, tx=None, exit_record=True):
    run = Path(root)
    (run / "hls_k26_10ns/solution/sim/report/verilog").mkdir(parents=True)
    (run / "vitis_hls.log").write_text(text)
    if tx is not None:
        (run / "hls_k26_10ns/solution/sim/report/verilog/result.transaction.rpt").write_text(tx)
    if exit_record:
        (run / "exit.json").write_text(json.dumps({
            "popen_returncode": exit_code if signal is None else -15,
            "exit_code": None if signal else exit_code, "terminated_by_signal": signal,
            "elapsed_seconds": 1.0, "start": {}, "end": {}}))
    return run


class DenseClassifierCalibration(unittest.TestCase):
    def classify(self, **kwargs):
        with tempfile.TemporaryDirectory(prefix="m7-dense-cls-") as directory:
            text = kwargs.pop("text")
            return CLS.classify(make_run(Path(directory) / "run", text, **kwargs))

    def verdict(self, **kwargs):
        try:
            return self.classify(**kwargs)["verdict"]
        except CLS.InstrumentError:
            return "INSTRUMENT_ERROR"

    def test_pass_and_cycle_mutation_is_observed(self):
        result = self.classify(text=log(), tx=transactions())
        self.assertEqual(result["verdict"], "PASS")
        self.assertTrue(all(case["rtl_postcheck"] == "PASS" for case in result["cases"]))
        self.assertEqual(result["cycle_table"][1]["incremental_cycles"], 1000)
        changed = self.classify(text=log(), tx=transactions().replace(" 83198 ", " 83298 ", 1))
        self.assertEqual(changed["cycle_table"][1]["incremental_cycles"], 1100)

    def test_postcheck_failure_is_fail_with_first_case_and_unchecked_rest(self):
        result = self.classify(text=log(post=1, post_pass=False, cosim="FAIL"),
                               exit_code=1, tx=transactions())
        self.assertEqual(result["verdict"], "FAIL")
        states = [case["rtl_postcheck"] for case in result["cases"]]
        self.assertEqual(states[:2], ["PASS", "FIRST_FAIL"])
        self.assertEqual(set(states[2:]), {"NOT_CHECKED"})

    def test_csim_failure_is_fail(self):
        result = self.classify(text=log(csim=3, csim_pass=False), exit_code=1)
        self.assertEqual(result["verdict"], "FAIL")
        self.assertEqual(result["cases"][3]["csim"], "FIRST_FAIL")

    def test_signal_is_signaled_even_mid_rtl(self):
        self.assertEqual(self.verdict(text=log(rtl=6, post=None), signal="SIGTERM"),
                         "SIGNALED")

    def test_missing_exit_record_is_instrument_error(self):
        self.assertEqual(self.verdict(text=log(), tx=transactions(), exit_record=False),
                         "INSTRUMENT_ERROR")

    def test_incomplete_rtl_without_signal_is_instrument_error(self):
        self.assertEqual(self.verdict(text=log(rtl=6, post=None), exit_code=1),
                         "INSTRUMENT_ERROR")
        self.assertEqual(self.verdict(text=log(rtl=6, post=0, post_pass=False, cosim=None),
                                      exit_code=1), "INSTRUMENT_ERROR")

    def test_pass_markers_with_nonzero_exit_or_short_report_are_instrument_errors(self):
        self.assertEqual(self.verdict(text=log(), exit_code=1, tx=transactions()),
                         "INSTRUMENT_ERROR")
        self.assertEqual(self.verdict(text=log(), tx=transactions(rows=12)),
                         "INSTRUMENT_ERROR")

    def test_metadata_disagreement_between_executions_is_instrument_error(self):
        bad = group(13, True).replace("dda_steps=640", "dda_steps=641", 1)
        self.assertEqual(self.verdict(text=log(post_group=bad), tx=transactions()),
                         "INSTRUMENT_ERROR")


if __name__ == "__main__":
    unittest.main()
