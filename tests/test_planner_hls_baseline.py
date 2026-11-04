"""Regression for actual hls_report.sh nested JSON schema and fail-closed gate."""
import unittest
from tools.planner_model.hls.baseline import check_synth_pragmas


def clean_report():
    return dict(kind='synth',upstream=dict(
        synthesisPragmaReportContent='== Pragma Report\n* Valid Pragma Syntax\n| dataflow |',
        synthesisLogContent='AGENT_STEP_RESULT: OK\n'))


class PragmaGateTest(unittest.TestCase):
    def test_accepts_real_nested_schema(self):
        check_synth_pragmas(clean_report())

    def test_rejects_warning_inside_upstream(self):
        report=clean_report()
        report['upstream']['synthesisLogContent']+='WARNING: [HLS 207-5555] unexpected pragma argument\n'
        with self.assertRaises(RuntimeError):check_synth_pragmas(report)

    def test_rejects_ignored_section_inside_upstream(self):
        report=clean_report()
        report['upstream']['synthesisPragmaReportContent']+='\n* Ignored Pragmas\n'
        with self.assertRaises(RuntimeError):check_synth_pragmas(report)

    def test_rejects_wrong_flat_schema(self):
        with self.assertRaises(RuntimeError):check_synth_pragmas(clean_report()['upstream'])

    def test_rejects_missing_log(self):
        report=clean_report();del report['upstream']['synthesisLogContent']
        with self.assertRaises(RuntimeError):check_synth_pragmas(report)

    def test_rejects_empty_or_wrong_type(self):
        for value in ['',None,[]]:
            report=clean_report();report['upstream']['synthesisPragmaReportContent']=value
            with self.subTest(value=value),self.assertRaises(RuntimeError):check_synth_pragmas(report)
