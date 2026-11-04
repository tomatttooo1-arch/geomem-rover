"""The 3.8 stand-ins for `PurePath.is_relative_to` and `str.removeprefix`.

Four tools carry a private copy of one of these two helpers, because the standard-library
methods only exist from Python 3.9 and `/usr/bin/python3` is 3.8. The copies are deliberate
duplication — the repo keeps small private helpers per module rather than a shared utility —
so the risk is not that one of them is wrong today but that one of them drifts. This file is
cross-cutting for exactly that reason: one case matrix, applied to every copy.

Three of the copies guard a path-escape check (`tools/fetch_replay_data.py` over tar members,
`tools/rtabmap_prepare_tum.py` and `tools/rtabmap_reload_coordinate_prepare.py` over dataset
inputs), so the negative cases here are the ones that matter: an escaping path must be
rejected. A helper that simply returned `True` would pass every positive case and is caught
only by those.

On 3.9+ each case is additionally checked against the real standard-library method, which
makes the expected column below an oracle rather than an assertion of our own belief.

`rtabmap_reason_audit` and `rtabmap_reload_coordinate_prepare` already have test files that
drive their call sites. The other two tools have none, so `CallSiteEscapeGuardTests` at the
bottom drives *their* escape checks for real rather than leaving them pinned by a source
grep: a helper tested only in isolation says nothing about whether the call site still calls
it.
"""
import io
import json
import pathlib
from pathlib import Path
import shutil
import tarfile
import tempfile
import unittest
from unittest import mock

import tools.fetch_replay_data
import tools.rtabmap_prepare_tum
from tools.fetch_replay_data import path_within as fetch_path_within
from tools.rtabmap_prepare_tum import path_within as tum_path_within
from tools.rtabmap_reload_coordinate_prepare import path_within as reload_path_within
from tools.rtabmap_reason_audit import remove_prefix

PATH_COPIES = (('tools/fetch_replay_data.py', fetch_path_within),
               ('tools/rtabmap_prepare_tum.py', tum_path_within),
               ('tools/rtabmap_reload_coordinate_prepare.py', reload_path_within))

# (path, root, expected).  `is_relative_to` is purely lexical: it never touches the
# filesystem, never resolves symlinks and never collapses '..'.
PATH_CASES = (
    # --- inside: must be accepted ---
    ('/data/run/rgb/1.png', '/data/run', True),
    ('/data/run', '/data/run', True),              # a root is relative to itself
    ('/data/run/./rgb/1.png', '/data/run', True),  # '.' components are dropped by PurePath
    ('/data/run/rgb', '/data/', True),             # trailing slash on the root
    ('rel/a/b', 'rel/a', True),                    # both relative
    ('/a/b/c/d/e', '/a', True),
    ('/data/run/rgb//1.png', '/data/run', True),   # repeated separators collapse
    # --- outside: MUST be rejected (these are the negative controls) ---
    ('/data/run-2/rgb/1.png', '/data/run', False),  # sibling sharing a string prefix
    ('/data/runner', '/data/run', False),           # string-prefix but not a path prefix
    ('/data/ru', '/data/run', False),               # the root is not a string prefix either
    ('/data', '/data/run', False),                  # a parent is not inside its child
    ('/', '/data', False),
    ('/etc/passwd', '/data/run', False),
    ('../outside', 'rel/a', False),
    ('rel/b', 'rel/a', False),
    # --- absolute/relative mix: False, never an exception ---
    ('rel/a/b', '/data/run', False),
    ('/data/run/rgb', 'data/run', False),
)

# '..' is NOT collapsed, so these escape the root yet are lexically inside it. The
# standard-library method answers True as well; it is `resolve()` at the call site, not
# the containment test, that defeats the escape. Pinned so that a "helpful" helper that
# started collapsing '..' — and so diverged from the method it stands in for — is caught.
DOT_DOT_CASES = (
    ('/data/run/../../etc/passwd', '/data/run', '/etc/passwd'),
    ('/data/run/..', '/data/run', '/data'),
)

# (text, prefix, expected)
PREFIX_CASES = (
    ('received_info', 'received_', 'info'),
    ('received_odom_info', 'received_', 'odom_info'),
    ('received_', 'received_', ''),            # prefix is the whole string
    ('acquisition', 'received_', 'acquisition'),  # no match: unchanged
    ('', 'received_', ''),
    ('Received_info', 'received_', 'Received_info'),  # case-sensitive
    ('rreceived_info', 'received_', 'rreceived_info'),  # match must be anchored
    ('received_info', '', 'received_info'),    # empty prefix always matches, strips nothing
    ('', '', ''),
    ('info_received_', 'received_', 'info_received_'),  # suffix is not a prefix
)


class PathWithinTests(unittest.TestCase):
    def test_every_copy_answers_the_case_matrix(self):
        for name, helper in PATH_COPIES:
            for path, root, expected in PATH_CASES:
                with self.subTest(copy=name, path=path, root=root):
                    self.assertIs(helper(Path(path), Path(root)), expected)

    def test_escaping_paths_are_rejected_by_every_copy(self):
        """Negative control on its own: a helper stuck at True fails only here."""
        escaping = [(p, r) for p, r, expected in PATH_CASES if not expected]
        self.assertGreaterEqual(len(escaping), 9)
        for name, helper in PATH_COPIES:
            for path, root in escaping:
                with self.subTest(copy=name, path=path, root=root):
                    self.assertIs(helper(Path(path), Path(root)), False)

    def test_dot_dot_is_not_collapsed_and_resolve_is_what_defeats_the_escape(self):
        """The helper is lexical; the call sites resolve() first, and that is the guard."""
        for path, root, resolved in DOT_DOT_CASES:
            for name, helper in PATH_COPIES:
                with self.subTest(copy=name, path=path):
                    # lexically inside, exactly like the standard-library method
                    self.assertIs(helper(Path(path), Path(root)), True)
                    # ... and rejected once the caller resolves, which is what it does
                    self.assertEqual(str(Path(path).resolve()), resolved)
                    self.assertIs(helper(Path(path).resolve(), Path(root).resolve()), False)

    def test_the_copies_agree_with_each_other(self):
        for path, root, _ in PATH_CASES:
            verdicts = {name: helper(Path(path), Path(root)) for name, helper in PATH_COPIES}
            self.assertEqual(len(set(verdicts.values())), 1, verdicts)

    @unittest.skipUnless(hasattr(pathlib.PurePath, 'is_relative_to'),
                         'standard-library oracle needs Python 3.9+')
    def test_every_copy_matches_the_standard_library_method(self):
        for path, root, expected in PATH_CASES:
            oracle = Path(path).is_relative_to(Path(root))
            self.assertIs(oracle, expected, 'case matrix disagrees with the stdlib: %r' % (path,))
            for name, helper in PATH_COPIES:
                with self.subTest(copy=name, path=path, root=root):
                    self.assertIs(helper(Path(path), Path(root)), oracle)

    def test_a_relative_absolute_mix_answers_false_instead_of_raising(self):
        for name, helper in PATH_COPIES:
            with self.subTest(copy=name):
                self.assertIs(helper(Path('rel/a'), Path('/abs')), False)
                self.assertIs(helper(Path('/abs'), Path('rel/a')), False)

    def test_the_helper_does_not_touch_the_filesystem(self):
        """Purely lexical: the verdict must not depend on anything existing."""
        for name, helper in PATH_COPIES:
            with self.subTest(copy=name):
                self.assertIs(helper(Path('/nonexistent/x/y'), Path('/nonexistent/x')), True)
                self.assertIs(helper(Path('/nonexistent/x'), Path('/nonexistent/y')), False)


class RemovePrefixTests(unittest.TestCase):
    def test_the_case_matrix(self):
        for text, prefix, expected in PREFIX_CASES:
            with self.subTest(text=text, prefix=prefix):
                self.assertEqual(remove_prefix(text, prefix), expected)

    def test_a_prefix_that_does_not_match_leaves_the_string_alone(self):
        """Negative control: a helper that blindly slices fails only here."""
        unmatched = [(t, p, e) for t, p, e in PREFIX_CASES if not t.startswith(p) or not p]
        self.assertGreater(len(unmatched), 4)
        for text, prefix, expected in unmatched:
            with self.subTest(text=text, prefix=prefix):
                self.assertEqual(remove_prefix(text, prefix), text)
                self.assertEqual(expected, text)

    @unittest.skipUnless(hasattr(str, 'removeprefix'),
                         'standard-library oracle needs Python 3.9+')
    def test_it_matches_the_standard_library_method(self):
        for text, prefix, expected in PREFIX_CASES:
            oracle = text.removeprefix(prefix)
            self.assertEqual(oracle, expected, 'case matrix disagrees with the stdlib')
            with self.subTest(text=text, prefix=prefix):
                self.assertEqual(remove_prefix(text, prefix), oracle)


class InterpreterFloorTests(unittest.TestCase):
    """The point of the helpers: the modules that own them must import on 3.8."""

    def test_the_helper_owners_do_not_use_the_3_9_only_methods(self):
        import tools.fetch_replay_data
        import tools.rtabmap_prepare_tum
        import tools.rtabmap_reason_audit
        import tools.rtabmap_reload_coordinate_prepare
        for module in (tools.fetch_replay_data, tools.rtabmap_prepare_tum,
                       tools.rtabmap_reason_audit, tools.rtabmap_reload_coordinate_prepare):
            source = Path(module.__file__).read_text()
            body = '\n'.join(line for line in source.splitlines()
                             if '``' not in line)  # docstrings name the methods on purpose
            with self.subTest(module=module.__name__):
                self.assertNotIn('.is_relative_to(', body)
                self.assertNotIn('.removeprefix(', body)


class CallSiteEscapeGuardTests(unittest.TestCase):
    """The two tools with no test file of their own: drive the guard, not just the helper.

    Both guards reject before anything is written, so the negative case is cheap and needs
    no network and no dataset.
    """

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, str(self.tmp), True)

    # ---- tools/rtabmap_prepare_tum.py ----
    def tum_source(self, rgb_name):
        source = self.tmp / 'source'
        (source / 'rgb').mkdir(parents=True)
        (source / 'depth').mkdir(parents=True)
        (source / 'rgb' / 'a.png').write_bytes(b'rgb')
        (source / 'depth' / 'a.png').write_bytes(b'depth')
        (self.tmp / 'outside.png').write_bytes(b'outside')
        (source / 'rgb.txt').write_text('1.000 %s\n' % rgb_name)
        (source / 'depth.txt').write_text('1.001 depth/a.png\n')
        return source

    def run_tum(self, rgb_name, output):
        argv = ['rtabmap_prepare_tum.py', str(self.tum_source(rgb_name)), str(output)]
        with mock.patch('sys.argv', argv), mock.patch('sys.stdout', io.StringIO()):
            tools.rtabmap_prepare_tum.main()

    def test_prepare_tum_rejects_an_rgb_row_that_escapes_the_dataset(self):
        """Negative control, executed: the '..' name must not be copied out of the source."""
        output = self.tmp / 'out-escape'
        with self.assertRaises(ValueError) as caught:
            self.run_tum('../outside.png', output)
        self.assertEqual(str(caught.exception), 'input path escapes dataset')
        self.assertFalse((output / 'rgb_sync' / '1.000.png').exists())

    def test_prepare_tum_accepts_a_row_inside_the_dataset(self):
        """Positive control: the same path, one directory over, goes through."""
        output = self.tmp / 'out-ok'
        self.run_tum('rgb/a.png', output)
        self.assertEqual((output / 'rgb_sync' / '1.000.png').read_bytes(), b'rgb')
        self.assertEqual(json.loads((output / 'INPUT.json').read_text())['rgb_count'], 1)

    # ---- tools/fetch_replay_data.py ----
    def archive_with(self, member_name):
        archive = self.tmp / ('%s.tgz' % abs(hash(member_name)))
        payload = self.tmp / 'payload'
        payload.write_bytes(b'payload')
        with tarfile.open(archive, 'w:gz') as tar:
            tar.add(str(payload), arcname=member_name)
        return archive

    def run_fetch(self, member_name, output):
        argv = ['fetch_replay_data.py', '--out', str(output),
                '--archive', str(self.archive_with(member_name))]
        with mock.patch('sys.argv', argv), mock.patch('sys.stdout', io.StringIO()):
            tools.fetch_replay_data.main()

    def test_fetch_replay_data_rejects_a_tar_member_that_escapes_the_output(self):
        """Negative control, executed: a '..' member must stop the extraction."""
        output = self.tmp / 'fetch-escape'
        with self.assertRaises(ValueError) as caught:
            self.run_fetch('../escaped.txt', output)
        self.assertEqual(str(caught.exception),
                         'archive contains non-regular or escaping member')
        self.assertFalse((self.tmp / 'escaped.txt').exists())
        self.assertFalse((output / 'SOURCE.json').exists())

    @unittest.skipUnless(hasattr(tarfile, 'data_filter'),
                         "extractall(filter=...) needs the tarfile extraction filters "
                         "(3.12, backported to 3.8.17); the 3.8.10 reference interpreter "
                         "raises TypeError here -- see the branch report, not fixed")
    def test_fetch_replay_data_accepts_a_member_inside_the_output(self):
        """Positive control: the same archive without the '..' extracts."""
        output = self.tmp / 'fetch-ok'
        self.run_fetch('desk/inside.txt', output)
        self.assertEqual((output / 'desk' / 'inside.txt').read_bytes(), b'payload')
        self.assertIn('source_url', json.loads((output / 'SOURCE.json').read_text()))


if __name__ == '__main__':
    unittest.main()
