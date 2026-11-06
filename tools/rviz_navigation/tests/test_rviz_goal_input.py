import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch


MODULE_PATH = Path(__file__).parents[1] / "rviz_goal_input.py"
SPEC = importlib.util.spec_from_file_location("rviz_goal_input", MODULE_PATH)
gui = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = gui
SPEC.loader.exec_module(gui)


class RvizGoalInputTests(unittest.TestCase):
    def test_discovers_rviz_window_by_current_tree_title(self):
        result = subprocess.CompletedProcess([], 0,
            'xwininfo: Window id: 0x1 (root)\n  0xa00014 "RViz"\n  0x10 "terminal"\n', '')
        with patch.object(gui, "_run_xwininfo", return_value=result) as run:
            with patch.object(gui, "_run_xprop", return_value=subprocess.CompletedProcess([], 0,
                    'WM_STATE(WM_STATE):\n\twindow state: Normal\n', '')):
                with patch.object(gui, "inspect_rviz_window", return_value=gui.Window(0xa00014, "RViz", True)):
                    self.assertEqual(gui.discover_rviz_window(), gui.Window(0xa00014, "RViz", True))
        run.assert_called_once_with("-root", "-tree")

    def test_discovery_ignores_unmanaged_rviz_child_and_qt_windows(self):
        tree = subprocess.CompletedProcess([], 0,
            '0x600014 "/path/navigation.rviz - RViz"\n'
            '0x600011 "rviz2"\n'
            '0x600004 "Qt Selection Owner for rviz2"\n', '')
        properties = {
            0x600014: 'WM_STATE(WM_STATE):\n\twindow state: Normal\n',
            0x600011: 'WM_STATE: not found.\n',
            0x600004: 'WM_STATE: not found.\n',
        }

        def xprop(*args):
            window_id = int(args[1], 16)
            return subprocess.CompletedProcess([], 0, properties[window_id], '')

        expected = gui.Window(0x600014, "/path/navigation.rviz - RViz", True)
        with patch.object(gui, "_run_xwininfo", return_value=tree):
            with patch.object(gui, "_run_xprop", side_effect=xprop):
                with patch.object(gui, "inspect_rviz_window", return_value=expected) as inspect:
                    self.assertEqual(gui.discover_rviz_window(), expected)
        inspect.assert_called_once_with(0x600014)

    def test_rejects_missing_or_ambiguous_rviz_window(self):
        self.assertEqual(gui.rviz_windows_from_tree('0x10 "terminal"'), [])
        candidates = gui.rviz_windows_from_tree('0x10 "RViz A"\n0x20 "rviz2"')
        self.assertEqual(candidates, [(0x10, "RViz A"), (0x20, "rviz2")])

    def test_discovery_fails_fast_if_no_rviz_window_exists(self):
        result = subprocess.CompletedProcess([], 0, '0x10 "terminal"\n', '')
        with patch.object(gui, "_run_xwininfo", return_value=result):
            with self.assertRaisesRegex(gui.InputError, "no RViz-titled"):
                gui.discover_rviz_window()

    def test_discovery_rejects_ambiguous_rviz_windows(self):
        result = subprocess.CompletedProcess([], 0, '0x10 "RViz A"\n0x20 "rviz2"\n', '')
        with patch.object(gui, "_run_xwininfo", return_value=result):
            with patch.object(gui, "_run_xprop", return_value=subprocess.CompletedProcess([], 0,
                    'WM_STATE(WM_STATE):\n\twindow state: Normal\n', '')):
                def viewable(window_id):
                    title = "RViz A" if window_id == 0x10 else "rviz2"
                    return gui.Window(window_id, title, True)
                with patch.object(gui, "inspect_rviz_window", side_effect=viewable):
                    with self.assertRaisesRegex(gui.InputError, "multiple RViz windows"):
                        gui.discover_rviz_window()

    def test_discovery_ignores_unmapped_rviz_popup_but_keeps_visible_window(self):
        tree = subprocess.CompletedProcess([], 0,
            '0x10 "/path/navigation.rviz - RViz"\n'
            '0x19 "rviz2"\n', '')
        properties = {
            0x10: 'WM_STATE(WM_STATE):\n\twindow state: Normal\n',
            0x19: 'WM_STATE(WM_STATE):\n\twindow state: Withdrawn\n',
        }

        def xprop(*args):
            window_id = int(args[1], 16)
            return subprocess.CompletedProcess([], 0, properties[window_id], '')

        def inspect(window_id):
            if window_id == 0x19:
                raise gui.InputError('RViz window 0x19 \'rviz2\' is not viewable')
            return gui.Window(0x10, "/path/navigation.rviz - RViz", True)

        expected = gui.Window(0x10, "/path/navigation.rviz - RViz", True)
        with patch.object(gui, "_run_xwininfo", return_value=tree):
            with patch.object(gui, "_run_xprop", side_effect=xprop):
                with patch.object(gui, "inspect_rviz_window", side_effect=inspect):
                    self.assertEqual(gui.discover_rviz_window(), expected)

    def test_discovery_still_rejects_multiple_viewable_windows_with_popup(self):
        tree = subprocess.CompletedProcess([], 0,
            '0x10 "RViz A"\n0x19 "rviz2"\n0x20 "RViz B"\n', '')
        with patch.object(gui, "_run_xwininfo", return_value=tree):
            with patch.object(gui, "_run_xprop", return_value=subprocess.CompletedProcess([], 0,
                    'WM_STATE(WM_STATE):\n\twindow state: Normal\n', '')):
                def inspect(window_id):
                    if window_id == 0x19:
                        raise gui.InputError('RViz window 0x19 \'rviz2\' is not viewable')
                    return gui.Window(window_id, f"RViz {window_id}", True)
                with patch.object(gui, "inspect_rviz_window", side_effect=inspect):
                    with self.assertRaisesRegex(gui.InputError, "multiple RViz windows"):
                        gui.discover_rviz_window()

    def test_discovery_rejects_unmanaged_rviz_candidates(self):
        result = subprocess.CompletedProcess([], 0, '0x10 "rviz2"\n', '')
        with patch.object(gui, "_run_xwininfo", return_value=result):
            with patch.object(gui, "_run_xprop", return_value=subprocess.CompletedProcess([], 0,
                    'WM_STATE: not found.\n', '')):
                with self.assertRaisesRegex(gui.InputError, "no managed RViz top-level"):
                    gui.discover_rviz_window()

    def test_rejects_stale_window_id_before_x_input(self):
        failed = subprocess.CompletedProcess([], 1, "", "xwininfo: error: Bad Window")
        with patch.object(gui, "_run_xwininfo", return_value=failed):
            with self.assertRaisesRegex(gui.InputError, "no longer valid"):
                gui.inspect_rviz_window(0xa00014)

    def test_rejects_non_rviz_or_non_viewable_window(self):
        wrong = subprocess.CompletedProcess([], 0, 'Window id: 0x12 "terminal"\nMap State: IsViewable', '')
        with patch.object(gui, "_run_xwininfo", return_value=wrong):
            with self.assertRaisesRegex(gui.InputError, "identity mismatch"):
                gui.inspect_rviz_window(0x12)
        unmapped = subprocess.CompletedProcess([], 0, 'Window id: 0x12 "RViz"\nMap State: IsUnMapped', '')
        with patch.object(gui, "_run_xwininfo", return_value=unmapped):
            with self.assertRaisesRegex(gui.InputError, "not viewable"):
                gui.inspect_rviz_window(0x12)

    def test_no_focus_is_a_fail_fast_error(self):
        with patch.object(gui, "discover_rviz_window", return_value=gui.Window(0xa00014, "RViz", True)):
            with patch.object(gui, "input_focus", return_value=0):
                with self.assertRaisesRegex(gui.InputError, "not active"):
                    gui.checked_live_focus()

    def test_focus_on_live_rviz_child_is_accepted(self):
        expected = gui.Window(0xa00014, "RViz", True)
        child = subprocess.CompletedProcess([], 0,
            'Window id: 0xa00004\nParent window id: 0xa00014 "RViz"\n', '')
        with patch.object(gui, "discover_rviz_window", return_value=expected):
            with patch.object(gui, "input_focus", return_value=0xa00004):
                with patch.object(gui, "_run_xwininfo", return_value=child) as inspect:
                    self.assertEqual(gui.checked_live_focus(), (expected, 0xa00004))
        inspect.assert_called_once_with("-all", "-id", "0xa00004")

    def test_focus_outside_rviz_window_tree_is_rejected(self):
        expected = gui.Window(0xa00014, "RViz", True)
        outside = subprocess.CompletedProcess([], 0,
            'Window id: 0x20\nParent window id: 0x1 "root"\n', '')
        with patch.object(gui, "discover_rviz_window", return_value=expected):
            with patch.object(gui, "input_focus", return_value=0x20):
                with patch.object(gui, "_run_xwininfo", return_value=outside):
                    with self.assertRaisesRegex(gui.InputError, "not active"):
                        gui.checked_live_focus()

    def test_matching_live_window_and_focus_are_accepted(self):
        expected = gui.Window(0xa00014, "RViz", True)
        with patch.object(gui, "discover_rviz_window", return_value=expected):
            with patch.object(gui, "input_focus", return_value=0xa00014):
                self.assertEqual(gui.checked_live_focus(), (expected, 0xa00014))


if __name__ == "__main__":
    unittest.main()
