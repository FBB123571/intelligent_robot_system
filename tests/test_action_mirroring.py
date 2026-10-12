import sys
import unittest


@unittest.skipIf(sys.version_info[0] < 3, 'Action generation requires Python 3')
class MirrorTests(unittest.TestCase):
    def setUp(self):
        from pathlib import Path
        self.root = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(self.root / 'tools'))
        import generate_action_candidates
        self.tool = generate_action_candidates

    def test_existing_right_candidate_and_double_mirror(self):
        rows = self.tool.action_frames(self.root / 'humanoid/ActionGroup/grab_turn_left.d6a')
        result = self.tool.mirror_frames(rows)
        existing = self.tool.action_frames(self.root / 'humanoid/ActionGroup/candidates/grab_turn_right.d6a')
        self.assertEqual(result, [list(r) for r in existing])
        self.assertEqual(self.tool.mirror_frames(result), [list(r) for r in rows])
        for old, new in zip(rows, result):
            for servo in (6, 7, 8, 14, 15, 16, 17, 18):
                self.assertEqual(old[servo + 1], new[servo + 1])

    def test_roundtrip_16_and_18_channels_refuses_overwrite(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as tmp:
            for channels in (16, 18):
                rows = [(1, 1000) + (500,) * channels]
                path = Path(tmp) / ('action%d.d6a' % channels)
                self.tool.write_new(path, self.tool.mirror_frames(rows, 'body'))
                self.assertEqual(self.tool.action_frames(path), rows)
                before = path.read_bytes()
                with self.assertRaises(FileExistsError):
                    self.tool.write_new(path, rows)
                self.assertEqual(path.read_bytes(), before)

    def test_composed_turn_preserves_leg_trajectory_timing_and_grasp(self):
        grab = self.tool.action_frames(self.root / 'humanoid/ActionGroup/grab.d6a')
        for side in ('left', 'right'):
            source = self.tool.action_frames(self.root / ('humanoid/ActionGroup/reference/turn_%s.d6a' % side))
            result = self.tool.compose_carry_turn(source, grab)
            existing = self.tool.action_frames(self.root / ('humanoid/ActionGroup/candidates/grab_turn_%s_composed.d6a' % side))
            self.assertEqual(result, [list(r) for r in existing])
            self.assertEqual(result[0][1], 600)
            for old, new in zip(source, result[1:]):
                self.assertEqual(old[1], new[1])
                for servo in list(range(1, 6)) + list(range(9, 14)):
                    self.assertEqual(old[servo + 1], new[servo + 1])
            for row in result:
                for servo in (6, 7, 8, 14, 15, 16, 17, 18):
                    self.assertEqual(row[servo + 1], grab[-1][servo + 1])

    def test_eighteen_channel_gait_with_sixteen_channel_grab(self):
        source_dir = self.root / 'humanoid/ActionGroup/carry_gait_sources_20261012'
        grab = self.tool.action_frames(source_dir / 'grab.d6a')
        for name in ('go_forward', 'turn_left_fast', 'turn_right_fast'):
            source = self.tool.action_frames(source_dir / (name + '.d6a'))
            result = self.tool.compose_carry_turn(source, grab, entry_ms=0)
            self.assertEqual(len(result), len(source))
            for old, new in zip(source, result):
                self.assertEqual(len(new), 18)
                self.assertEqual(new[1], old[1])
                for servo in list(range(1, 6)) + list(range(9, 14)):
                    self.assertEqual(new[servo + 1], old[servo + 1])
                for servo in (6, 7, 8, 14, 15, 16):
                    self.assertEqual(new[servo + 1], grab[-1][servo + 1])
