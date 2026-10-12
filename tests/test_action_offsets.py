import sys
import unittest
import tempfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from generate_action_offsets import offset_frames,generate
from generate_action_candidates import action_frames


class OffsetTests(unittest.TestCase):
    def test_mirrored_hip_delta_preserves_timing_arms_and_other_channels(self):
        for count in (16,18):
            source=[(7,300)+(500,)*count,(9,50)+(400,)*count]
            output=offset_frames(source,{4:-20,12:20})
            for before,after in zip(source,output):
                self.assertEqual(after[5],before[5]-20)
                self.assertEqual(after[13],before[13]+20)
                for i in set(range(count+2))-{5,13}: self.assertEqual(before[i],after[i])

    def test_overflow_rejected_before_any_output_is_created(self):
        source=ROOT/'humanoid/ActionGroup/carry_generated/grab_go_forward.d6a'
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'offset'
            with self.assertRaises(ValueError): generate([source],out,{4:1000})
            self.assertFalse(out.exists())

    def test_source_untouched_suffix_and_existing_copy_not_overwritten(self):
        source=ROOT/'humanoid/ActionGroup/carry_generated/grab_go_forward.d6a'
        original=source.read_bytes()
        with tempfile.TemporaryDirectory() as tmp:
            report=generate([source],tmp,{4:20,12:-20})
            dest=Path(tmp)/'grab_go_forward_offset.d6a'
            self.assertEqual(len(action_frames(dest)),len(action_frames(source)))
            before=dest.read_bytes()
            with self.assertRaises(FileExistsError): generate([source],tmp,{4:-20,12:20})
            self.assertEqual(dest.read_bytes(),before)
            self.assertFalse(report['validated_on_hardware'])
        self.assertEqual(source.read_bytes(),original)
