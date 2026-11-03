"""User-facing recording states must distinguish preparation and final saving."""
from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rk'))
from video_recording import recording_presentation


class RecordingPresentationTests(unittest.TestCase):
    def test_normal_states_have_actionable_text_buttons_and_styles(self):
        cases=[
            ({'state':'idle'},'未录像｜按9开始录像','开始录像（9）','idle',False),
            ({'state':'starting'},'正在准备录像，尚未开始','准备中（9可取消）','preparing',False),
            ({'state':'recording','active':True,'duration_s':7.9},
             '● 正在录像  00:07｜按9停止并保存','停止并保存（9）','recording',False),
            ({'state':'stopping'},'正在保存录像，请勿拔U盘','保存中…','saving',True),
            ({'state':'idle','saved':True},'录像已停止，保存成功｜按9录制新视频',
             '重新录像（9）','saved',False),
            ({'state':'error','error':'U盘写入失败'},'录像失败：U盘写入失败',
             '重试录像（9）','error',False),
        ]
        for status,text,button,style,busy in cases:
            with self.subTest(state=status['state'],saved=status.get('saved',False)):
                view=recording_presentation(status)
                self.assertEqual(view['text'],text)
                self.assertEqual(view['button'],button)
                self.assertEqual(view['style'],style)
                self.assertIs(view['busy'],busy)

    def test_pending_and_error_states_take_priority_over_stale_active_and_saved_flags(self):
        for state,style in [('starting','preparing'),('stopping','saving'),('error','error')]:
            with self.subTest(state=state):
                view=recording_presentation(dict(state=state,active=True,saved=True,
                    duration_s=42,error='写入失败' if state=='error' else ''))
                self.assertEqual(view['style'],style)
                self.assertNotIn('● 正在录像',view['text'])
                self.assertNotIn('保存成功',view['text'])

    def test_reported_error_overrides_other_states_and_a_previously_saved_file(self):
        for state in ('idle','starting','recording','stopping'):
            with self.subTest(state=state):
                view=recording_presentation(dict(state=state,active=True,saved=True,error='U盘已移除'))
                self.assertEqual(view['style'],'error')
                self.assertTrue(view['text'].startswith('录像失败：'))
                self.assertIn('U盘已移除',view['text'])
                self.assertFalse(view['busy'])

    def test_elapsed_time_uses_submitted_video_duration_and_expands_at_one_hour(self):
        for seconds,clock in [(0,'00:00'),(.99,'00:00'),(59.99,'00:59'),
                              (60,'01:00'),(3599.9,'59:59'),(3600,'01:00:00'),
                              (3661.9,'01:01:01')]:
            with self.subTest(seconds=seconds):
                view=recording_presentation(dict(state='recording',active=True,duration_s=seconds))
                self.assertEqual(view['text'],f'● 正在录像  {clock}｜按9停止并保存')

    def test_a_recording_file_path_alone_does_not_claim_saved_success(self):
        status=dict(state='idle',active=False,path='/usb/unfinished.avi',frames=50,saved=False)
        self.assertEqual(recording_presentation(status)['style'],'idle')
        self.assertNotIn('保存成功',recording_presentation(status)['text'])


if __name__=='__main__':unittest.main()
