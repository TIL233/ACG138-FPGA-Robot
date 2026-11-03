"""Exercise the actual HDMI update and map pixels without a camera or desktop."""
import importlib.util
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

from robot_rebuild.tests import test_hdmi_lifecycle as lifecycle


class LidarMapViewTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('lidar_map_view'),
                             'The HDMI radar map rasterizer is missing')
        import lidar_map_view
        self.map = lidar_map_view

    @staticmethod
    def obstacle(pixel):
        red, green, blue = pixel
        return red > 220 and 75 < green < 200 and blue < 110

    def test_four_clockwise_sector_centres_and_metric_rings(self):
        ranges = [600] * 36  # An unmeasured bin must ignore even plausible range bytes.
        for i, distance in ((0, 1000), (9, 2000), (18, 4000), (27, 8000)):
            ranges[i] = distance
        mask = sum(1 << i for i in (0, 9, 18, 27))
        rgb = self.map.render_map(mask, ranges)
        width = self.map.MAP_SIZE
        def pixel(x, y):
            address = 3 * (y * width + x)
            return rgb[address:address+3]
        # 672 px square, center (336,336), radius 300; +5° = just right of up.
        for x, y in ((343, 261), (485, 349), (316, 560), (37, 310)):
            self.assertTrue(self.obstacle(pixel(x, y)), (x, y, pixel(x, y)))
        self.assertFalse(self.obstacle(pixel(371, 307)))
        self.assertNotEqual(pixel(468, 270), pixel(342, 276))  # Unknown vs measured sector.

    def test_map_requires_click_and_stays_navigable_on_timeout_and_disconnect(self):
        harness = lifecycle.HdmiTests(); harness.setUp(); self.addCleanup(harness.doCleanups)
        viewer = harness.viewer()
        viewer.link.nav_telemetry = None
        class Pages:
            current = 'camera'
            def set_visible_child_name(self, name): self.current = name
            def get_visible_child_name(self): return self.current
        class Button:
            visible = False
            def set_visible(self, visible): self.visible = visible
        viewer.view_stack = Pages()
        viewer.lidar_map_button = Button()
        viewer.lidar_map_canvas = mock.Mock()
        viewer.lidar_map_caption = mock.Mock()
        viewer.lidar_map_key = None
        viewer.lidar_map_pixbuf = None
        viewer.link.lidar_map_telemetry = None
        harness.module.GLib.Bytes.new.side_effect = lambda rgb: rgb
        harness.module.GdkPixbuf.Pixbuf.new_from_bytes.side_effect = lambda rgb, *_: rgb
        viewer.update()
        self.assertFalse(viewer.lidar_map_button.visible)
        self.assertIsNone(viewer.lidar_map_pixbuf)

        viewer.link.lidar_map_telemetry = SimpleNamespace(known_mask=1, ranges_mm=(1000,)*36)
        viewer.link.lidar_map_live.return_value = True
        viewer.update()
        self.assertEqual(viewer.view_stack.current, 'camera')
        self.assertTrue(viewer.lidar_map_button.visible)
        self.assertIsNone(viewer.lidar_map_pixbuf)
        viewer.show_lidar_map()
        self.assertEqual(viewer.view_stack.current, 'map')
        self.assertEqual(viewer.lidar_map_pixbuf, self.map.render_map(1, (1000,)*36))
        self.assertIn('灰', viewer.lidar_map_caption.set_text.call_args.args[0])
        viewer.link.lidar_map_live.return_value = False
        viewer.update()
        self.assertEqual(viewer.lidar_map_pixbuf, self.map.render_map(0, ()))
        self.assertIn('等待', viewer.lidar_map_caption.set_text.call_args.args[0])
        viewer.link.lidar_map_telemetry = None
        viewer.update()
        self.assertEqual(viewer.view_stack.current, 'map')
        self.assertFalse(viewer.lidar_map_button.visible)
        viewer.show_camera_page()
        self.assertEqual(viewer.view_stack.current, 'camera')
        viewer.link.send.assert_not_called()

    def test_page_centres_same_scale_in_wide_and_tall_windows_and_labels_all_rings(self):
        harness = lifecycle.HdmiTests(); harness.setUp(); self.addCleanup(harness.doCleanups)
        viewer = harness.viewer()
        viewer.lidar_map_pixbuf = self.map.render_map(0, ())
        canvas = mock.Mock()
        layout = mock.Mock(); layout.get_pixel_size.return_value = (30, 18)
        canvas.create_pango_layout.return_value = layout
        for width, height, origin, side in ((1920, 994, (491, 28), 938), (600, 900, (40, 190), 520)):
            with self.subTest(width=width, height=height):
                canvas.get_allocated_width.return_value = width
                canvas.get_allocated_height.return_value = height
                canvas.create_pango_layout.reset_mock()
                ctx = mock.Mock()
                with mock.patch.dict(sys.modules, {'gi.repository': SimpleNamespace(PangoCairo=mock.Mock())}):
                    viewer.draw_lidar_map(canvas, ctx)
                self.assertEqual(ctx.translate.call_args.args, origin)
                self.assertEqual(ctx.scale.call_args.args, (side/self.map.MAP_SIZE,)*2)
                labels = {call.args[0] for call in canvas.create_pango_layout.call_args_list}
                self.assertEqual(labels, {'前 · 车头', '后', '左', '右', '1 m', '2 m', '4 m', '8 m'})


if __name__ == '__main__':
    unittest.main()
