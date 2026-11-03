"""Recording RGB/alpha behavior without a GTK installation or a display."""
from dataclasses import FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rk'))
import recording_overlay as overlay


class CompositeTests(unittest.TestCase):
    def test_alpha_extremes_keep_transparent_pixels_and_preserve_rgb_order(self):
        frame = bytes((10, 20, 30, 40, 50, 60))
        patch = overlay.OverlayPatch(0, 0, 2, 1,
                                     bytes((200, 150, 100, 250, 30, 1)), bytes((0, 255)))
        result = overlay.composite_rgb(frame, 2, 1, patch)
        self.assertEqual(result, bytes((10, 20, 30, 250, 30, 1)))
        self.assertEqual(frame, bytes((10, 20, 30, 40, 50, 60)))

    def test_fractional_alpha_rounds_channels_and_only_changes_patch_rectangle(self):
        frame = bytes((0, 10, 255, 20, 30, 40, 50, 60, 70,
                       80, 90, 100, 110, 120, 130, 140, 150, 160))
        patch = overlay.OverlayPatch(1, 1, 2, 1,
                                     bytes((255, 0, 80, 0, 255, 10)), bytes((128, 220)))
        result = overlay.composite_rgb(frame, 3, 2, patch)
        expected = bytearray(frame)
        expected[12:15] = bytes((183, 60, 105))
        expected[15:18] = bytes((19, 241, 31))
        self.assertEqual(result, bytes(expected))
        self.assertEqual(patch.rgb, bytes((255, 0, 80, 0, 255, 10)))
        self.assertEqual(patch.alpha, bytes((128, 220)))

    def test_frame_buffer_is_not_mutated_and_result_is_independent(self):
        frame = bytearray((5, 10, 15, 20, 25, 30))
        original = bytes(frame)
        patch = overlay.OverlayPatch(1, 0, 1, 1, b'\xff\xff\xff', b'\x80')
        result = overlay.composite_rgb(frame, 2, 1, patch)
        self.assertIsInstance(result, bytes)
        self.assertEqual(bytes(frame), original)
        frame[:] = b'\x00' * len(frame)
        self.assertEqual(result, bytes((5, 10, 15, 138, 140, 143)))

    def test_none_and_empty_patch_return_new_equal_bytes_without_numpy(self):
        frame = bytes(range(12))
        for patch in (None, overlay.OverlayPatch(2, 2, 0, 0, b'', b'')):
            with self.subTest(patch=patch), mock.patch('builtins.__import__', side_effect=AssertionError):
                result = overlay.composite_rgb(frame, 2, 2, patch)
            self.assertEqual(result, frame)
            self.assertIsNot(result, frame)

    def test_invalid_frame_geometry_and_lengths_are_rejected(self):
        for width, height, data in ((0, 1, b''), (1, -1, b''), (True, 1, b'abc'),
                                    (1.0, 1, b'abc'), (1, 1, b'ab'), (1, 1, b'abcd')):
            with self.subTest(width=width, height=height, data=data), self.assertRaises(ValueError):
                overlay.composite_rgb(data, width, height, None)

    def test_out_of_bounds_and_invalid_patch_geometry_are_rejected(self):
        for x, y, width, height in ((-1, 0, 1, 1), (0, -1, 1, 1), (2, 0, 1, 1),
                                     (0, 2, 1, 1), (1, 0, 2, 1), (0, 1, 1, 2),
                                     (0, 0, -1, 1), (0, 0, 1, -1), (True, 0, 1, 1),
                                     (0, 0, 1.0, 1)):
            patch = overlay.OverlayPatch(x, y, width, height, b'abc', b'\x00')
            with self.subTest(geometry=(x, y, width, height)), self.assertRaises(ValueError):
                overlay.composite_rgb(bytes(12), 2, 2, patch)

    def test_patch_length_is_checked_even_when_alpha_is_transparent(self):
        for rgb, alpha in ((b'ab', b'\x00'), (b'abcd', b'\x00'),
                           (b'abc', b''), (b'abc', b'\x00\x00')):
            patch = overlay.OverlayPatch(0, 0, 1, 1, rgb, alpha)
            with self.subTest(rgb=rgb, alpha=alpha), self.assertRaises(ValueError):
                overlay.composite_rgb(b'abc', 1, 1, patch)

    def test_patch_freezes_its_fields_and_copies_mutable_buffer_inputs(self):
        rgb, alpha = bytearray(b'abc'), bytearray(b'\x80')
        patch = overlay.OverlayPatch(0, 0, 1, 1, rgb, alpha)
        rgb[0], alpha[0] = 0, 0
        self.assertEqual((patch.rgb, patch.alpha), (b'abc', b'\x80'))
        with self.assertRaises(FrozenInstanceError):
            patch.x = 1

    def test_native_cairo_colors_are_unpremultiplied_and_row_padding_is_ignored(self):
        # 50% red, opaque blue, and a padding word that must not become a pixel.
        data = struct.pack('=III', 0x80800000, 0xFF0102FE, 0xFFFFFFFF)
        surface = SimpleNamespace(flush=lambda: None, get_width=lambda: 2,
                                  get_height=lambda: 1, get_stride=lambda: 12,
                                  get_data=lambda: data)
        patch = overlay._surface_to_patch(surface, 3, 4)
        self.assertEqual((patch.x, patch.y, patch.width, patch.height), (3, 4, 2, 1))
        self.assertEqual(patch.rgb, bytes((255, 0, 0, 1, 2, 254)))
        self.assertEqual(patch.alpha, bytes((128, 255)))
        self.assertEqual(overlay.composite_rgb(bytes(6), 2, 1,
                         overlay.OverlayPatch(0, 0, 2, 1, patch.rgb, patch.alpha)),
                         bytes((128, 0, 0, 1, 2, 254)))

    def test_renderer_constructor_and_empty_text_do_not_load_gi_or_cairo(self):
        with mock.patch('builtins.__import__', side_effect=AssertionError):
            renderer = overlay.TextOverlayRenderer(1280, 720)
            patch = renderer.render('')
            self.assertIs(renderer.render(''), patch)
        self.assertEqual((patch.width, patch.height, patch.rgb, patch.alpha), (0, 0, b'', b''))


if __name__ == '__main__':
    unittest.main()
