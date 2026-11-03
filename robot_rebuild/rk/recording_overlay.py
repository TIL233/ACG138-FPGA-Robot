"""Render only the HDMI status text and blend it into shared RGB camera frames.

Create/render ``TextOverlayRenderer`` on the GTK main thread. Its immutable
``OverlayPatch`` can then be handed to the recording worker; compositing needs
only NumPy and never reads the display or captures a desktop.
"""
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class OverlayPatch:
    x: int
    y: int
    width: int
    height: int
    rgb: bytes
    alpha: bytes

    def __post_init__(self):
        # Freeze buffer inputs too, so a patch is safe to share with the worker.
        for name in ('rgb', 'alpha'):
            value = getattr(self, name)
            if not isinstance(value, bytes):
                object.__setattr__(self, name, memoryview(value).tobytes())


def _geometry(width, height, *, empty=False):
    minimum = 0 if empty else 1
    if (type(width) is not int or type(height) is not int
            or width < minimum or height < minimum):
        raise ValueError('width and height must be positive integers' if not empty
                         else 'patch width and height must be nonnegative integers')


def composite_rgb(rgb: bytes, width: int, height: int,
                  patch: Optional[OverlayPatch]) -> bytes:
    """Return a new packed RGB frame, using straight (unpremultiplied) alpha.

    Geometry and exact buffer lengths are checked even for a transparent patch.
    The camera buffer and patch buffers are never modified.
    """
    _geometry(width, height)
    if len(rgb) != width * height * 3:
        raise ValueError('RGB frame length does not match its geometry')
    if patch is None:
        return memoryview(rgb).tobytes()
    if not isinstance(patch, OverlayPatch):
        raise TypeError('patch must be an OverlayPatch or None')
    _geometry(patch.width, patch.height, empty=True)
    if (type(patch.x) is not int or type(patch.y) is not int
            or patch.x < 0 or patch.y < 0
            or patch.x + patch.width > width
            or patch.y + patch.height > height):
        raise ValueError('overlay patch lies outside the RGB frame')
    area = patch.width * patch.height
    if len(patch.rgb) != area * 3 or len(patch.alpha) != area:
        raise ValueError('overlay buffer lengths do not match its geometry')
    if area == 0:
        return memoryview(rgb).tobytes()

    import numpy as np
    frame = np.frombuffer(rgb, dtype=np.uint8).reshape(height, width, 3).copy()
    region = frame[patch.y:patch.y + patch.height, patch.x:patch.x + patch.width]
    foreground = np.frombuffer(patch.rgb, dtype=np.uint8).reshape(
        patch.height, patch.width, 3).astype(np.uint16)
    alpha = np.frombuffer(patch.alpha, dtype=np.uint8).reshape(
        patch.height, patch.width, 1).astype(np.uint16)
    # The two weighted terms sum to at most 65025, so uint16 cannot overflow.
    region[:] = (foreground * alpha + region.astype(np.uint16) * (255 - alpha)
                 + 127) // 255
    return frame.tobytes()


def _surface_to_patch(surface, x, y):
    """Decode Cairo's native-endian, premultiplied ARGB32 including row padding."""
    import numpy as np
    surface.flush()
    width, height = surface.get_width(), surface.get_height()
    # Reading native uint32 values works for BGRA bytes on little endian and
    # ARGB bytes on big endian without assuming the host's byte order.
    pixels = np.ndarray((height, width), dtype=np.uint32,
                        buffer=surface.get_data(), strides=(surface.get_stride(), 4))
    alpha = (pixels >> 24).astype(np.uint32)
    premultiplied = np.stack(((pixels >> 16) & 255, (pixels >> 8) & 255,
                             pixels & 255), axis=2)
    straight = (premultiplied * 255 + alpha[:, :, None] // 2) // np.maximum(
        alpha[:, :, None], 1)
    straight[alpha == 0] = 0
    return OverlayPatch(x, y, width, height,
                        np.minimum(straight, 255).astype(np.uint8).tobytes(),
                        alpha.astype(np.uint8).tobytes())


class TextOverlayRenderer:
    """Cache a small Pango/Cairo text panel; call ``render`` on the GTK thread.

    Noto Sans CJK SC provides Chinese glyphs on the RK image. Pango's normal
    font fallback still applies if a particular glyph needs another font.
    No GTK widget, camera, screenshot, or Pillow dependency is involved.
    """
    def __init__(self, width: int, height: int):
        _geometry(width, height)
        self.width, self.height = width, height
        self._last_text = None
        self._last_patch = None
        self._libraries = None

    def _load_libraries(self):
        if self._libraries is None:
            try:
                import gi
                gi.require_version('Pango', '1.0')
                gi.require_version('PangoCairo', '1.0')
                from gi.repository import Pango, PangoCairo
                import cairo
            except (ImportError, ValueError) as exc:
                raise RuntimeError('Chinese recording overlay requires PyGObject, '
                                   'PangoCairo and pycairo') from exc
            self._libraries = (Pango, PangoCairo, cairo)
        return self._libraries

    def render(self, text: str) -> OverlayPatch:
        if not isinstance(text, str):
            raise TypeError('overlay text must be a string')
        if text == self._last_text:
            return self._last_patch

        # Preserve the usual 16px margin, while remaining bounded for tiny input.
        margin_x = min(16, (self.width - 1) // 2)
        margin_y = min(16, (self.height - 1) // 2)
        maximum_width = self.width - 2 * margin_x
        maximum_height = min(self.height * 55 // 100,
                             self.height - 2 * margin_y)
        clean_text = text.replace('\x00', '').strip()
        if not clean_text or maximum_height < 1:
            patch = OverlayPatch(0, 0, 0, 0, b'', b'')
        else:
            Pango, PangoCairo, cairo = self._load_libraries()
            padding = min(10, (maximum_width - 1) // 2,
                          (maximum_height - 1) // 2)
            text_width = maximum_width - 2 * padding
            text_height = maximum_height - 2 * padding
            measure = cairo.Context(cairo.ImageSurface(cairo.FORMAT_ARGB32, 1, 1))
            layout = PangoCairo.create_layout(measure)
            font = Pango.FontDescription('Noto Sans CJK SC')
            font.set_absolute_size(max(12, round(16 * self.height / 720)) * Pango.SCALE)
            layout.set_font_description(font)
            layout.set_width(text_width * Pango.SCALE)
            layout.set_wrap(Pango.WrapMode.WORD_CHAR)
            layout.set_ellipsize(Pango.EllipsizeMode.END)
            layout.set_height(text_height * Pango.SCALE)

            # Bound work even for accidental unbounded logs passed as status.
            if len(clean_text) > 16384:
                clean_text = clean_text[:16384].rstrip() + '\u2026'
            layout.set_text(clean_text, -1)
            measured_width, measured_height = layout.get_pixel_size()
            if measured_height > text_height:
                # Pango guarantees a line per paragraph, so many explicit
                # newlines can exceed set_height. Find a fitting text prefix.
                low, high, fitting = 0, len(clean_text), None
                while low <= high:
                    middle = (low + high) // 2
                    candidate = clean_text[:middle].rstrip() + '\u2026'
                    layout.set_text(candidate, -1)
                    if layout.get_pixel_size()[1] <= text_height:
                        fitting = candidate
                        low = middle + 1
                    else:
                        high = middle - 1
                if fitting is None:
                    patch = OverlayPatch(0, 0, 0, 0, b'', b'')
                    self._last_text, self._last_patch = text, patch
                    return patch
                layout.set_text(fitting, -1)
                measured_width, measured_height = layout.get_pixel_size()

            panel_width = min(maximum_width, max(1, measured_width + 2 * padding))
            panel_height = min(maximum_height, max(1, measured_height + 2 * padding))
            surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, panel_width, panel_height)
            context = cairo.Context(surface)
            context.set_source_rgba(0, 0, 0, 220 / 255)
            context.paint()
            context.rectangle(padding, padding, panel_width - 2 * padding,
                              panel_height - 2 * padding)
            context.clip()
            context.set_source_rgba(1, 1, 1, 1)
            context.move_to(padding, padding)
            PangoCairo.update_layout(context, layout)
            PangoCairo.show_layout(context, layout)
            patch = _surface_to_patch(surface, margin_x,
                                      self.height - margin_y - panel_height)
        self._last_text, self._last_patch = text, patch
        return patch
