"""Real XTEST events on an isolated X11 display; no simulator/UI callbacks."""
import ctypes as C
from collections import Counter
import struct
import time
import zlib


def luminance(rgb):
    channels = [v/3294.6 if v <= 10 else ((v/255+.055)/1.055)**2.4 for v in rgb]
    return sum(v*weight for v, weight in zip(channels, (.2126, .7152, .0722)))


def contrast(first, second):
    light, dark = sorted((luminance(first), luminance(second)), reverse=True)
    return (light+.05)/(dark+.05)


class TextProperty(C.Structure):
    _fields_ = [("value", C.c_void_p), ("encoding", C.c_ulong),
                ("format", C.c_int), ("count", C.c_ulong)]


class XImage(C.Structure):
    _fields_ = [(name, C.c_int) for name in ("width", "height", "xoffset", "format")] + [
        ("data", C.c_void_p)] + [(name, C.c_int) for name in (
        "byte_order", "bitmap_unit", "bitmap_bit_order", "bitmap_pad", "depth", "stride", "bpp")] + [
        ("red", C.c_ulong), ("green", C.c_ulong), ("blue", C.c_ulong)]


class Inputs:
    def __init__(self, display):
        self.x = C.CDLL("libX11.so.6")
        self.t = C.CDLL("libXtst.so.6")
        self.x.XOpenDisplay.argtypes = [C.c_char_p]
        self.x.XOpenDisplay.restype = C.c_void_p
        self.d = self.x.XOpenDisplay(display.encode())
        if not self.d:
            raise RuntimeError("Cannot open isolated X11 display")
        signatures = {
            "XDefaultRootWindow": ([C.c_void_p], C.c_ulong),
            "XQueryTree": ([C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_ulong), C.POINTER(C.POINTER(C.c_ulong)), C.POINTER(C.c_uint)], C.c_int),
            "XGetWMName": ([C.c_void_p, C.c_ulong, C.POINTER(TextProperty)], C.c_int),
            "XInternAtom": ([C.c_void_p, C.c_char_p, C.c_int], C.c_ulong),
            "XGetWindowProperty": ([C.c_void_p, C.c_ulong, C.c_ulong, C.c_long, C.c_long,
                                    C.c_int, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_int),
                                    C.POINTER(C.c_ulong), C.POINTER(C.c_ulong), C.POINTER(C.c_void_p)], C.c_int),
            "XFree": ([C.c_void_p], C.c_int),
            "XSetInputFocus": ([C.c_void_p, C.c_ulong, C.c_int, C.c_ulong], C.c_int),
            "XMoveWindow": ([C.c_void_p, C.c_ulong, C.c_int, C.c_int], C.c_int),
            "XCreateSimpleWindow": ([C.c_void_p, C.c_ulong, C.c_int, C.c_int, C.c_uint,
                                      C.c_uint, C.c_uint, C.c_ulong, C.c_ulong], C.c_ulong),
            "XMapWindow": ([C.c_void_p, C.c_ulong], C.c_int),
            "XDestroyWindow": ([C.c_void_p, C.c_ulong], C.c_int),
            "XGetInputFocus": ([C.c_void_p, C.POINTER(C.c_ulong), C.POINTER(C.c_int)], C.c_int),
            "XGetGeometry": ([C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_int),
                               C.POINTER(C.c_int), C.POINTER(C.c_uint), C.POINTER(C.c_uint),
                               C.POINTER(C.c_uint), C.POINTER(C.c_uint)], C.c_int),
            "XGetImage": ([C.c_void_p, C.c_ulong, C.c_int, C.c_int, C.c_uint, C.c_uint,
                           C.c_ulong, C.c_int], C.POINTER(XImage)),
            "XDestroyImage": ([C.c_void_p], C.c_int),
            "XFlush": ([C.c_void_p], C.c_int),
            "XStringToKeysym": ([C.c_char_p], C.c_ulong),
            "XKeysymToKeycode": ([C.c_void_p, C.c_ulong], C.c_ubyte),
            "XCloseDisplay": ([C.c_void_p], C.c_int),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(self.x, name)
            function.argtypes, function.restype = arguments, result
        self.t.XTestFakeMotionEvent.argtypes = [C.c_void_p, C.c_int, C.c_int, C.c_int, C.c_ulong]
        self.t.XTestFakeButtonEvent.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_ulong]
        self.t.XTestFakeKeyEvent.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_ulong]
        self.root = self.x.XDefaultRootWindow(self.d)
        self.title_atom = self.x.XInternAtom(self.d, b"_NET_WM_NAME", 0)
        self.dummy = None

    def window(self):
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            result = self.find_window()
            if result:
                self.x.XMoveWindow(self.d, result, 0, 0)
                self.x.XFlush(self.d)
                time.sleep(.1)
                return result
            time.sleep(.1)
        raise RuntimeError("Native console window not found: " + repr(self.titles))

    def find_window(self):
        root, parent, count, children = C.c_ulong(), C.c_ulong(), C.c_uint(), C.POINTER(C.c_ulong)()
        self.x.XQueryTree(self.d, self.root, C.byref(root), C.byref(parent), C.byref(children), C.byref(count))
        result = None
        self.titles = []
        for i in range(count.value):
            name = TextProperty()
            title = ""
            if self.x.XGetWMName(self.d, children[i], C.byref(name)) and name.value:
                title = C.string_at(name.value, name.count).decode(errors="replace")
                self.x.XFree(name.value)
            # WM_NAME may use legacy COMPOUND_TEXT for Chinese. Prefer the
            # EWMH UTF-8 title even when WM_NAME contains the ASCII app name.
            kind, fmt, size, after, data = C.c_ulong(), C.c_int(), C.c_ulong(), C.c_ulong(), C.c_void_p()
            self.x.XGetWindowProperty(self.d, children[i], self.title_atom, 0, 1024, 0, 0,
                C.byref(kind), C.byref(fmt), C.byref(size), C.byref(after), C.byref(data))
            if data:
                if fmt.value == 8:
                    title = C.string_at(data, size.value).decode(errors="replace")
                self.x.XFree(data)
            if title:
                self.titles.append(title)
            if "simple_rbt" in title or "SpRobo" in title:
                result = children[i]
        if children:
            self.x.XFree(children)
        return result

    def focus(self, window):
        self.x.XSetInputFocus(self.d, window, 2, 0)
        self.x.XFlush(self.d)
        time.sleep(.08)

    def blur(self):
        self.dummy = self.x.XCreateSimpleWindow(self.d, self.root, 1400, 970, 20, 20, 0, 0, 0)
        self.x.XMapWindow(self.d, self.dummy)
        self.x.XFlush(self.d)
        time.sleep(.08)
        self.focus(self.dummy)
        focused, revert = C.c_ulong(), C.c_int()
        self.x.XGetInputFocus(self.d, C.byref(focused), C.byref(revert))
        if focused.value != self.dummy:
            raise RuntimeError("X11 focus did not leave the console")

    def move(self, x, y):
        self.t.XTestFakeMotionEvent(self.d, 0, round(x), round(y), 0)
        self.x.XFlush(self.d)
        time.sleep(.025)

    def button(self, pressed):
        self.t.XTestFakeButtonEvent(self.d, 1, bool(pressed), 0)
        self.x.XFlush(self.d)
        time.sleep(.025)

    def click(self, x, y):
        self.move(x, y)
        self.button(True)
        self.button(False)

    def key(self, name, pressed):
        symbol = self.x.XStringToKeysym(name.encode())
        code = self.x.XKeysymToKeycode(self.d, symbol)
        if not code:
            raise RuntimeError("Key unavailable: " + name)
        self.t.XTestFakeKeyEvent(self.d, code, bool(pressed), 0)
        self.x.XFlush(self.d)
        time.sleep(.02)

    def number(self, x, y, value):
        self.click(x, y)
        self.type_number(value)

    def press(self, name):
        self.key(name, True)
        self.key(name, False)

    def tab(self, reverse=False):
        if reverse:
            self.key("Shift_L", True)
        self.press("Tab")
        if reverse:
            self.key("Shift_L", False)

    def select_edge(self, last=False):
        self.press("Return")
        self.press("End" if last else "Home")
        self.press("Return")

    def type_number(self, value):
        self.key("Control_L", True)
        self.key("a", True)
        self.key("a", False)
        self.key("Control_L", False)
        for character in value:
            name = {".": "period", "-": "minus"}.get(character, character)
            self.key(name, True)
            self.key(name, False)
        self.key("Return", True)
        self.key("Return", False)

    def pixels(self, window):
        root, x, y = C.c_ulong(), C.c_int(), C.c_int()
        width, height, border, depth = (C.c_uint() for _ in range(4))
        if not self.x.XGetGeometry(self.d, window, C.byref(root), C.byref(x), C.byref(y),
                                  C.byref(width), C.byref(height), C.byref(border), C.byref(depth)):
            raise RuntimeError("Cannot read native window geometry")
        width, height = width.value, height.value
        image = self.x.XGetImage(self.d, window, 0, 0, width, height, C.c_ulong(-1).value, 2)
        if not image:
            raise RuntimeError("Cannot capture the actual X11 window")
        try:
            info = image.contents
            if info.bpp != 32 or info.byte_order != 0 or (info.red, info.green, info.blue) != (0xff0000, 0xff00, 0xff):
                raise RuntimeError("Native test display needs 32-bit little-endian BGR pixels")
            raw = C.string_at(info.data, info.stride*height)
            pixels = bytearray(width*height*3)
            for y in range(height):
                row = raw[y*info.stride:y*info.stride+width*4]
                start = y*width*3
                pixels[start:start+width*3:3] = row[2::4]
                pixels[start+1:start+width*3:3] = row[1::4]
                pixels[start+2:start+width*3:3] = row[0::4]
            return width, height, pixels
        finally:
            self.x.XDestroyImage(image)

    def viewport(self, window, image_width, image_height):
        """Locate the actual rendered rectangle, independent of font line heights."""
        width, height, pixels = self.pixels(window)
        # Compare the image to the adjacent pane padding. Both the image and
        # controls can be dark; absolute brightness is not a rectangle boundary.
        left, right = width-18-image_width, width-18
        def image_row(y):
            outside = pixels[(y*width+right+8)*3:(y*width+right+8)*3+3]
            return sum(max(abs(a-b) for a, b in zip(
                pixels[(y*width+x)*3:(y*width+x)*3+3], outside)) > 12
                for x in (right-16, right-24, right-32)) >= 2
        runs, start = [], None
        for y in range(66, height):
            if image_row(y):
                if start is None:
                    start = y
            elif start is not None:
                runs.append((start, y-start))
                start = None
        if start is not None:
            runs.append((start, height-start))
        # A scene row can share the pane's color. Use the observer's actual
        # height to join those small holes, while requiring scene evidence
        # across at least 90% of the candidate rectangle.
        matches = []
        for i, (top, _) in enumerate(runs):
            covered = 0
            for bottom, length in runs[i:]:
                covered += length
                extent = bottom+length-top
                if extent > image_height+2:
                    break
                if abs(extent-image_height) <= 2 and covered >= .9*extent:
                    matches.append((top, extent))
        if len(matches) != 1:
            raise RuntimeError(f"Cannot identify native observer rectangle {image_width}x{image_height}: {runs}")
        return left, matches[0][0], image_width, image_height

    def primary_button(self, window):
        """Find the wide contrasting Start fill, independent of its accent hue."""
        width, height, pixels = self.pixels(window)
        def filled(x, y):
            color = pixels[(y*width+x)*3:(y*width+x)*3+3]
            outside = pixels[(y*width+8)*3:(y*width+8)*3+3]
            return contrast(color, outside) > 3
        rows = [y for y in range(66, height)
                if sum(filled(x, y) for x in range(30, 330, 10)) >= 24]
        runs = []
        for y in rows:
            if not runs or y > runs[-1][-1]+2:
                runs.append([y])
            else:
                runs[-1].append(y)
        fills = [run for run in runs if 12 <= run[-1]-run[0] <= 50]
        if len(fills) != 1:
            raise RuntimeError("Cannot identify the native Start button")
        return 180, (fills[0][0]+fills[0][-1])/2

    def readability(self, window):
        """Measure rendered glyph interiors, not theme constants or GPU callbacks.

        These regions contain ordinary title/body text, not status or accent
        colors. Repeated high-contrast pixels capture solid glyph interiors
        without treating antialiased edge pixels as unreadable.
        """
        width, height, pixels = self.pixels(window)
        regions = {"title": (18, 10, 110, 55),
                   "subtitle": (132, 12, 450, 52),
                   "selected value": (30, 135, 280, 170)}
        evidence = []
        for name, (left, top, right, bottom) in regions.items():
            colors = Counter(tuple(pixels[(y*width+x)*3:(y*width+x)*3+3])
                for y in range(top, min(bottom, height))
                for x in range(left, min(right, width)))
            background, _ = colors.most_common(1)[0]
            candidates = [(rgb, count) for rgb, count in colors.items()
                          if contrast(rgb, background) >= 2]
            interiors = [(rgb, count) for rgb, count in candidates if count >= 3]
            foreground, count = max(interiors, key=lambda item: contrast(item[0], background), default=(background, 0))
            count = sum(n for rgb, n in candidates if max(abs(a-b) for a, b in zip(rgb, foreground)) <= 12)
            ratio = contrast(foreground, background)
            ink_pixels = sum(count for _, count in candidates)
            colored_pixels = sum(count for rgb, count in candidates if max(rgb)-min(rgb) > 45)
            colored_fraction = colored_pixels/max(1, ink_pixels)
            passed = count >= 20 and ratio >= 4.5 and colored_fraction <= .05
            evidence.append({"region": name, "background_rgb": background,
                "text_rgb": foreground, "solid_text_pixels": count,
                "contrast_ratio": round(ratio, 2),
                "unexpected_color_fraction": round(colored_fraction, 3), "passed": passed})
        return all(item["passed"] for item in evidence), evidence

    def capture(self, window, path):
        width, height, pixels = self.pixels(window)
        rows = bytearray()
        for y in range(height):
            rows.append(0)
            rows.extend(pixels[y*width*3:(y+1)*width*3])
        def chunk(kind, data):
            return struct.pack(">I",len(data))+kind+data+struct.pack(">I",zlib.crc32(kind+data))
        path.write_bytes(b"\x89PNG\r\n\x1a\n"+
            chunk(b"IHDR",struct.pack(">IIBBBBB",width,height,8,2,0,0,0))+
            chunk(b"IDAT",zlib.compress(rows))+chunk(b"IEND",b""))

    def close(self):
        if self.dummy:
            self.x.XDestroyWindow(self.d, self.dummy)
        self.x.XCloseDisplay(self.d)
