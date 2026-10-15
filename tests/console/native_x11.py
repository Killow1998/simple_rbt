"""Real XTEST events on an isolated X11 display; no simulator/UI callbacks."""
import ctypes as C
import struct
import time
import zlib


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
            if "simple_rbt" not in title and "SpRobo" not in title:
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
        # The right pane has a fixed 18px inset. Sample inside the rounded
        # image corners; the image is dark, while surrounding controls are light.
        left, right = width-18-image_width, width-18
        def dark(y):
            return sum(max(pixels[(y*width+x)*3:(y*width+x)*3+3]) < 220
                       for x in (right-16, right-24, right-32)) >= 2
        runs, start = [], None
        for y in range(66, height):
            if dark(y):
                if start is None:
                    start = y
            elif start is not None:
                runs.append((start, y-start))
                start = None
        matches = [(y, h) for y, h in runs if abs(h-image_height) <= 2]
        if len(matches) != 1:
            raise RuntimeError(f"Cannot identify native observer rectangle {image_width}x{image_height}: {runs}")
        return left, matches[0][0], image_width, image_height

    def primary_button(self, window):
        """Find the large blue Start button, rather than assuming a footer height."""
        width, height, pixels = self.pixels(window)
        def blue(x, y):
            r, g, b = pixels[(y*width+x)*3:(y*width+x)*3+3]
            return b > 180 and b > r+80 and g > r+40
        rows = [y for y in range(66, height)
                if sum(blue(x, y) for x in range(30, 330, 10)) >= 24]
        if not rows or rows[-1]-rows[0] > 50:
            raise RuntimeError("Cannot identify the native Start button")
        return 180, (rows[0]+rows[-1])/2

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
