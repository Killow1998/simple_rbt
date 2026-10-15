// Verification helper only: capture actual X11 pixels and close the test window.
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <png.h>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

Window find_window(Display* display, Window parent) {
  char* title = nullptr;
  Atom type{};
  int format{};
  unsigned long count{}, remaining{};
  unsigned char* utf8{};
  std::string name;
  if (XGetWindowProperty(display, parent, XInternAtom(display, "_NET_WM_NAME", False),
                         0, 1024, False, AnyPropertyType, &type, &format,
                         &count, &remaining, &utf8) == Success && utf8 && format == 8)
    name.assign(reinterpret_cast<char*>(utf8), count);
  if (utf8) XFree(utf8);
  XWindowAttributes attributes{};
  const bool titled = XFetchName(display, parent, &title) && title;
  if (name.empty() && titled) name = title;
  const bool match = name.find("SpRobo") != std::string::npos ||
                     name.find("simple_rbt") != std::string::npos;
  if (title) XFree(title);
  if (match && XGetWindowAttributes(display, parent, &attributes) &&
      attributes.map_state == IsViewable && attributes.width > 500)
    return parent;
  Window root{}, owner{}, *children{};
  unsigned child_count{};
  Window result{};
  if (XQueryTree(display, parent, &root, &owner, &children, &child_count)) {
    for (unsigned i = 0; i < child_count && !result; ++i)
      result = find_window(display, children[i]);
    if (children) XFree(children);
  }
  return result;
}

unsigned char channel(unsigned long pixel, unsigned long mask) {
  if (!mask) return 0;
  unsigned shift = 0;
  while (!(mask & 1)) { mask >>= 1; ++shift; }
  return static_cast<unsigned char>(((pixel >> shift) & mask) * 255 / mask);
}

void write_png(const char* path, XImage* image) {
  FILE* file = std::fopen(path, "wb");
  if (!file) throw std::runtime_error("Cannot write native screenshot");
  png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png) { std::fclose(file); throw std::runtime_error("PNG allocation failed"); }
  png_infop info = png_create_info_struct(png);
  if (!png || !info || setjmp(png_jmpbuf(png))) {
    std::fclose(file);
    throw std::runtime_error("PNG encoding failed");
  }
  png_init_io(png, file);
  png_set_IHDR(png, info, image->width, image->height, 8, PNG_COLOR_TYPE_RGB,
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
  png_write_info(png, info);
  std::vector<unsigned char> row(image->width * 3);
  for (int y = 0; y < image->height; ++y) {
    for (int x = 0; x < image->width; ++x) {
      const unsigned long pixel = XGetPixel(image, x, y);
      row[x*3] = channel(pixel, image->red_mask);
      row[x*3+1] = channel(pixel, image->green_mask);
      row[x*3+2] = channel(pixel, image->blue_mask);
    }
    png_write_row(png, row.data());
  }
  png_write_end(png, info);
  png_destroy_write_struct(&png, &info);
  std::fclose(file);
}

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("Usage: capture-x11 OUTPUT.png");
    Display* display = XOpenDisplay(nullptr);
    if (!display) throw std::runtime_error("X11 display is unavailable");
    const Window root = DefaultRootWindow(display);
    const Window window = find_window(display, root);
    if (!window) throw std::runtime_error("Native SpRobo window not found");
    XWindowAttributes attributes{};
    XGetWindowAttributes(display, window, &attributes);
    XSync(display, False);
    XImage* image = XGetImage(display, window, 0, 0, attributes.width, attributes.height,
                             AllPlanes, ZPixmap);
    if (!image) throw std::runtime_error("Cannot read X11 window pixels");
    write_png(argv[1], image);
    XDestroyImage(image);
    XEvent close{};
    close.xclient.type = ClientMessage;
    close.xclient.window = window;
    close.xclient.message_type = XInternAtom(display, "WM_PROTOCOLS", False);
    close.xclient.format = 32;
    close.xclient.data.l[0] = XInternAtom(display, "WM_DELETE_WINDOW", False);
    close.xclient.data.l[1] = CurrentTime;
    XSendEvent(display, window, False, NoEventMask, &close);
    XFlush(display);
    XCloseDisplay(display);
    std::cout << "Actual native X11 pixels captured; normal close requested\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
