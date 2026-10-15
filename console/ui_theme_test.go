package main

import (
	"math"
	"testing"

	"github.com/egoist/mygo/ui"
)

func contrast(a, b ui.Color) float64 {
	luminance := func(c ui.Color) float64 {
		channel := func(v uint8) float64 {
			s := float64(v) / 255
			if s <= .04045 {
				return s / 12.92
			}
			return math.Pow((s+.055)/1.055, 2.4)
		}
		return .2126*channel(c.R) + .7152*channel(c.G) + .0722*channel(c.B)
	}
	x, y := luminance(a), luminance(b)
	return (math.Max(x, y) + .05) / (math.Min(x, y) + .05)
}

// These checks protect the palette; native-window checks additionally catch
// renderers painting glyphs in the wrong colour despite correct theme values.
func TestThemeReadability(t *testing.T) {
	for _, name := range []string{"dark", "light"} {
		t.Run(name, func(t *testing.T) {
			p := themeFor(name)
			check := func(label string, fg, bg ui.Color, minimum float64) {
				t.Helper()
				if ratio := contrast(fg, bg); ratio < minimum {
					t.Errorf("%s: %.2f:1, need %.1f:1", label, ratio, minimum)
				}
			}
			for _, bg := range []ui.Color{p.Background, p.Surface, p.SurfaceHover, p.SurfacePressed} {
				check("body", p.Text, bg, 4.5)
				check("secondary", p.TextMuted, bg, 4.5)
				check("control border", p.Border, bg, 3)
				check("focus", p.Focus, bg, 3)
			}
			for _, bg := range []ui.Color{p.Accent, p.AccentHover, p.AccentPressed} {
				check("primary action", p.AccentText, bg, 4.5)
			}
			for _, fg := range []ui.Color{p.Success, p.Warning, p.Danger} {
				check("status", fg, p.Background, 4.5)
				check("status on panel", fg, p.Surface, 4.5)
			}
			check("selected text", p.Text, p.Selection, 4.5)
			check("tooltip", p.InverseText, p.Inverse, 4.5)
		})
	}
}
