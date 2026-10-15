package main

import "github.com/egoist/mygo/ui"

// Keep the console legible independently of the desktop's appearance. Colours
// for text, controls and their interaction states belong to the same palette.
var darkTheme = ui.Theme{
	Dark:           true,
	Background:     ui.Hex("#191d24"),
	Surface:        ui.Hex("#2a323e"),
	SurfaceHover:   ui.Hex("#333e4c"),
	SurfacePressed: ui.Hex("#3c495a"),
	Border:         ui.Hex("#8999ad"),
	Text:           ui.Hex("#e3e7ed"),
	TextMuted:      ui.Hex("#b5bfcd"),
	Accent:         ui.Hex("#93b4dc"),
	AccentHover:    ui.Hex("#adc7e7"),
	AccentPressed:  ui.Hex("#83a2c7"),
	AccentText:     ui.Hex("#17212e"),
	Danger:         ui.Hex("#e8aaa6"),
	Warning:        ui.Hex("#dac08e"),
	Success:        ui.Hex("#a6c6b5"),
	Selection:      ui.Hex("#465d7a"),
	Focus:          ui.Hex("#b0c9e9"),
	Inverse:        ui.Hex("#d8dfe8"),
	InverseText:    ui.Hex("#191d24"),
	Scrollbar:      ui.Hex("#8190a4"),
	ScrollbarWidth: 6,
	Radius:         7,
	Spacing:        4,
	FontSize:       14,
}

var lightTheme = ui.Theme{
	Background:     ui.Hex("#eef1f5"),
	Surface:        ui.Hex("#e2e7ed"),
	SurfaceHover:   ui.Hex("#d5dde6"),
	SurfacePressed: ui.Hex("#c4ced9"),
	Border:         ui.Hex("#607087"),
	Text:           ui.Hex("#202c3a"),
	TextMuted:      ui.Hex("#45556a"),
	Accent:         ui.Hex("#335f8d"),
	AccentHover:    ui.Hex("#294f79"),
	AccentPressed:  ui.Hex("#234466"),
	AccentText:     ui.Hex("#ffffff"),
	Danger:         ui.Hex("#953d44"),
	Warning:        ui.Hex("#79541e"),
	Success:        ui.Hex("#34634f"),
	Selection:      ui.Hex("#bacfe4"),
	Focus:          ui.Hex("#335f8d"),
	Inverse:        ui.Hex("#293544"),
	InverseText:    ui.Hex("#eef1f5"),
	Scrollbar:      ui.Hex("#607087"),
	ScrollbarWidth: 6,
	Radius:         7,
	Spacing:        4,
	FontSize:       14,
}

func themeFor(name string) *ui.Theme {
	if name == "light" {
		return &lightTheme
	}
	return &darkTheme
}

func panelBackground(c *ui.Context) ui.Color {
	if c.Theme().Dark {
		return ui.Hex("#222831")
	}
	return ui.Hex("#f6f7f9")
}

func panelDivider(c *ui.Context) ui.Color {
	if c.Theme().Dark {
		return ui.Hex("#3e4856")
	}
	return ui.Hex("#c8d0da")
}

// Sensor imagery keeps a neutral dark surround in either appearance.
var viewportBackground = ui.Hex("#151b24")
var viewportText = ui.Hex("#b5bfcd")

func stopBackground(c *ui.Context) ui.Color {
	if c.Theme().Dark {
		return ui.Hex("#84464d")
	}
	return ui.Hex("#f2d9dc")
}
