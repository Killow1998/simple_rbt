package main

import (
	"testing"

	"github.com/egoist/mygo/ui"
)

// The menu keeps canonical choices even if its language changes while open.
// No native window, simulated robot or desktop input is used by this test.
func TestLocalizedSelectPreservesCanonicalState(t *testing.T) {
	for _, test := range []struct {
		name    string
		choices []string
	}{
		{"terrain", []string{"flat", "ramp"}},
		{"gizmo", []string{"Move / 平移", "Rotate / 旋转"}},
		{"source", []string{manualLabel, programLabel, ros1Label}},
	} {
		t.Run(test.name, func(t *testing.T) {
			a := &app{preferences: Preferences{Language: "en"}}
			selected, changes := test.choices[0], 0
			tt := ui.NewTester(func(c *ui.Context) {
				if a.localizedSelect(c.Key("choice"), &selected, test.choices).Width(280).Label("choice").Changed() {
					changes++
				}
			}, 320, 240)
			if err := tt.Click("choice"); err != nil {
				t.Fatal(err)
			}
			a.preferences.Language = "zh-CN"
			tt.Frame()
			if selected != test.choices[0] || changes != 0 || !tt.HasText(a.optionLabel(test.choices[1])) {
				t.Fatalf("language switch changed canonical state or lost translated options: %q, changes %d", selected, changes)
			}
			tt.Key(0, ui.KeyDown)
			tt.Key(0, ui.KeyEnter)
			if selected != test.choices[1] || changes == 0 {
				t.Fatalf("translated keyboard choice did not select canonical value: %q", selected)
			}
			before := changes
			a.preferences.Language = "en"
			tt.Frame()
			if selected != test.choices[1] || changes != before {
				t.Fatal("language switch emitted a new configuration/control change")
			}
			if test.name == "source" && sourceName(selected) != "program" {
				t.Fatal("translated command source changed its IPC identity")
			}
		})
	}
}
