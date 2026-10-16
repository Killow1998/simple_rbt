package main

import (
	"github.com/egoist/mygo/ui"
	"math"
)

func (a *app) openingInspector(c *ui.Context, o *SceneObject) {
	if o.Kind != "floor" {
		return
	}
	enabled := o.Opening != nil
	if ui.Checkbox(c.Key("opening-enabled"), &enabled, a.tr("Cut rectangular opening", "开矩形洞口")).Changed() {
		o.Opening = nil
		if enabled {
			o.Opening = &SlabOpening{Size: [2]float64{o.Size[0] * .3, o.Size[1] * .3}}
		}
	}
	if o.Opening == nil {
		return
	}
	for i, axis := range []string{"X", "Y"} {
		for _, dimension := range []bool{true, false} {
			label := a.tr("Opening centre ", "洞口中心 ") + axis
			value := &o.Opening.Center[i]
			limit := o.Size[i]/2 - o.Opening.Size[i]/2
			lo, hi := -limit, limit
			if dimension {
				label = a.tr("Opening size ", "洞口尺寸 ") + axis
				value = &o.Opening.Size[i]
				lo, hi = .02, o.Size[i]-2*math.Abs(o.Opening.Center[i])
			}
			field(c, label, func() {
				el := ui.NumberInput(c.Key(label), value, lo, hi, .1).FillWidth().Label(label)
				a.sceneFieldFocus(el, label)
			})
		}
	}
	ui.Text(c, a.tr("Local slab coordinates, metres. The opening cuts through physics and sensor geometry.", "使用楼板局部坐标，单位为米。洞口会贯穿物理和传感器几何。")).FontSize(12).TextColor(c.Theme().TextMuted)
}
