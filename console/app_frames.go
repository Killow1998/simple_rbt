package main

import (
	"fmt"
	"image"
	"math"

	"github.com/egoist/mygo/ui"
	"simple-rbt/console/session"
)

func (a *app) consume() {
	packets := a.manager.Client.Poll()
	if a.restartPending {
		return
	}
	for _, packet := range packets {
		switch packet.Kind {
		case session.KindMetadata:
			data, err := session.DecodeJSON(packet)
			if err != nil {
				a.status = err.Error()
				continue
			}
			a.applyMetadata(decodeAs[metadata](data))
		case session.KindView, session.KindCamera, session.KindPanorama:
			frame, err := session.DecodeCamera(packet)
			if err != nil {
				a.status = err.Error()
				continue
			}
			if packet.Kind == session.KindPanorama && (a.cfg.Sensors.Camera || a.cfg.Sensors.Depth) {
				continue
			}
			pixels := framePixels(frame)
			if pixels == nil {
				continue
			}
			if packet.Kind == session.KindView {
				if a.observer == nil {
					a.observer = ui.NewBitmap(pixels)
				} else {
					a.observer.Update(pixels)
				}
				a.observerSize = [2]int{frame.Width, frame.Height}
				a.viewStamp = packet.StampNS
			} else {
				if a.camera == nil {
					a.camera = ui.NewBitmap(pixels)
				} else {
					a.camera.Update(pixels)
				}
				a.cameraSize = [2]int{frame.Width, frame.Height}
				a.cameraStamp = packet.StampNS
			}
		case session.KindError:
			data, _ := session.DecodeJSON(packet)
			a.status = fmt.Sprint(data["message"])
			a.showLog = true
			a.pending = map[string]Pose{}
			for name, pose := range a.meta.Mounts {
				a.cfg.Mounts[name] = pose
			}
		case session.KindOdometry:
			if value, err := session.DecodeOdometry(packet); err == nil {
				a.velocity = Vec3(value.Linear)
				a.angular = Vec3(value.Angular)
				a.odomStamp = packet.StampNS
			}
		}
	}
	if !a.manager.Client.Connected() {
		a.stopManual()
		a.loading = true
	}
}
func framePixels(frame session.CameraFrame) *image.RGBA {
	if frame.Width <= 0 || frame.Height <= 0 {
		return nil
	}
	out := image.NewRGBA(image.Rect(0, 0, frame.Width, frame.Height))
	pixels := frame.Width * frame.Height
	if len(frame.RGB) == pixels*3 {
		for i := 0; i < pixels; i++ {
			copy(out.Pix[i*4:i*4+3], frame.RGB[i*3:i*3+3])
			out.Pix[i*4+3] = 255
		}
		return out
	}
	if len(frame.Depth) == pixels {
		for i, d := range frame.Depth {
			intensity := byte(0)
			if !math.IsInf(float64(d), 0) && !math.IsNaN(float64(d)) && d > 0 {
				intensity = byte(255 * clamp(1-float64(d)/20, 0, 1))
			}
			out.Pix[i*4] = intensity
			out.Pix[i*4+1] = intensity
			out.Pix[i*4+2] = intensity
			out.Pix[i*4+3] = 255
		}
		return out
	}
	return nil
}
