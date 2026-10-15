package main

const manualLabel = "Manual / 手动"
const programLabel = "Program · IPC / 程序 · IPC"
const ros1Label = "Program · ROS 1 / 程序 · ROS 1"
const ros2Label = "Program · ROS 2 / 程序 · ROS 2"

func sourceLabel(source string) string {
	switch source {
	case "ros1":
		return ros1Label
	case "ros2":
		return ros2Label
	case "program":
		return programLabel
	case "gui":
		return manualLabel
	}
	return "None / 未设置"
}
func sourceName(label string) string {
	switch label {
	case ros1Label:
		return "ros1"
	case ros2Label:
		return "ros2"
	case programLabel:
		return "program"
	}
	return "gui"
}
func (a *app) manualMode() bool {
	return a.stage == "operate" && a.meta.State == "running" && a.meta.Source == "gui" && a.sourceLabel == manualLabel
}
func (a *app) blurManual() {
	if a.manualMode() {
		a.stopManual()
	}
}
func (a *app) relinquishManual() {
	a.pad = joystick{}
	a.commandMu.Lock()
	a.manualHeartbeat = false
	a.command = Vec3{}
	a.commandMu.Unlock()
}
func (a *app) sendManualHeartbeat() {
	a.commandMu.Lock()
	defer a.commandMu.Unlock()
	if a.manualHeartbeat {
		_ = a.manager.Client.SendCommand([3]float64(a.command))
	}
}
