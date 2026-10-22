package main

import (
	"encoding/binary"
	"encoding/json"
	"io"
	"net"
	"path/filepath"
	"testing"
	"time"

	"simple-rbt/console/session"
)

type observedPacket struct {
	kind uint16
	data []byte
}

func readControl(conn net.Conn) (observedPacket, error) {
	header := make([]byte, 24)
	if _, err := io.ReadFull(conn, header); err != nil {
		return observedPacket{}, err
	}
	packet := observedPacket{kind: binary.LittleEndian.Uint16(header[6:]), data: make([]byte, binary.LittleEndian.Uint32(header[16:]))}
	_, err := io.ReadFull(conn, packet.data)
	return packet, err
}
func controlPeer(t *testing.T) (string, <-chan observedPacket) {
	t.Helper()
	dir := t.TempDir()
	control, err := net.Listen("unix", filepath.Join(dir, "control.sock"))
	if err != nil {
		t.Fatal(err)
	}
	data, err := net.Listen("unix", filepath.Join(dir, "data.sock"))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { control.Close(); data.Close() })
	packets := make(chan observedPacket, 32)
	go func() {
		conn, err := control.Accept()
		if err != nil {
			return
		}
		defer conn.Close()
		if _, err = readControl(conn); err != nil {
			return
		}
		header := make([]byte, 26)
		copy(header, "SRBT")
		binary.LittleEndian.PutUint16(header[4:], 1)
		binary.LittleEndian.PutUint16(header[6:], uint16(session.KindHello))
		binary.LittleEndian.PutUint32(header[16:], 2)
		copy(header[24:], "{}")
		if _, err = conn.Write(header); err != nil {
			return
		}
		for {
			packet, err := readControl(conn)
			if err != nil {
				return
			}
			packets <- packet
		}
	}()
	go func() {
		conn, err := data.Accept()
		if err == nil {
			defer conn.Close()
			_, _ = io.Copy(io.Discard, conn)
		}
	}()
	return dir, packets
}
func TestProgramFocusAndMetadataNeverSendControlOrTakeOver(t *testing.T) {
	for _, owner := range []string{"program", "ros1", "ros2"} {
		t.Run(owner, func(t *testing.T) {
			dir, packets := controlPeer(t)
			a := newApp(session.Options{StateDir: t.TempDir()}, defaultConfig())
			t.Cleanup(a.manager.Events.Close)
			if err := a.manager.Client.Connect(dir); err != nil {
				t.Fatal(err)
			}
			defer a.manager.Client.Close()
			a.stage = "operate"
			a.meta.State = "running"
			a.applyMetadata(metadata{State: "running", Source: owner, Applied: Vec3{.4, 0, 0}})
			a.command = Vec3{.7, 0, 0} // A stale local value still must not become a program command.
			a.blurManual()
			a.sendManualHeartbeat()
			select {
			case packet := <-packets:
				t.Fatalf("program focus/metadata emitted control kind %d: %s", packet.kind, packet.data)
			case <-time.After(50 * time.Millisecond):
			}
			if a.meta.Source != owner || a.meta.Applied != (Vec3{.4, 0, 0}) {
				t.Fatal("program ownership/target changed on focus loss")
			}
		})
	}
}
func TestManualFocusLossSendsZeroOnRealControlSocket(t *testing.T) {
	dir, packets := controlPeer(t)
	a := newApp(session.Options{StateDir: t.TempDir()}, defaultConfig())
	t.Cleanup(a.manager.Events.Close)
	if err := a.manager.Client.Connect(dir); err != nil {
		t.Fatal(err)
	}
	defer a.manager.Client.Close()
	a.stage = "operate"
	a.meta.State = "running"
	a.applyMetadata(metadata{State: "running", Source: "gui"})
	a.command = Vec3{.4, 0, 0}
	a.pad.Held = true
	a.blurManual()
	select {
	case packet := <-packets:
		var value map[string][]float64
		if packet.kind != uint16(session.KindCommand) || json.Unmarshal(packet.data, &value) != nil {
			t.Fatalf("manual blur packet: %+v", packet)
		}
		if len(value["velocity"]) != 3 {
			t.Fatal("manual blur omitted its velocity target")
		}
		for _, number := range value["velocity"] {
			if number != 0 {
				t.Fatal("manual blur sent nonzero target")
			}
		}
	case <-time.After(time.Second):
		t.Fatal("manual blur did not send zero")
	}
	if a.command != (Vec3{}) || a.pad.Held {
		t.Fatal("manual joystick remained active after blur")
	}
}
