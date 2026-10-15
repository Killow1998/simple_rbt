package session

import (
	"bytes"
	"encoding/binary"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
	"time"
)

type fragmented struct{ data []byte }

func (f *fragmented) Read(dst []byte) (int, error) {
	if len(f.data) == 0 {
		return 0, os.ErrClosed
	}
	dst[0] = f.data[0]
	f.data = f.data[1:]
	return 1, nil
}
func TestFragmentedVersionedFrame(t *testing.T) {
	raw := make([]byte, 27)
	copy(raw, "SRBT")
	binary.LittleEndian.PutUint16(raw[4:], 1)
	binary.LittleEndian.PutUint16(raw[6:], uint16(KindMetadata))
	binary.LittleEndian.PutUint64(raw[8:], 9876543210)
	binary.LittleEndian.PutUint32(raw[16:], 3)
	copy(raw[24:], "{} ")
	packet, err := readPacket(&fragmented{data: raw})
	if err != nil || packet.Kind != KindMetadata || packet.StampNS != 9876543210 || !bytes.Equal(packet.Payload, []byte("{} ")) {
		t.Fatalf("complete fragmented frame: %+v %v", packet, err)
	}
	raw[4] = 2
	if _, err = readPacket(bytes.NewReader(raw)); err == nil {
		t.Fatal("unsupported protocol was accepted")
	}
}
func TestCPPFixture(t *testing.T) {
	fixture := os.Getenv("SIMPLE_RBT_CPP_IPC_FIXTURE")
	if fixture == "" {
		t.Skip("requires the independent C++ IPC fixture")
	}
	directory := t.TempDir()
	session := filepath.Join(directory, "ipc")
	result := filepath.Join(directory, "report.json")
	command := exec.Command(fixture, session, "gui", result)
	if err := command.Start(); err != nil {
		t.Fatal(err)
	}
	defer func() { command.Process.Kill(); command.Wait() }()
	client := NewClient(nil)
	defer client.Close()
	deadline := time.Now().Add(3 * time.Second)
	for {
		if err := client.Connect(session); err == nil {
			break
		}
		if time.Now().After(deadline) {
			t.Fatal("C++ fixture connection timeout")
		}
		time.Sleep(20 * time.Millisecond)
	}
	gotCamera, gotOdom := false, false
	deadline = time.Now().Add(2 * time.Second)
	for !(gotCamera && gotOdom) && time.Now().Before(deadline) {
		for _, packet := range client.Poll() {
			switch packet.Kind {
			case KindCamera:
				frame, err := DecodeCamera(packet)
				if err != nil {
					t.Fatal(err)
				}
				if frame.Width != 32 || frame.Height != 24 || len(frame.RGB) != 32*24*3 || len(frame.Depth) != 32*24 {
					t.Fatal("C++ camera dimensions")
				}
				for _, value := range frame.RGB {
					if value != 91 {
						t.Fatal("C++ image payload changed")
					}
				}
				for _, value := range frame.Depth {
					if value != 2.5 {
						t.Fatal("C++ float32 depth payload changed")
					}
				}
				gotCamera = true
			case KindOdometry:
				value, err := DecodeOdometry(packet)
				if err != nil || value.Position != [3]float64{.5, .6, .7} || value.Quaternion != [4]float64{1, 0, 0, 0} || value.Linear[0] != .4 {
					t.Fatalf("C++ odometry fields: %+v %v", value, err)
				}
				gotOdom = true
			}
		}
		time.Sleep(20 * time.Millisecond)
	}
	if !gotCamera || !gotOdom {
		t.Fatal("cross-language sensor delivery")
	}
	if err := client.SendCommand([3]float64{.3, 0, 0}); err != nil {
		t.Fatal(err)
	}
}
