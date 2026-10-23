package session

import (
	"encoding/json"
	"math"
	"os"
	"os/exec"
	"testing"
	"time"
)

func number(v any) float64        { value, _ := v.(float64); return value }
func array(v any) []any           { value, _ := v.([]any); return value }
func object(v any) map[string]any { value, _ := v.(map[string]any); return value }
func waitMetadata(t *testing.T, client *Client, condition func(map[string]any) bool) map[string]any {
	t.Helper()
	deadline := time.Now().Add(8 * time.Second)
	for time.Now().Before(deadline) {
		for _, packet := range client.Poll() {
			if packet.Kind == KindError {
				t.Fatalf("core rejection: %s", packet.Payload)
			}
			if packet.Kind == KindMetadata {
				value, err := DecodeJSON(packet)
				if err != nil {
					t.Fatal(err)
				}
				if condition(value) {
					return value
				}
			}
		}
		time.Sleep(20 * time.Millisecond)
	}
	t.Fatal("core metadata condition timed out")
	return nil
}
func TestCPPWorkerLifecycle(t *testing.T) {
	worker := os.Getenv("SIMPLE_RBT_CPP_WORKER")
	if worker == "" {
		t.Skip("requires the host standalone C++ core")
	}
	options := DefaultOptions()
	options.WorkerPath = worker
	if root := os.Getenv("SIMPLE_RBT_CPP_DATA_ROOT"); root != "" {
		options.DataRoot = root
	}
	options.StateDir = t.TempDir()
	manager := NewManager(options, nil)
	defer manager.Events.Close()
	defer func() {
		if t.Failed() {
			t.Logf("core process output: %v", manager.Logs())
		}
	}()
	defer manager.Close()
	client := manager.Client
	config := map[string]any{"robot": "ranger_mini", "terrain": "flat", "backend": "cpu", "settle_s": .5, "sensors": map[string]any{"mid360": false, "camera": true, "depth": true, "panorama": false}}
	if err := manager.Start(config); err != nil {
		t.Fatal(err)
	}
	waitMetadata(t, client, func(v map[string]any) bool { return v["session_state"] == "config" })
	mount := map[string]any{"op": "mount", "sensor": "mid360", "position": [3]float64{.2, .1, .3}, "quaternion": [4]float64{1, 0, 0, 0}}
	if err := client.SendRequest(mount); err != nil {
		t.Fatal(err)
	}
	metadata := waitMetadata(t, client, func(v map[string]any) bool {
		p := array(object(object(v["mounts"])["mid360"])["position"])
		return len(p) == 3 && math.Abs(number(p[0])-.2) < 1e-8
	})
	imu := array(object(metadata["mid360_internal_imu"])["position"])
	for i, expected := range []float64{.211, .12329, .25588} {
		if len(imu) != 3 || math.Abs(number(imu[i])-expected) > 1e-8 {
			t.Fatal("rigid MID360 IMU offset")
		}
	}
	if err := client.SendRequest(map[string]any{"op": "start"}); err != nil {
		t.Fatal(err)
	}
	waitMetadata(t, client, func(v map[string]any) bool { return v["session_state"] == "running" })
	begin := time.Now()
	seenCamera := false
	// Running metadata precedes EGL/shader initialization. This lifecycle test
	// waits for its first real frame; sensor throughput is checked by E2E.
	for time.Since(begin) < 2*time.Second || (!seenCamera && time.Since(begin) < 10*time.Second) {
		client.SendCommand([3]float64{.3, 0, 0})
		for _, packet := range client.Poll() {
			if packet.Kind == KindCamera {
				frame, err := DecodeCamera(packet)
				if err != nil || len(frame.RGB) == 0 || len(frame.Depth) == 0 {
					t.Fatal("real camera channels")
				}
				seenCamera = true
			}
		}
		time.Sleep(100 * time.Millisecond)
	}
	if !seenCamera {
		t.Fatal("real C++ camera was not delivered")
	}
	client.SendCommand([3]float64{})
	waitMetadata(t, client, func(v map[string]any) bool { p := array(v["applied_command"]); return len(p) == 3 && number(p[0]) == 0 })
	client.SendRequest(map[string]any{"op": "source", "source": "none"})
	waitMetadata(t, client, func(v map[string]any) bool { return v["source"] == "none" })
	client.SendCommand([3]float64{.5, 0, 0})
	waitMetadata(t, client, func(v map[string]any) bool {
		p := array(v["applied_command"])
		return v["source"] == "none" && len(p) == 3 && number(p[0]) == 0
	})
	client.SendRequest(map[string]any{"op": "source", "source": "gui"})
	waitMetadata(t, client, func(v map[string]any) bool {
		p := array(v["applied_command"])
		return v["source"] == "gui" && len(p) == 3 && number(p[0]) == 0
	})
	firstDirectory := manager.SessionDir
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
	if client.Connected() {
		t.Fatal("worker shutdown retained a connection")
	}
	if _, err := os.Stat(firstDirectory); !os.IsNotExist(err) {
		t.Fatal("owned session directory was not removed")
	}
	config["robot"] = "scout_mini"
	if err := manager.Start(config); err != nil {
		t.Fatal(err)
	}
	if manager.Client != client {
		t.Fatal("restart changed the public client pointer")
	}
	metadata = waitMetadata(t, client, func(v map[string]any) bool { return v["robot"] == "scout_mini" && v["session_state"] == "config" })
	encoded, _ := json.Marshal(metadata)
	t.Logf("actual C++ worker verified: mount, camera, ownership, stop, shutdown, reconnect (%d metadata bytes)", len(encoded))
}

func TestROSOwnerSwitch(t *testing.T) {
	worker := os.Getenv("SIMPLE_RBT_CPP_WORKER")
	if worker == "" || os.Getenv("SIMPLE_RBT_TEST_ROS1") != "1" {
		t.Skip("requires the host core and optional official Noetic adapter")
	}
	t.Setenv("ROS_MASTER_URI", "http://127.0.0.1:11389")
	t.Setenv("ROS_IP", "127.0.0.1")
	t.Setenv("SIMPLE_RBT_START_MASTER", "1")
	options := DefaultOptions()
	options.WorkerPath = worker
	if root := os.Getenv("SIMPLE_RBT_CPP_DATA_ROOT"); root != "" {
		options.DataRoot = root
	}
	options.StateDir = t.TempDir()
	manager := NewManager(options, nil)
	defer manager.Events.Close()
	defer func() {
		if t.Failed() {
			t.Logf("core process output: %v", manager.Logs())
		}
	}()
	defer manager.Close()
	if err := manager.Start(map[string]any{"robot": "ranger_mini", "backend": "cpu", "sensors": map[string]any{"mid360": false, "camera": false, "depth": false, "panorama": false}}); err != nil {
		t.Fatal(err)
	}
	client := manager.Client
	waitMetadata(t, client, func(v map[string]any) bool { return v["session_state"] == "config" })
	if err := manager.StartAdapter("ros1"); err != nil {
		t.Fatal(err)
	}
	waitMetadata(t, client, func(v map[string]any) bool { return number(object(v["connections"])["ros1"]) > 0 })
	client.SendRequest(map[string]any{"op": "source", "source": "ros1"})
	client.SendRequest(map[string]any{"op": "start"})
	waitMetadata(t, client, func(v map[string]any) bool { return v["source"] == "ros1" && v["session_state"] == "running" })
	manager.mu.Lock()
	container := manager.containers["ros1"]
	manager.mu.Unlock()
	script := `source /opt/ros/noetic/setup.bash; exec python3 -c 'import rospy,time;from geometry_msgs.msg import Twist;rospy.init_node("owner_switch_oracle",anonymous=True);p=rospy.Publisher("/ranger_mini/cmd_vel",Twist,queue_size=1);time.sleep(.5);m=Twist();m.linear.x=.3;end=time.monotonic()+2
while time.monotonic()<end:p.publish(m);time.sleep(.05)'`
	publisher := exec.Command("docker", "exec", container, "bash", "-c", script)
	if err := publisher.Start(); err != nil {
		t.Fatal(err)
	}
	defer publisher.Wait()
	waitMetadata(t, client, func(v map[string]any) bool {
		a := array(v["applied_command"])
		return len(a) == 3 && math.Abs(number(a[0])-.3) < 1e-8
	})
	client.SendRequest(map[string]any{"op": "stop"})
	waitMetadata(t, client, func(v map[string]any) bool {
		a := array(v["applied_command"])
		return v["source"] == "gui" && len(a) == 3 && number(a[0]) == 0
	})
	time.Sleep(350 * time.Millisecond)
	waitMetadata(t, client, func(v map[string]any) bool {
		a := array(v["applied_command"])
		return v["source"] == "gui" && len(a) == 3 && number(a[0]) == 0
	})
	t.Log("Stop clears the ROS prior owner while its publisher continues sending")
}
