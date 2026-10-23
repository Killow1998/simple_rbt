// Package session connects a native console to the independent C++ core.
package session

import (
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"net"
	"path/filepath"
	"simple-rbt/console/diagnostics"
	"sync"
	"time"
)

type Client struct {
	Events        *diagnostics.Log
	requestSeq    uint64
	action        string
	lastCommand   [3]float64
	subscriptions []string
	mu            sync.Mutex
	writeMu       sync.Mutex
	control, data net.Conn
	generation    uint64
	pending       []Packet
	pendingBytes  int
	update        func()
}

func NewClient(onUpdate func()) *Client {
	return &Client{update: onUpdate, subscriptions: []string{"view", "camera", "odom", "metadata", "status"}}
}

// SetSubscriptions applies to the next connection (configuration restarts).
func (c *Client) SetSubscriptions(names []string) {
	c.mu.Lock()
	c.subscriptions = append([]string(nil), names...)
	c.mu.Unlock()
}
func (c *Client) subscriptionList() []string {
	c.mu.Lock()
	defer c.mu.Unlock()
	return append([]string(nil), c.subscriptions...)
}
func (c *Client) Connected() bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.control != nil && c.data != nil
}
func (c *Client) Connect(dir string) error {
	c.Close()
	control, err := net.DialTimeout("unix", filepath.Join(dir, "control.sock"), 200*time.Millisecond)
	if err != nil {
		return err
	}
	data, err := net.DialTimeout("unix", filepath.Join(dir, "data.sock"), 200*time.Millisecond)
	if err != nil {
		control.Close()
		return err
	}
	token := make([]byte, 16)
	if _, err = rand.Read(token); err != nil {
		control.Close()
		data.Close()
		return err
	}
	hello, _ := json.Marshal(map[string]any{"role": "gui", "token": hex.EncodeToString(token), "subscriptions": c.subscriptionList()})
	c.mu.Lock()
	c.control = control
	c.data = data
	c.generation++
	generation := c.generation
	c.mu.Unlock()
	if err = writePacket(control, Packet{Kind: KindHello, Payload: hello}); err != nil {
		c.Close()
		return err
	}
	// Wait for control authorization before pairing the data channel.
	control.SetReadDeadline(time.Now().Add(2 * time.Second))
	first, err := readPacket(control)
	control.SetReadDeadline(time.Time{})
	if err != nil || first.Kind != KindHello {
		c.Close()
		if err == nil {
			err = errors.New("core hello missing")
		}
		return err
	}
	if err = writePacket(data, Packet{Kind: KindHello, Payload: hello}); err != nil {
		c.Close()
		return err
	}
	if c.Events != nil {
		c.Events.Event("ipc.connected", nil)
	}
	c.stage(first, generation)
	go c.read(control, generation)
	go c.read(data, generation)
	return nil
}
func (c *Client) read(conn net.Conn, generation uint64) {
	for {
		packet, err := readPacket(conn)
		if err != nil {
			c.disconnect(generation)
			return
		}
		c.stage(packet, generation)
	}
}
func latest(kind Kind) bool {
	return kind == KindExternalCamera || kind == KindJointState || kind == KindView || kind == KindCamera || kind == KindPanorama || kind == KindOdometry || kind == KindMetadata || kind == KindStatus
}
func (c *Client) stage(packet Packet, generation uint64) {
	c.mu.Lock()
	if c.generation != generation {
		c.mu.Unlock()
		return
	}
	if latest(packet.Kind) {
		for i := 0; i < len(c.pending); {
			if c.pending[i].Kind == packet.Kind {
				c.pendingBytes -= len(c.pending[i].Payload)
				c.pending = append(c.pending[:i], c.pending[i+1:]...)
			} else {
				i++
			}
		}
	}
	for len(c.pending) > 0 && (len(c.pending) >= 128 || c.pendingBytes+len(packet.Payload) > 64*1024*1024) {
		c.pendingBytes -= len(c.pending[0].Payload)
		c.pending = c.pending[1:]
	}
	c.pendingBytes += len(packet.Payload)
	c.pending = append(c.pending, packet)
	callback := c.update
	c.mu.Unlock()
	if callback != nil {
		callback()
	}
}
func (c *Client) disconnect(generation uint64) {
	c.mu.Lock()
	if c.generation != generation {
		c.mu.Unlock()
		return
	}
	control, data := c.control, c.data
	c.control = nil
	c.data = nil
	c.generation++
	callback := c.update
	c.mu.Unlock()
	if control != nil {
		control.Close()
	}
	if data != nil {
		data.Close()
	}
	if c.Events != nil && (control != nil || data != nil) {
		c.Events.Event("ipc.disconnected", nil)
	}
	if callback != nil && (control != nil || data != nil) {
		callback()
	}
}
func (c *Client) Close() {
	c.mu.Lock()
	generation := c.generation
	c.mu.Unlock()
	c.disconnect(generation)
	c.mu.Lock()
	c.pending = nil
	c.pendingBytes = 0
	c.mu.Unlock()
}
func (c *Client) Poll() []Packet {
	c.mu.Lock()
	defer c.mu.Unlock()
	result := c.pending
	c.pending = nil
	c.pendingBytes = 0
	return result
}
func (c *Client) send(kind Kind, value any) error {
	payload, err := json.Marshal(value)
	if err != nil {
		return err
	}
	c.writeMu.Lock()
	defer c.writeMu.Unlock()
	c.mu.Lock()
	conn, generation := c.control, c.generation
	c.mu.Unlock()
	if conn == nil {
		return errors.New("core is disconnected")
	}
	conn.SetWriteDeadline(time.Now().Add(200 * time.Millisecond))
	err = writePacket(conn, Packet{Kind: kind, Payload: payload})
	conn.SetWriteDeadline(time.Time{})
	if err != nil {
		c.disconnect(generation)
	}
	return err
}
func (c *Client) SetAction(id string) { c.mu.Lock(); c.action = id; c.mu.Unlock() }
func (c *Client) SendRequest(value map[string]any) error {
	copy := map[string]any{}
	for k, v := range value {
		copy[k] = v
	}
	c.mu.Lock()
	c.requestSeq++
	id := fmt.Sprintf("gui-%d-%d", time.Now().UnixNano(), c.requestSeq)
	copy["request_id"] = id
	copy["action_id"] = c.action
	c.mu.Unlock()
	level := "info"
	if value["op"] == "view" || value["op"] == "mount" {
		level = "trace"
	}
	if c.Events != nil {
		c.Events.Emit("request.sent", level, copy)
	}
	err := c.send(KindRequest, copy)
	if err != nil && c.Events != nil {
		c.Events.Emit("request.send_failed", "error", map[string]any{"request_id": id, "message": err.Error()})
	}
	return err
}
func (c *Client) SendCommand(value [3]float64) error {
	c.mu.Lock()
	changed := value != c.lastCommand
	c.lastCommand = value
	c.mu.Unlock()
	if changed && c.Events != nil {
		c.Events.Event("command.sent", map[string]any{"velocity": value})
	}
	return c.send(KindCommand, map[string]any{"velocity": value})
}
func (c *Client) SendJoints(value JointCommand) error { return c.send(KindJointCommand, value) }
