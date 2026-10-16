// Package session connects a native console to the independent C++ core.
package session

import (
	"crypto/rand"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"path/filepath"
	"sync"
	"time"
)

type Kind uint16

const (
	KindHello Kind = iota + 1
	KindRequest
	KindCommand
	KindClock
	KindIMU
	KindLidar
	KindOdometry
	KindCamera
	KindPanorama
	KindStatus
	KindMetadata
	KindView
	KindError
	KindCameraIMU
)
const MaxPayload = 32 * 1024 * 1024
const ProtocolVersion = 1

type Packet struct {
	Kind    Kind
	StampNS uint64
	Payload []byte
}
type Client struct {
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
	return kind == KindView || kind == KindCamera || kind == KindPanorama || kind == KindOdometry || kind == KindMetadata || kind == KindStatus
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
func (c *Client) SendRequest(value map[string]any) error { return c.send(KindRequest, value) }
func (c *Client) SendCommand(value [3]float64) error {
	return c.send(KindCommand, map[string]any{"velocity": value})
}
func readPacket(reader io.Reader) (Packet, error) {
	var header [24]byte
	if _, err := io.ReadFull(reader, header[:]); err != nil {
		return Packet{}, err
	}
	kind := Kind(binary.LittleEndian.Uint16(header[6:8]))
	length := binary.LittleEndian.Uint32(header[16:20])
	if string(header[:4]) != "SRBT" || binary.LittleEndian.Uint16(header[4:6]) != ProtocolVersion || binary.LittleEndian.Uint32(header[20:24]) != 0 || kind < KindHello || kind > KindCameraIMU || length > MaxPayload {
		return Packet{}, errors.New("unsupported or oversized IPC frame")
	}
	packet := Packet{Kind: kind, StampNS: binary.LittleEndian.Uint64(header[8:16]), Payload: make([]byte, length)}
	_, err := io.ReadFull(reader, packet.Payload)
	return packet, err
}
func writePacket(writer io.Writer, packet Packet) error {
	if len(packet.Payload) > MaxPayload {
		return errors.New("IPC payload is too large")
	}
	frame := make([]byte, 24+len(packet.Payload))
	copy(frame, "SRBT")
	binary.LittleEndian.PutUint16(frame[4:6], ProtocolVersion)
	binary.LittleEndian.PutUint16(frame[6:8], uint16(packet.Kind))
	binary.LittleEndian.PutUint64(frame[8:16], packet.StampNS)
	binary.LittleEndian.PutUint32(frame[16:20], uint32(len(packet.Payload)))
	copy(frame[24:], packet.Payload)
	for len(frame) > 0 {
		written, err := writer.Write(frame)
		if err != nil {
			return err
		}
		if written == 0 {
			return io.ErrShortWrite
		}
		frame = frame[written:]
	}
	return nil
}
func DecodeJSON(packet Packet) (map[string]any, error) {
	var result map[string]any
	err := json.Unmarshal(packet.Payload, &result)
	if err != nil {
		return nil, fmt.Errorf("core JSON: %w", err)
	}
	return result, nil
}
