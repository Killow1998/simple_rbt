package session

import (
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"io"
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
