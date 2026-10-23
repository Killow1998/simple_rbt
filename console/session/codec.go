package session

import (
	"encoding/binary"
	"errors"
	"math"
)

type CameraFrame struct {
	Width, Height int
	RGB           []byte
	Depth         []float32
	Radial        bool
}
type Odometry struct {
	Position        [3]float64
	Quaternion      [4]float64
	Linear, Angular [3]float64
}
type JointCommand struct {
	Name     []string  `json:"name"`
	Position []float64 `json:"position"`
	Hold     bool      `json:"hold"`
}
type JointState struct {
	Name     []string  `json:"name"`
	Position []float64 `json:"position"`
	Velocity []float64 `json:"velocity"`
}

func DecodeCamera(packet Packet) (CameraFrame, error) {
	data := packet.Payload
	if len(data) < 25 {
		return CameraFrame{}, errors.New("camera header is truncated")
	}
	width, height := binary.LittleEndian.Uint32(data[:4]), binary.LittleEndian.Uint32(data[4:8])
	rgb, depth := binary.LittleEndian.Uint64(data[9:17]), binary.LittleEndian.Uint64(data[17:25])
	pixels := uint64(width) * uint64(height)
	if width == 0 || height == 0 || data[8] > 1 || pixels > (MaxPayload-25)/7 ||
		(rgb != 0 && rgb != pixels*3) || (depth != 0 && depth != pixels) ||
		(rgb == 0 && (packet.Kind == KindView || depth == 0)) || uint64(len(data)) != 25+rgb+depth*4 {
		return CameraFrame{}, errors.New("camera channel lengths are invalid")
	}
	result := CameraFrame{Width: int(width), Height: int(height), Radial: data[8] != 0, RGB: data[25 : 25+rgb]}
	result.Depth = make([]float32, depth)
	for i := range result.Depth {
		result.Depth[i] = math.Float32frombits(binary.LittleEndian.Uint32(data[25+rgb+uint64(i)*4:]))
	}
	return result, nil
}
func DecodeOdometry(packet Packet) (Odometry, error) {
	if len(packet.Payload) != 13*8 {
		return Odometry{}, errors.New("odometry payload is truncated")
	}
	result := Odometry{}
	offset := 0
	read := func(array []float64) {
		for i := range array {
			array[i] = math.Float64frombits(binary.LittleEndian.Uint64(packet.Payload[offset:]))
			offset += 8
		}
	}
	read(result.Position[:])
	read(result.Quaternion[:])
	read(result.Linear[:])
	read(result.Angular[:])
	return result, nil
}
