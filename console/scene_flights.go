package main

import "math"

// Build flights in connection-local coordinates, then apply the authored yaw
// and lower-floor anchor once. The landing contributes length but no rise.
func (s *Scene) flightBoxes(o SceneObject) []SceneObject {
	c := o.Connection
	rise, run := s.connectionRise(o), o.Size[0]
	if rise <= 0 || run <= 0 || (o.Kind == "stairs" && (c.Steps < 1 || c.Steps > 128)) {
		return nil
	}
	boxes := []SceneObject{}
	emit := func(p, size Vec3, q Quat) {
		b := o
		b.Connection = nil
		b.Position = add(o.Position, rotate(p, o.Quaternion))
		b.Quaternion = qnormalize(qproduct(o.Quaternion, q))
		b.Size = size
		boxes = append(boxes, b)
	}
	flight := func(start Vec3, yaw, length, height float64, steps int) {
		heading := fromEuler(Vec3{0, 0, yaw})
		part := func(p, size Vec3, q Quat) { emit(add(start, rotate(p, heading)), size, qproduct(heading, q)) }
		if o.Kind == "stairs" {
			if steps < 1 {
				return
			}
			tread, h := length/float64(steps), height/float64(steps)
			for i := 1; i <= steps; i++ {
				z := float64(i) * h
				part(Vec3{(float64(i) - .5) * tread, 0, z / 2}, Vec3{tread, o.Size[1], z}, Quat{1, 0, 0, 0})
			}
		} else {
			angle := math.Atan2(height, length)
			thickness := o.Size[2]
			part(Vec3{length / 2, 0, height/2 - thickness/(2*math.Cos(angle))}, Vec3{math.Hypot(length, height) + thickness*math.Tan(angle), o.Size[1], thickness}, qaxis(Vec3{0, 1, 0}, -angle*180/math.Pi))
		}
	}
	if c.Turn == 0 {
		flight(Vec3{}, 0, run, rise, c.Steps)
		return boxes
	}
	portion := .5
	first := c.Steps / 2
	if o.Kind == "stairs" {
		portion = float64(first) / float64(c.Steps)
	}
	length, height := run*portion, rise*portion
	flight(Vec3{}, 0, length, height, first)
	width, depth := o.Size[1], math.Max(c.Landing, o.Size[1])
	centre := Vec3{length + depth/2, 0, height - o.Size[2]/2}
	dimensions := Vec3{depth, width, o.Size[2]}
	start := Vec3{length + depth/2, width / 2, height}
	if c.Turn == -90 {
		start[1] = -width / 2
	}
	if c.Turn == 180 {
		centre[1] = width / 2
		dimensions[1] = 2 * width
		start = Vec3{length, width, height}
	}
	emit(centre, dimensions, Quat{1, 0, 0, 0})
	flight(start, float64(c.Turn), run-length, rise-height, c.Steps-first)
	return boxes
}
func (s *Scene) connectionEnd(o SceneObject) Vec3 {
	rise, run := s.connectionRise(o), o.Size[0]
	c := o.Connection
	end := Vec3{run, 0, rise}
	if c.Turn != 0 {
		fraction := .5
		if o.Kind == "stairs" && c.Steps > 0 {
			fraction = float64(c.Steps/2) / float64(c.Steps)
		}
		first := run * fraction
		depth := math.Max(c.Landing, o.Size[1])
		switch c.Turn {
		case 90:
			end = Vec3{first + depth/2, o.Size[1]/2 + run - first, rise}
		case -90:
			end = Vec3{first + depth/2, -o.Size[1]/2 - run + first, rise}
		case 180:
			end = Vec3{2*first - run, o.Size[1], rise}
		}
	}
	return add(s.pose(o).Position, rotate(end, o.Quaternion))
}
