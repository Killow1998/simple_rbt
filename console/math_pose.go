package main

import "math"

type Vec3 [3]float64
type Quat [4]float64 // scalar first, matching the core's wxyz protocol.
type Pose struct {
	Position   Vec3 `json:"position"`
	Quaternion Quat `json:"quaternion"`
}

func add(a, b Vec3) Vec3         { return Vec3{a[0] + b[0], a[1] + b[1], a[2] + b[2]} }
func sub(a, b Vec3) Vec3         { return Vec3{a[0] - b[0], a[1] - b[1], a[2] - b[2]} }
func mul(a Vec3, s float64) Vec3 { return Vec3{a[0] * s, a[1] * s, a[2] * s} }
func dot(a, b Vec3) float64      { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }
func cross(a, b Vec3) Vec3 {
	return Vec3{a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]}
}
func norm(a Vec3) float64 { return math.Sqrt(dot(a, a)) }
func normalized(a Vec3) Vec3 {
	if n := norm(a); n > 1e-12 {
		return mul(a, 1/n)
	}
	return Vec3{}
}
func clamp(v, lo, hi float64) float64 { return math.Max(lo, math.Min(hi, v)) }
func axis(i int) Vec3                 { v := Vec3{}; v[i] = 1; return v }
func qnormalize(q Quat) Quat {
	n := math.Sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
	if n < 1e-12 {
		return Quat{1, 0, 0, 0}
	}
	for i := range q {
		q[i] /= n
	}
	return q
}
func qproduct(a, b Quat) Quat {
	return Quat{a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3], a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2], a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1], a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]}
}
func rotate(v Vec3, q Quat) Vec3 {
	q = qnormalize(q)
	vector := Vec3{q[1], q[2], q[3]}
	t := mul(cross(vector, v), 2)
	return add(v, add(mul(t, q[0]), cross(vector, t)))
}
func qaxis(v Vec3, degrees float64) Quat {
	v = normalized(v)
	a := degrees * math.Pi / 360
	s := math.Sin(a)
	return Quat{math.Cos(a), v[0] * s, v[1] * s, v[2] * s}
}
func fromEuler(e Vec3) Quat {
	return qnormalize(qproduct(qproduct(qaxis(axis(2), e[2]), qaxis(axis(1), e[1])), qaxis(axis(0), e[0])))
}
func toEuler(q Quat) Vec3 {
	q = qnormalize(q)
	w, x, y, z := q[0], q[1], q[2], q[3]
	d := 180 / math.Pi
	return Vec3{math.Atan2(2*(w*x+y*z), 1-2*(x*x+y*y)) * d, math.Asin(clamp(2*(w*y-z*x), -1, 1)) * d, math.Atan2(2*(w*z+x*y), 1-2*(y*y+z*z)) * d}
}
func internalImu(p Pose, offset Vec3) Vec3 {
	return add(p.Position, rotate(offset, p.Quaternion))
}
func samePose(a, b Pose) bool {
	d := 0.
	for i := range a.Quaternion {
		d += a.Quaternion[i] * b.Quaternion[i]
	}
	return norm(sub(a.Position, b.Position)) < 1e-5 && math.Abs(d) > 1-1e-6
}
