package diagnostics

import (
	"syscall"
	"unsafe"
)

type syscallTimespec = syscall.Timespec

func monotonicNS(ts *syscallTimespec) int64 {
	_, _, err := syscall.Syscall(syscall.SYS_CLOCK_GETTIME, 1, uintptr(unsafe.Pointer(ts)), 0)
	if err != 0 {
		return 0
	}
	return ts.Sec*1000000000 + ts.Nsec
}
