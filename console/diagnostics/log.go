package diagnostics

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"strings"
	"sync"
	"time"
)

type Record struct {
	Schema    int    `json:"schema"`
	Session   string `json:"session"`
	Writer    string `json:"writer"`
	Seq       uint64 `json:"seq"`
	Component string `json:"component"`
	Event     string `json:"event"`
	Level     string `json:"level"`
	WallNS    int64  `json:"wall_ns"`
	MonoNS    int64  `json:"mono_ns"`
	SimNS     uint64 `json:"sim_ns"`
	Fields    any    `json:"fields"`
}
type Health struct {
	Queued                                         int
	Dropped, WriteBatches, WriteFailures, Deferred uint64
	Error                                          string
}
type Log struct {
	modules         []string
	reported        uint64
	Directory       string
	mu              sync.Mutex
	session, writer string
	seq             uint64
	queue           []byte
	health          Health
	stop, done      chan struct{}
	closed          bool
	trace           bool
}

func wallNS() int64 { return time.Now().UnixNano() }
func New(directory string) *Log {
	l := &Log{Directory: directory, writer: fmt.Sprintf("gui-%d-%d", os.Getpid(), wallNS()), stop: make(chan struct{}), done: make(chan struct{}), trace: os.Getenv("SIMPLE_RBT_TRACE") == "1"}
	l.modules = []string{"sensor", "ipc", "ros", "control"}
	if names, ok := os.LookupEnv("SIMPLE_RBT_TRACE_MODULES"); ok {
		l.modules = strings.Split(names, ",")
	}
	l.session = l.writer
	go l.run()
	l.Event("process.started", map[string]any{"pid": os.Getpid()})
	return l
}
func (l *Log) Modules() []string {
	l.mu.Lock()
	defer l.mu.Unlock()
	return append([]string(nil), l.modules...)
}
func (l *Log) SetModules(names []string) {
	l.mu.Lock()
	l.modules = append([]string(nil), names...)
	l.mu.Unlock()
	l.Event("logging.modules", map[string]any{"modules": names})
}
func (l *Log) SetSession(id string) { l.mu.Lock(); l.session = id; l.mu.Unlock() }
func (l *Log) Session() string      { l.mu.Lock(); defer l.mu.Unlock(); return l.session }
func (l *Log) TraceEnabled() bool   { l.mu.Lock(); defer l.mu.Unlock(); return l.trace }
func (l *Log) SetTrace(on bool) {
	l.mu.Lock()
	l.trace = on
	l.mu.Unlock()
	l.Event("logging.trace", map[string]any{"enabled": on})
}
func (l *Log) Event(name string, fields any) { l.Emit(name, "info", fields) }
func (l *Log) Emit(name, level string, fields any) {
	if l == nil {
		return
	}
	l.mu.Lock()
	defer l.mu.Unlock()
	if l.closed || (level == "trace" && (!l.trace || !strings.Contains(","+strings.Join(l.modules, ",")+",", ",control,"))) {
		return
	}
	l.seq++
	// Linux CLOCK_MONOTONIC shares the C++ clock domain on the same host.
	var ts syscallTimespec
	mono := monotonicNS(&ts)
	l.enqueue(Record{1, l.session, l.writer, l.seq, "gui", name, level, wallNS(), mono, 0, fields})
}
func (l *Log) enqueue(record Record) {
	data, err := json.Marshal(record)
	if err != nil || len(data) > 16384 || len(l.queue)+len(data)+1 > 2*1024*1024 {
		l.health.Dropped++
		return
	}
	l.queue = append(l.queue, data...)
	l.queue = append(l.queue, '\n')
}

// Import accepts structured ROS records from the owned adapter stdout only.
func (l *Log) Import(line, component string) bool {
	var r Record
	if json.Unmarshal([]byte(line), &r) != nil || r.Schema != 1 || r.Component != component || r.Event == "" {
		return false
	}
	l.mu.Lock()
	defer l.mu.Unlock()
	if l.closed {
		return true
	}
	// Do not rewrite producer timestamps/sequence: these preserve causality.
	l.enqueue(r)
	return true
}
func (l *Log) Health() Health {
	l.mu.Lock()
	defer l.mu.Unlock()
	h := l.health
	h.Queued = len(l.queue)
	return h
}
func (l *Log) flush(final bool) {
	l.mu.Lock()
	if l.health.Dropped > l.reported {
		l.seq++
		data, _ := json.Marshal(Record{Schema: 1, Session: l.session, Writer: l.writer, Seq: l.seq, Component: "gui", Event: "logging.dropped", Level: "warning", WallNS: wallNS(), Fields: map[string]any{"count": l.health.Dropped - l.reported}})
		l.queue = append(l.queue, append(data, '\n')...)
		l.reported = l.health.Dropped
	}
	batch := l.queue
	l.queue = nil
	l.mu.Unlock()
	if len(batch) == 0 {
		return
	}
	err := appendBatch(l.Directory, batch, Limit)
	deadline := time.Now().Add(500 * time.Millisecond)
	for final && errors.Is(err, ErrStoreBusy) && time.Now().Before(deadline) {
		time.Sleep(20 * time.Millisecond)
		err = appendBatch(l.Directory, batch, Limit)
	}
	l.mu.Lock()
	if err == nil {
		l.health.WriteBatches++
		l.health.Error = ""
	} else {
		busy := errors.Is(err, ErrStoreBusy)
		if busy {
			l.health.Deferred++
		} else {
			l.health.WriteFailures++
			l.health.Error = err.Error()
		}
		if !final && len(l.queue)+len(batch) <= 2*1024*1024 {
			l.queue = append(batch, l.queue...)
		} else {
			l.health.Dropped += uint64(bytes.Count(batch, []byte{'\n'}))
		}
		if final || (!busy && (l.health.WriteFailures == 1 || l.health.WriteFailures%30 == 0)) {
			fmt.Fprintln(os.Stderr, "diagnostics:", err)
		}
	}
	l.mu.Unlock()
}
func (l *Log) run() {
	defer close(l.done)
	tick := time.NewTicker(time.Second)
	defer tick.Stop()
	for {
		select {
		case <-tick.C:
			l.flush(false)
		case <-l.stop:
			l.flush(true)
			return
		}
	}
}
func (l *Log) Close() {
	if l == nil {
		return
	}
	l.Event("process.stopped", nil)
	l.mu.Lock()
	if !l.closed {
		l.closed = true
		close(l.stop)
	}
	l.mu.Unlock()
	<-l.done
}
func filterRecords(records []Record, filter string) []Record {
	if filter == "" {
		return records
	}
	result := []Record{}
	filter = strings.ToLower(filter)
	for _, r := range records {
		data, _ := json.Marshal(r)
		if strings.Contains(strings.ToLower(string(data)), filter) {
			result = append(result, r)
		}
	}
	return result
}
