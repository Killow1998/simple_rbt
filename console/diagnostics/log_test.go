package diagnostics

import (
	"archive/zip"
	"encoding/json"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

func TestRetentionAndExport(t *testing.T) {
	dir := t.TempDir()
	old := filepath.Join(dir, "srbt-1000000000000000000-1.jsonl")
	f, err := os.Create(old)
	if err != nil {
		t.Fatal(err)
	}
	f.Truncate(Limit - 10)
	f.Close()
	unrelated := filepath.Join(dir, "user-notes.txt")
	os.WriteFile(unrelated, []byte("keep"), 0600)
	batch := []byte(`{"schema":1,"event":"new","component":"core"}` + "\n")
	if err = appendBatch(dir, batch, Limit); err != nil {
		t.Fatal(err)
	}
	if _, err = os.Stat(old); !os.IsNotExist(err) {
		t.Fatal("old segment not evicted")
	}
	if data, _ := os.ReadFile(unrelated); string(data) != "keep" {
		t.Fatal("unrelated file changed")
	}
	_, total, _ := files(dir)
	if total > Limit {
		t.Fatal("100 MB quota exceeded")
	}
	destination := filepath.Join(t.TempDir(), "diagnostics.zip")
	if err = Export(dir, destination, map[string]any{"test": true}); err != nil {
		t.Fatal(err)
	}
	z, err := zip.OpenReader(destination)
	if err != nil {
		t.Fatal(err)
	}
	defer z.Close()
	if len(z.File) != 2 {
		t.Fatal("export must contain manifest and managed segment only")
	}
}
func TestBatchingOverflowAndFailure(t *testing.T) {
	dir := t.TempDir()
	l := New(dir)
	for i := 0; i < 2000; i++ {
		l.Event("button", map[string]any{"n": i})
	}
	if h := l.Health(); h.WriteBatches != 0 {
		t.Fatal("events caused immediate disk writes", h)
	}
	l.Close()
	rows, _, err := ReadRecent(dir, "button")
	if err != nil || len(rows) < 1990 {
		t.Fatal("batch not persisted", len(rows), err)
	}
	blocked := filepath.Join(t.TempDir(), "file")
	os.WriteFile(blocked, []byte("owned fixture"), 0600)
	broken := New(blocked)
	broken.Event("failure-test", nil)
	broken.Close()
	if broken.Health().WriteFailures == 0 || broken.Health().Error == "" {
		t.Fatal("write failure hidden")
	}
	flood := New(t.TempDir())
	payload := strings.Repeat("x", 12000)
	for i := 0; i < 500; i++ {
		flood.Event("flood", payload)
	}
	if flood.Health().Dropped == 0 || flood.Health().Queued > 2*1024*1024 {
		t.Fatal("unbounded event queue")
	}
	flood.Close()
	rows, _, err = ReadRecent(flood.Directory, "logging.dropped")
	if err != nil || len(rows) == 0 {
		t.Fatal("overflow marker missing", err)
	}
}
func TestROSImportAndTrace(t *testing.T) {
	l := New(t.TempDir())
	l.SetSession("shared")
	l.Emit("disabled", "trace", nil)
	l.SetTrace(true)
	l.Emit("enabled", "trace", nil)
	row := Record{Schema: 1, Component: "ros1", Session: "shared", Event: "ros.publish_called", WallNS: time.Now().UnixNano(), Fields: map[string]any{"sample": "lidar:42"}}
	data, _ := json.Marshal(row)
	if !l.Import(string(data), "ros1") || l.Import(string(data), "ros2") {
		t.Fatal("producer identity lost")
	}
	l.Close()
	rows, _, _ := ReadRecent(l.Directory, "")
	seen := map[string]bool{}
	for _, r := range rows {
		seen[r.Event] = true
	}
	if seen["disabled"] || !seen["enabled"] || !seen["ros.publish_called"] {
		t.Fatal(seen)
	}
}

func TestBusyStoreDefersWithoutFalseFailure(t *testing.T) {
	l := New(t.TempDir())
	if err := locked(l.Directory, func() error { l.flush(false); return nil }); err != nil {
		t.Fatal(err)
	}
	h := l.Health()
	if h.Deferred != 1 || h.WriteFailures != 0 || h.Dropped != 0 || h.Error != "" {
		t.Fatal(h)
	}
	l.Close()
	rows, _, err := ReadRecent(l.Directory, "")
	if err != nil || len(rows) == 0 {
		t.Fatal("deferred batch not recovered", err)
	}
}

func TestSearchFindsEventsOutsideLiveTail(t *testing.T) {
	dir := t.TempDir()
	var batch strings.Builder
	for i := 0; i < 4000; i++ {
		name := "later"
		if i == 0 {
			name = "old-target"
		}
		data, _ := json.Marshal(Record{Schema: 1, Event: name, WallNS: int64(i)})
		batch.Write(data)
		batch.WriteByte('\n')
	}
	if err := appendBatch(dir, []byte(batch.String()), Limit); err != nil {
		t.Fatal(err)
	}
	recent, _, _ := ReadRecent(dir, "old-target")
	if len(recent) != 0 {
		t.Fatal("fixture not outside live tail")
	}
	found, _, err := Search(dir, "old-target")
	if err != nil || len(found) != 1 {
		t.Fatal("old retained event not searchable", err)
	}
}

func TestCloseWaitsForShortExportLock(t *testing.T) {
	l := New(t.TempDir())
	done := make(chan struct{})
	if err := locked(l.Directory, func() error { go func() { l.Close(); close(done) }(); time.Sleep(80 * time.Millisecond); return nil }); err != nil {
		t.Fatal(err)
	}
	<-done
	if l.Health().Dropped != 0 {
		t.Fatal("shutdown batch lost to brief export lock")
	}
	rows, _, _ := ReadRecent(l.Directory, "process.stopped")
	if len(rows) != 1 {
		t.Fatal("final record absent")
	}
}
