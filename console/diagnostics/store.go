// Package diagnostics persists bounded structured events without per-event I/O.
package diagnostics

import (
	"bufio"
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"syscall"
)

var ErrStoreBusy = errors.New("log store busy; batch deferred")

const Limit int64 = 100000000
const segmentLimit int64 = 5000000

var owned = regexp.MustCompile(`^srbt-[0-9]+-[0-9]+\.jsonl$`)

func files(directory string) ([]string, int64, error) {
	entries, err := os.ReadDir(directory)
	if os.IsNotExist(err) {
		return nil, 0, nil
	}
	if err != nil {
		return nil, 0, err
	}
	var names []string
	var total int64
	for _, entry := range entries {
		if !owned.MatchString(entry.Name()) || !entry.Type().IsRegular() {
			continue
		}
		info, e := entry.Info()
		if e != nil {
			return nil, 0, e
		}
		total += info.Size()
		names = append(names, entry.Name())
	}
	sort.Strings(names)
	return names, total, nil
}
func locked(directory string, action func() error) error {
	if err := os.MkdirAll(directory, 0700); err != nil {
		return err
	}
	fd, err := syscall.Open(filepath.Join(directory, ".lock"), syscall.O_CREAT|syscall.O_RDWR|syscall.O_CLOEXEC|syscall.O_NOFOLLOW, 0600)
	if err != nil {
		return err
	}
	defer syscall.Close(fd)
	if err = syscall.Flock(fd, syscall.LOCK_EX|syscall.LOCK_NB); err != nil {
		if errors.Is(err, syscall.EWOULDBLOCK) {
			return ErrStoreBusy
		}
		return err
	}
	defer syscall.Flock(fd, syscall.LOCK_UN)
	return action()
}
func appendBatch(directory string, batch []byte, limit int64) error {
	if len(batch) == 0 {
		return nil
	}
	if int64(len(batch)) > limit {
		return fmt.Errorf("log batch exceeds quota")
	}
	return locked(directory, func() error {
		names, total, err := files(directory)
		if err != nil {
			return err
		}
		for len(names) > 0 && total+int64(len(batch)) > limit {
			path := filepath.Join(directory, names[0])
			info, e := os.Stat(path)
			if e != nil {
				return e
			}
			if e = os.Remove(path); e != nil {
				return e
			}
			total -= info.Size()
			names = names[1:]
		}
		target := ""
		if len(names) > 0 {
			path := filepath.Join(directory, names[len(names)-1])
			info, e := os.Stat(path)
			if e != nil {
				return e
			}
			if info.Size()+int64(len(batch)) <= segmentLimit {
				target = path
			}
		}
		if target == "" {
			target = filepath.Join(directory, fmt.Sprintf("srbt-%d-%d.jsonl", wallNS(), os.Getpid()))
		}
		fd, err := syscall.Open(target, syscall.O_CREAT|syscall.O_RDWR|syscall.O_APPEND|syscall.O_CLOEXEC|syscall.O_NOFOLLOW, 0600)
		if err != nil {
			return err
		}
		f := os.NewFile(uintptr(fd), target)
		defer f.Close()
		original, err := f.Seek(0, io.SeekEnd)
		if err != nil {
			return err
		}
		if original > 0 {
			size := original
			if size > 16384 {
				size = 16384
			}
			tail := make([]byte, int(size))
			if _, err = f.ReadAt(tail, original-size); err != nil {
				return err
			}
			if tail[len(tail)-1] != '\n' {
				original = original - size + int64(bytes.LastIndexByte(tail, '\n')+1)
				if err = f.Truncate(original); err != nil {
					return err
				}
			}
		}
		_, err = f.Write(batch)
		if err != nil {
			f.Truncate(original)
		}
		return err
	})
}

// ReadRecent bounds disk reads and visible rows independently of the store quota.
func ReadRecent(directory, filter string) ([]Record, int64, error) {
	names, total, err := files(directory)
	if err != nil {
		return nil, total, err
	}
	var result []Record
	budget := int64(4 * 1024 * 1024)
	for i := len(names) - 1; i >= 0 && budget > 0 && len(result) < 2000; i-- {
		f, e := os.Open(filepath.Join(directory, names[i]))
		if e != nil {
			continue
		}
		info, e := f.Stat()
		if e != nil {
			f.Close()
			continue
		}
		start := info.Size() - budget
		if start < 0 {
			start = 0
		}
		f.Seek(start, 0)
		scanner := bufio.NewScanner(f)
		scanner.Buffer(make([]byte, 4096), 32768)
		if start > 0 {
			scanner.Scan()
		}
		var part []Record
		for scanner.Scan() {
			var r Record
			if json.Unmarshal(scanner.Bytes(), &r) == nil && r.Schema == 1 {
				part = append(part, r)
			}
		}
		f.Close()
		budget -= info.Size() - start
		for j := len(part) - 1; j >= 0 && len(result) < 2000; j-- {
			result = append(result, part[j])
		}
	}
	sort.SliceStable(result, func(i, j int) bool { return result[i].WallNS > result[j].WallNS })
	return filterRecords(result, filter), total, nil
}
