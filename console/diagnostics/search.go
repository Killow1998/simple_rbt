package diagnostics

import (
	"bufio"
	"encoding/json"
	"io"
	"os"
	"path/filepath"
	"sort"
	"strings"
)

// Search scans the retained store only on explicit request. Memory is bounded to
// the latest 500 matching events; unlike live tailing, old retained segments are
// included. Concurrent rotation may remove a file before it is opened.
func Search(directory, query string) ([]Record, int64, error) {
	names, total, err := files(directory)
	if err != nil {
		return nil, total, err
	}
	var matches []Record
	query = strings.ToLower(query)
	budget := Limit
	for i := len(names) - 1; i >= 0 && budget > 0; i-- {
		f, e := os.Open(filepath.Join(directory, names[i]))
		if os.IsNotExist(e) {
			continue
		}
		if e != nil {
			return nil, total, e
		}
		info, e := f.Stat()
		if e != nil {
			f.Close()
			return nil, total, e
		}
		size := info.Size()
		if size > budget {
			size = budget
		}
		budget -= size
		scanner := bufio.NewScanner(io.LimitReader(f, size))
		scanner.Buffer(make([]byte, 4096), 32768)
		for scanner.Scan() {
			if !strings.Contains(strings.ToLower(scanner.Text()), query) {
				continue
			}
			var row Record
			if json.Unmarshal(scanner.Bytes(), &row) != nil || row.Schema != 1 {
				continue
			}
			matches = append(matches, row)
			if len(matches) >= 1000 {
				sort.Slice(matches, func(i, j int) bool { return matches[i].WallNS > matches[j].WallNS })
				matches = matches[:500]
			}
		}
		e = scanner.Err()
		f.Close()
		if e != nil {
			return nil, total, e
		}
	}
	sort.Slice(matches, func(i, j int) bool { return matches[i].WallNS > matches[j].WallNS })
	if len(matches) > 500 {
		matches = matches[:500]
	}
	return matches, total, nil
}
