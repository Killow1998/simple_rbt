package diagnostics

import (
	"archive/zip"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path/filepath"
)

// Export is explicit and separate from automatic log retention. Locking gives a
// coherent snapshot; a busy producer retries next second without blocking physics.
func Export(directory, destination string, manifest any) error {
	out, err := os.OpenFile(destination, os.O_CREATE|os.O_EXCL|os.O_WRONLY, 0600)
	if err != nil {
		return err
	}
	ok := false
	defer func() {
		out.Close()
		if !ok {
			os.Remove(destination)
		}
	}()
	archive := zip.NewWriter(out)
	err = locked(directory, func() error {
		names, total, e := files(directory)
		if e != nil {
			return e
		}
		if total > Limit {
			return fmt.Errorf("log store exceeds quota; wait for rotation")
		}
		meta, e := archive.Create("manifest.json")
		if e != nil {
			return e
		}
		if e = json.NewEncoder(meta).Encode(manifest); e != nil {
			return e
		}
		for _, name := range names {
			input, e := os.Open(filepath.Join(directory, name))
			if e != nil {
				return e
			}
			entry, e := archive.Create(name)
			if e == nil {
				_, e = io.Copy(entry, input)
			}
			input.Close()
			if e != nil {
				return e
			}
		}
		return nil
	})
	closeErr := archive.Close()
	if err != nil {
		return err
	}
	if closeErr != nil {
		return closeErr
	}
	ok = true
	return out.Close()
}
