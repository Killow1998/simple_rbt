package session

import (
	"io"
	"io/fs"
	"os"
	"path/filepath"
)

// Both checkout and installed builds send only the reproducible adapter closure,
// excluding experiment data, virtual environments, weights and Git metadata.
func adapterContext(source, version string) (string, error) {
	target, err := os.MkdirTemp("", "srbt-adapter-")
	if err != nil {
		return "", err
	}
	files := []string{"deployment/" + version, "include", "ipc", "diagnostics", "wrappers", "third_party/nlohmann", "interfaces/livox_ros_driver2", "src/robots.cpp"}
	for _, relative := range files {
		root := filepath.Join(source, relative)
		err = filepath.WalkDir(root, func(path string, entry fs.DirEntry, walkErr error) error {
			if walkErr != nil {
				return walkErr
			}
			rel, err := filepath.Rel(source, path)
			if err != nil {
				return err
			}
			destination := filepath.Join(target, rel)
			if entry.IsDir() {
				return os.MkdirAll(destination, 0755)
			}
			info, err := entry.Info()
			if err != nil {
				return err
			}
			if !info.Mode().IsRegular() {
				return nil
			}
			input, err := os.Open(path)
			if err != nil {
				return err
			}
			defer input.Close()
			if err = os.MkdirAll(filepath.Dir(destination), 0755); err != nil {
				return err
			}
			output, err := os.OpenFile(destination, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, info.Mode().Perm())
			if err != nil {
				return err
			}
			_, copyErr := io.Copy(output, input)
			closeErr := output.Close()
			if copyErr != nil {
				return copyErr
			}
			return closeErr
		})
		if err != nil {
			os.RemoveAll(target)
			return "", err
		}
	}
	return target, nil
}
