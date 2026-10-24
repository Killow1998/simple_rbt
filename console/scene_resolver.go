package main

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"os/exec"
	"simple-rbt/console/session"
	"sync"
	"time"
)

type scenePart struct {
	ObjectID string `json:"object_id"`
	Semantic string `json:"semantic"`
	Pose
	Vertices []Vec3   `json:"vertices"`
	Faces    [][3]int `json:"faces"`
}
type resolvedScene struct {
	Parts []scenePart     `json:"parts"`
	Nodes map[string]Pose `json:"nodes"`
}

var sceneCompiler = struct {
	sync.Mutex
	worker, key string
	result      resolvedScene
	err         error
}{}

func setSceneWorker(worker string) {
	sceneCompiler.Lock()
	defer sceneCompiler.Unlock()
	sceneCompiler.worker = worker
	sceneCompiler.key = ""
}
func resolveScene(s *Scene) (resolvedScene, error) {
	sceneCompiler.Lock()
	defer sceneCompiler.Unlock()
	raw, err := json.Marshal(s)
	if err != nil {
		return resolvedScene{}, err
	}
	if string(raw) == sceneCompiler.key {
		return sceneCompiler.result, sceneCompiler.err
	}
	worker := sceneCompiler.worker
	if worker == "" {
		worker = session.DefaultOptions().WorkerPath
	}
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	cmd := exec.CommandContext(ctx, worker, "--resolve-scene")
	cmd.Stdin = bytes.NewReader(raw)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	output, err := cmd.Output()
	var result resolvedScene
	if err != nil {
		err = fmt.Errorf("Scene geometry / 场景几何: %s (%w)", stderr.String(), err)
	} else {
		err = json.Unmarshal(output, &result)
	}
	sceneCompiler.key, sceneCompiler.result, sceneCompiler.err = string(raw), result, err
	return result, err
}
