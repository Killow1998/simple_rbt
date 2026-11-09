package main

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"os/exec"
	"simple-rbt/console/session"
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

type sceneResolver struct {
	worker, key string
	result      resolvedScene
	err         error
}

func (r *sceneResolver) resolve(s *Scene) (resolvedScene, error) {
	raw, err := json.Marshal(s)
	if err != nil {
		return resolvedScene{}, err
	}
	if string(raw) == r.key {
		return r.result, r.err
	}
	worker := r.worker
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
	r.key, r.result, r.err = string(raw), result, err
	return result, err
}
