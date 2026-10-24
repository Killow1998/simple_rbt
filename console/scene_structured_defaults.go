package main

func (a *app) newStructuredScene() Scene {
	s := Scene{Version: 2, Format: "sprobo.scene", ID: "scene_" + newID(), Name: a.tr("New scene", "新场景"), Units: map[string]string{"length": "m", "angle": "rad"}, Coordinates: "right_handed_z_up", Objects: []SceneObject{}}
	s.Objects = append(s.Objects, SceneObject{ID: "floor", Name: a.tr("Floor", "地面"), Kind: "primitive", Parent: "world", Semantic: "floor", Pose: Pose{Position: Vec3{0, 0, -.1}, Quaternion: Quat{1, 0, 0, 0}}, Geometry: map[string]any{"type": "box", "length_m": 20., "width_m": 20., "height_m": .2}, Evidence: map[string]any{"basis": "manual"}})
	s.Objects = append(s.Objects, SceneObject{ID: "spawn", Name: a.tr("Robot spawn", "机器人出生点"), Kind: "group", Parent: "world", Semantic: "spawn", Pose: Pose{Quaternion: Quat{1, 0, 0, 0}}})
	return s
}
