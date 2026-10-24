#pragma once
#include <nlohmann/json.hpp>
namespace simple_rbt {
// Validates v2, resolves hierarchy/templates, and produces bounded convex parts
// and display triangles. This is the editor and physics geometry authority.
nlohmann::json resolve_structured_scene(const nlohmann::json &scene);
} // namespace simple_rbt
