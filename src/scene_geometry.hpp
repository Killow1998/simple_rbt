#pragma once
#include "simple_rbt/core.hpp"
#include "simple_rbt/scene_geometry.hpp"
#include <tinyxml2.h>
namespace simple_rbt::structured {
inline constexpr std::size_t max_parts = 4096;
void require(bool, const std::string &);
void fields(const Json &, std::initializer_list<const char *> required,
            std::initializer_list<const char *> optional = {});
double dimension(const Json &, const char *, bool zero = false);
void vector(const Json &, std::size_t);
Json primitive(const Json &);
Json planar_parts(const Json &);
Json polyhedron(const std::vector<Vec3> &,
                const std::vector<std::vector<int>> &);
Json preview(Json part);
Json spawn_document(const Json &, const std::string &);
void append(tinyxml2::XMLDocument &, const Json &);
} // namespace simple_rbt::structured
