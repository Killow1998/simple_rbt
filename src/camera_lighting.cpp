#define GL_GLEXT_PROTOTYPES
#include "camera_lighting.hpp"
#include <GL/gl.h>
#include <GL/glext.h>
#include <mujoco/mjmacro.h>
namespace simple_rbt {
namespace {
GLuint compile(GLenum type, const char *source) {
  GLuint shader = glCreateShader(type);
  if (!shader)
    return 0;
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  GLint ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (ok)
    return shader;
  glDeleteShader(shader);
  return 0;
}
} // namespace
CameraLighting::CameraLighting() {
  // Same vertex lighting as the two native passes, quantized separately to the
  // 8-bit framebuffer before addition. Only the common opaque two-light case
  // uses this program; unsupported scenes and shader failures retain native
  // shading.
  const char *vertex = R"glsl(#version 120
varying vec3 baseColor, sunColor;
vec3 illuminate(int light, vec3 normal, vec3 view) {
  vec3 direction = normalize(gl_LightSource[light].position.xyz);
  float diffuse = max(dot(normal, direction), 0.0);
  float specular = diffuse > 0.0
      ? pow(max(dot(normal, normalize(direction + view)), 0.0),
            gl_FrontMaterial.shininess) : 0.0;
  return clamp(gl_Color.rgb * (gl_LightSource[light].ambient.rgb +
               gl_LightSource[light].diffuse.rgb * diffuse) +
               gl_FrontMaterial.specular.rgb *
               gl_LightSource[light].specular.rgb * specular, 0.0, 1.0);
}
void main() {
  vec4 eye = gl_ModelViewMatrix * gl_Vertex;
  vec3 normal = normalize(gl_NormalMatrix * gl_Normal);
  vec3 view = normalize(-eye.xyz);
  baseColor = illuminate(0, normal, view);
  sunColor = illuminate(1, normal, view);
  gl_TexCoord[1] = vec4(dot(gl_EyePlaneS[1], eye), dot(gl_EyePlaneT[1], eye),
                       dot(gl_EyePlaneR[1], eye), dot(gl_EyePlaneQ[1], eye));
  gl_Position = ftransform();
}
 )glsl";
  const char *fragment = R"glsl(#version 120
uniform sampler2DShadow shadowMap;
varying vec3 baseColor, sunColor;
void main() {
  float lit = shadow2DProj(shadowMap, gl_TexCoord[1]).r;
  vec3 base = floor(baseColor * 255.0 + 0.5) / 255.0;
  vec3 sun = floor(sunColor * 255.0 + 0.5) / 255.0;
  gl_FragColor = vec4(base + (lit >= 0.99 ? sun : vec3(0.0)), 1.0);
}
 )glsl";

  GLuint vs = compile(GL_VERTEX_SHADER, vertex),
         fs = compile(GL_FRAGMENT_SHADER, fragment);
  if (vs && fs) {
    program_ = glCreateProgram();
    glAttachShader(program_, vs);
    glAttachShader(program_, fs);
    glLinkProgram(program_);
    GLint ok = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &ok);
    if (!ok) {
      glDeleteProgram(program_);
      program_ = 0;
    }
  }
  if (vs)
    glDeleteShader(vs);
  if (fs)
    glDeleteShader(fs);
  if (program_) {
    glUseProgram(program_);
    glUniform1i(glGetUniformLocation(program_, "shadowMap"), 1);
    glUseProgram(0);
  }
}
CameraLighting::~CameraLighting() {
  if (program_)
    glDeleteProgram(program_);
}
bool CameraLighting::supports(const mjvScene *s, const mjrContext *c) const {
#if mjVERSION_HEADER == 3015000
  if (!program_ || s->nlight != 2 || s->lights[0].type != mjLIGHT_DIRECTIONAL ||
      s->lights[1].type != mjLIGHT_DIRECTIONAL || s->lights[0].castshadow ||
      !s->lights[1].castshadow || !s->flags[mjRND_SHADOW] || !c->shadowFBO ||
      s->flags[mjRND_FOG] || s->flags[mjRND_WIREFRAME] ||
      s->flags[mjRND_SEGMENT])
    return false;
  for (int i = 0; i < s->ngeom; i++)
    if (s->geoms[i].reflectance > 0)
      return false;
  return true;
#else
  (void)s;
  (void)c;
  return false;
#endif
}
bool CameraLighting::supports(const mjvGeom *g, const mjrContext *c) const {
  return !g->transparent && g->rgba[3] == 1 && g->emission == 0 &&
         g->texid < 0 &&
         (g->matid < 0 ||
          c->mat_texid[mjNTEXROLE * g->matid + mjTEXROLE_RGB] < 0);
}
void CameraLighting::begin() const {
  glUseProgram(program_);
  glDisable(GL_BLEND);
}
void CameraLighting::end() const {
  glUseProgram(0);
  glEnable(GL_BLEND);
}
} // namespace simple_rbt
