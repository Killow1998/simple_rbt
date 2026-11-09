#pragma once
#include <stddef.h>
#include <mujoco/mujoco.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Callbacks return 0 on success; errors go in the supplied buffer.
 * No C++ exceptions may cross this interface. Model/data are borrowed.
 * update writes all model->nu controls in model actuator order and native
 * actuator units. The framework adds no observation transform, delay or PD.
 * after_step is optional: mj_step has returned, mj_forward has not run.
 * qpos/qvel/time are integrated; force/contact fields belong to that step. */
typedef struct {
  void *context;
  int (*update)(void *, const mjModel *, const mjData *, const mjtNum command[3],
                mjtNum *controls, char *error, size_t error_capacity);
  int (*after_step)(void *, const mjModel *, const mjData *, char *error,
                    size_t error_capacity);
  void (*destroy)(void *);
} SimpleRbtControllerV1;

/* Copy any needed configuration during creation. On failure, the plugin owns
 * cleanup. Check host_mujoco_version against the plugin's mjVERSION_HEADER. */
typedef int SimpleRbtControllerCreateV1(
    int host_mujoco_version, const mjModel *model, const char *config_json,
    SimpleRbtControllerV1 *out, char *error, size_t error_capacity);

SimpleRbtControllerCreateV1 simple_rbt_controller_create_v1;

#ifdef __cplusplus
}
#endif
