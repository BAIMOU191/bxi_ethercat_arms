#pragma once
#ifdef __cplusplus
extern "C" {
#endif
/* Acquire before configuring the host or requesting motor power. Close fd after cleanup. */
int arms_acquire_lease(void);
#ifdef __cplusplus
}
#endif
