#ifndef ARMS_PN_H
#define ARMS_PN_H

#include <stdbool.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

/* KGU EtherCAT V1.2：值均为对象字典 INT16 原始值。枚举非空时以 -1 结束。 */
typedef struct {
    unsigned number;
    const char *name, *alias;
    int min, max;
    const int *choices;
    const char *note;
} arms_pn_info;
const arms_pn_info *arms_pn_catalog(size_t *count);
const arms_pn_info *arms_pn_lookup(unsigned number);
bool arms_pn_accepts(const arms_pn_info *p, int value);
/* 仅反解 Pn000..199 的 BCD 地址；不是该字典的对象返回 -1。 */
int arms_pn_from_index(unsigned index);

#ifdef __cplusplus
}
#endif
#endif
