/*
 * KGU EtherCAT actuator (KaiserDrive KDE) - MIT mode process data layout.
 * Source: KGU EtherCAT manual V1.2 (mode 0x6060 = 10, "混合力控") and the
 * mapping verified on hardware in bxi_pci_drv_testmotor (23 B out / 16 B in).
 */
#ifndef ARMS_KGU_H
#define ARMS_KGU_H

#include <stdint.h>
#include <string.h>

#define KGU_VENDOR_ID 0x00010203u
#define KGU_PRODUCT_CODE 0x00000402u

#define KGU_MODE_MIT 10

#define KGU_OUT_BYTES 23
#define KGU_IN_BYTES 16

/* RxPDO 0x1601 (master -> drive) */
#define KGU_OUT_CONTROLWORD 0  /* 0x6040 u16 */
#define KGU_OUT_TARGET_POS 2   /* 0x4010 f32 rad (output side) */
#define KGU_OUT_TARGET_VEL 6   /* 0x4011 f32 rad/s */
#define KGU_OUT_TARGET_TFF 10  /* 0x4012 f32 N.m */
#define KGU_OUT_KP 14          /* 0x4013 f32 */
#define KGU_OUT_KD 18          /* 0x4014 f32 */
#define KGU_OUT_MODE 22        /* 0x6060 i8 */

/* TxPDO 0x1A01 (drive -> master) */
#define KGU_IN_STATUSWORD 0    /* 0x6041 u16 */
#define KGU_IN_POS 2           /* 0x4000 f32 rad */
#define KGU_IN_VEL 6           /* 0x4001 f32 rad/s */
#define KGU_IN_TORQUE 10       /* 0x4002 f32 N.m */
#define KGU_IN_ERROR 14        /* 0x603F u16 */

/* CiA402 control words (manual table "402 控制命令与状态字") */
#define KGU_CW_DISABLE 0x0000u
#define KGU_CW_SHUTDOWN 0x0006u
#define KGU_CW_SWITCH_ON 0x0007u
#define KGU_CW_ENABLE 0x000Fu
#define KGU_CW_FAULT_RESET 0x0080u

/* Status word patterns (masked with 0x006F) */
#define KGU_SW_FAULT_BIT 0x0008u

/* Supported cycle periods: 0.5 / 1 / 2 / 4 / 5 / 8 ms */

static inline uint16_t kgu_get_u16(const uint8_t *p, unsigned int off)
{
    return (uint16_t)(p[off] | ((uint16_t)p[off + 1] << 8));
}

static inline void kgu_put_u16(uint8_t *p, unsigned int off, uint16_t v)
{
    p[off] = (uint8_t)(v & 0xFFu);
    p[off + 1] = (uint8_t)(v >> 8);
}

static inline float kgu_get_f32(const uint8_t *p, unsigned int off)
{
    uint32_t raw = (uint32_t)p[off] | ((uint32_t)p[off + 1] << 8) |
                   ((uint32_t)p[off + 2] << 16) | ((uint32_t)p[off + 3] << 24);
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static inline void kgu_put_f32(uint8_t *p, unsigned int off, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    p[off] = (uint8_t)(raw & 0xFFu);
    p[off + 1] = (uint8_t)((raw >> 8) & 0xFFu);
    p[off + 2] = (uint8_t)((raw >> 16) & 0xFFu);
    p[off + 3] = (uint8_t)(raw >> 24);
}

typedef struct {
    uint16_t controlword;
    float pos, vel, tff, kp, kd;
    int8_t mode;
} kgu_command;

typedef struct {
    uint16_t statusword;
    float pos, vel, torque;
    uint16_t error;
} kgu_feedback;

static inline void kgu_encode(uint8_t *out, const kgu_command *c)
{
    kgu_put_u16(out, KGU_OUT_CONTROLWORD, c->controlword);
    kgu_put_f32(out, KGU_OUT_TARGET_POS, c->pos);
    kgu_put_f32(out, KGU_OUT_TARGET_VEL, c->vel);
    kgu_put_f32(out, KGU_OUT_TARGET_TFF, c->tff);
    kgu_put_f32(out, KGU_OUT_KP, c->kp);
    kgu_put_f32(out, KGU_OUT_KD, c->kd);
    out[KGU_OUT_MODE] = (uint8_t)c->mode;
}

static inline void kgu_decode(const uint8_t *in, kgu_feedback *f)
{
    f->statusword = kgu_get_u16(in, KGU_IN_STATUSWORD);
    f->pos = kgu_get_f32(in, KGU_IN_POS);
    f->vel = kgu_get_f32(in, KGU_IN_VEL);
    f->torque = kgu_get_f32(in, KGU_IN_TORQUE);
    f->error = kgu_get_u16(in, KGU_IN_ERROR);
}

#endif
