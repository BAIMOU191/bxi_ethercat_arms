#ifndef ARMS_TEST_RUNTIME_ENV_H
#define ARMS_TEST_RUNTIME_ENV_H

#include <stdbool.h>

/**
 * @brief 非 root 时通过 sudo 以相同参数重新启动本程序；成功时不返回
 * @return 已是 root 时返回 0；无法重新启动时返回 -1
 */
int arms_env_ensure_root(int argc, char **argv);

/** @brief EtherLab 主站未加载时启动 /etc/init.d/ethercat，并等待主站就绪 */
int arms_env_start_master(void);

/**
 * @brief 调用 rt_setup.sh apply 配置实时运行环境
 * @param config 配置文件路径，NULL 时与驱动共用默认路径查找
 * @param rt_cpu -2 使用配置值；-1 不做 CPU/IRQ 调优；非负值同时覆盖实时和中断 CPU
 * @return 0 成功；-1 失败；1 未找到 rt_setup.sh（跳过）
 */
int arms_env_rt_apply(const char *config, int rt_cpu);

/** @brief 还原 arms_env_rt_apply() 所做的设置 */
void arms_env_rt_revert(void);

#endif
