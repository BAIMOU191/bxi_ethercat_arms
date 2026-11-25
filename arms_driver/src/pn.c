#include "arms/pn.h"
#include <stdint.h>

/* 依据 KGU EtherCAT V1.2 附录 A、混合力控/传感器模式章节和报警附录。
 * 仅在 ESI 中占位、手册未说明的参数不收录。 */
static const int enc[] = {0, 1, 3, -1}, ratio[] = {51, 81, 101, -1};
static const int multiturn[] = {0, 16, -1}, poles[] = {4, 5, 8, -1};
#define TYPE_ONLY "手册未给数值范围；仅检查 INT16 编码，写前核实本机型规格"
static const arms_pn_info catalog[] = {
    {0,"参数复位",NULL,0,9999,NULL,"警告：1111 会在重新上电时恢复出厂参数，丢失原有设置"},
    {1,"驱动模式",NULL,2,5,NULL,"2=EtherCAT；改为串行模式可能失去总线控制"},
    {2,"监控量选择",NULL,1,100,NULL,"选择驱动器本地显示的监控量"},
    {4,"高速端编码器类型",NULL,0,3,enc,"重新上电生效"},
    {5,"内部速度模式旋转方向",NULL,0,1,NULL,"不是主站关节方向"},
    {6,"高速端绝对编码器分辨率",NULL,17,24,NULL,"单位 bit"},
    {9,"绝对编码器协议",NULL,0,2,NULL,"重新上电生效"},
    {10,"编码器 CRC 容错阈值",NULL,0,50,NULL,"按机型保留厂家设置"},
    {11,"编码器断线容错阈值",NULL,0,50,NULL,"按机型保留厂家设置"},
    {20,"再生电阻投入阈值",NULL,INT16_MIN,INT16_MAX,NULL,TYPE_ONLY},
    {21,"电压/电流软件型号",NULL,INT16_MIN,INT16_MAX,NULL,TYPE_ONLY},
    {24,"母线电压报警阈值",NULL,INT16_MIN,INT16_MAX,NULL,TYPE_ONLY},
    {26,"制动器保持电压","brake_voltage",0,24,NULL,"单位 V；不是抱闸开关"},
    {28,"高速端转速限制",NULL,0,5000,NULL,"rpm；PP/CSV/CSP，不是 MIT 关节限速"},
    {31,"电流环比例增益",NULL,0,9999,NULL,"不是 MIT 命令的 kp"},
    {32,"电流环积分增益",NULL,0,9999,NULL,"驱动器内部增益"},
    {33,"速度环比例增益",NULL,0,9999,NULL,"驱动器内部增益"},
    {34,"速度环积分增益",NULL,0,9999,NULL,"驱动器内部增益"},
    {35,"传感器力矩环比例增益",NULL,INT16_MIN,INT16_MAX,NULL,TYPE_ONLY},
    {36,"传感器力矩环积分增益",NULL,INT16_MIN,INT16_MAX,NULL,TYPE_ONLY},
    {39,"温度报警阈值",NULL,0,100,NULL,"单位 °C"},
    {42,"低速端编码器方向",NULL,0,1,NULL,"厂家注明客户无需修改"},
    {44,"速度指令滤波",NULL,0,9,NULL,"PP/CSP/CSV；0=不滤波"},
    {45,"低速端编码器分辨率档位",NULL,0,3,NULL,"关闭 / 19 / 20 / 23 bit"},
    {49,"旋转方向取反",NULL,0,1,NULL,"改变后须重新标定机械零位"},
    {50,"高速端 Biss_C 编码器方向",NULL,0,1,NULL,"与实物方向一致"},
    {51,"减速比",NULL,51,101,ratio,"重新上电生效"},
    {63,"位置环比例增益（细调）",NULL,1000,5000,NULL,"与 Pn065 配合"},
    {64,"位置前馈",NULL,0,100,NULL,"单位 %"},
    {65,"位置环比例增益（粗调）",NULL,0,24,NULL,"不是 MIT 命令的 kp"},
    {70,"EtherCAT 丢帧容错阈值",NULL,0,100,NULL,"保持出厂值"},
    {71,"高速端超速报警阈值",NULL,0,6500,NULL,"rpm；不是运动目标"},
    {74,"高速端编码器多圈配置",NULL,0,16,multiturn,"高速端编码器配置"},
    {75,"抱闸模式",NULL,0,2,NULL,"0=正常控制抱闸，1=强制松闸，2=强制抱闸；通过 brake 流程写入"},
    {77,"DC 时钟偏移",NULL,0,1,NULL,"重新上电生效"},
    {79,"EtherCAT 从站节点地址",NULL,0,100,NULL,"重新上电生效"},
    {80,"电机极对数",NULL,4,8,poles,"必须与电机匹配"},
    {82,"转矩系数","torque_scale",0,INT16_MAX,NULL,"原始值 × 0.1 A/N·m；10 表示 1 A/N·m，按实物标定"},
    {85,"电机额定电流",NULL,10,9999,NULL,"单位 0.01 A；保持出厂值"},
    {88,"0x606C 速度反馈单位",NULL,0,1,NULL,"0=rpm，1=count/s；非 MIT 反馈对象"},
    {91,"瞬时过载倍数",NULL,10,60,NULL,"单位 0.1 倍；保持出厂值"},
    {92,"长时过载倍数",NULL,5,30,NULL,"单位 0.1 倍；保持出厂值"},
    {96,"上电位置范围",NULL,0,1,NULL,"双编码器；重新上电生效"},
    {97,"参数保存触发",NULL,1,1,NULL,"写 1 等价于 pn_save；程序管理完整脉冲，空闲回读为 0"},
    {101,"硬件清零触发",NULL,1,1,NULL,"写 1 等价于 zero（硬件校零）；程序管理完整脉冲，重新上电生效"},
    {106,"加速度限制值",NULL,0,300,NULL,"PP/CSV/CSP；Pn107=1 时有效"},
    {107,"加速度限制开关",NULL,0,1,NULL,"PP/CSV/CSP；不是 MIT 模式开关"},
    {111,"内部转速",NULL,0,6000,NULL,"内部速度设置"},
    {148,"传感器力矩闭环位置",NULL,0,1,NULL,"扭矩传感器模式；0=控制器，1=驱动器"},
    {150,"单/双编码器选择",NULL,0,1,NULL,"0=双，1=单；重新上电生效"},
};
const arms_pn_info *arms_pn_catalog(size_t *count)
{
    if (count) *count = sizeof(catalog) / sizeof(catalog[0]);
    return catalog;
}
const arms_pn_info *arms_pn_lookup(unsigned n)
{
    size_t count;
    arms_pn_catalog(&count);
    for (size_t i = 0; i < count; ++i) if (catalog[i].number == n) return &catalog[i];
    return NULL;
}
bool arms_pn_accepts(const arms_pn_info *p, int v)
{
    if (!p || v < p->min || v > p->max) return false;
    if (!p->choices) return true;
    for (const int *c = p->choices; *c >= 0; ++c) if (*c == v) return true;
    return false;
}
int arms_pn_from_index(unsigned index)
{
    if (index < 0x2000 || index > 0x2999 || (index & 15) > 9 || ((index >> 4) & 15) > 9) return -1;
    return (int)(((index >> 8) & 15) * 100 + ((index >> 4) & 15) * 10 + (index & 15));
}
