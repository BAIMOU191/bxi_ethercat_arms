#!/usr/bin/env python3
"""把旧格式 arms.json 的关节配置转换为 joint_defaults + pos_limit / vel_limit / tor_limit 格式。

用法: migrate_config.py <arms.json> [--dry-run]

转换规则:
  limit.vel_fault    -> vel_limit（反馈超速保护阈值，rad/s）
  limit.torque_fault -> tor_limit（反馈超力矩保护阈值，N.m）
  pos_limit 统一取默认 [-3.0, 3.0]（软位置限位，越界后屏蔽朝外的速度和前馈）
  limit.pos / limit.vel / limit.tff、gain_max、gain_default、home 删除（不再截断指令，不再有默认增益和回零位置）
  offset_rad、direction 及其他字段原样保留。
所有关节取值相同的保护参数写入 joint_defaults，个别关节不同时写在该关节上。
原文件先备份为 <arms.json>.bak（已存在时加时间戳），再写入新内容。
"""
import json
import pathlib
import shutil
import sys
import time

DEFAULTS = {'pos_limit': [-3.0, 3.0], 'vel_limit': 3.0, 'tor_limit': 30.0}
OLD_FIELDS = ('limit', 'gain_max', 'gain_default', 'home')
WIDTH = 110


def compact(value):
    if isinstance(value, dict):
        if not value:
            return '{}'
        return '{ ' + ', '.join(f'{json.dumps(k)}: {compact(v)}' for k, v in value.items()) + ' }'
    if isinstance(value, list):
        return '[' + ', '.join(compact(v) for v in value) + ']'
    return json.dumps(value, ensure_ascii=False)


def dump(value, indent=0, key=None):
    pad = ' ' * indent
    head = f'{pad}{json.dumps(key)}: ' if key is not None else pad
    if key == 'joints' and isinstance(value, list):
        rows = [f'{pad}  {compact(v)}' for v in value]
        return head + '[\n' + ',\n'.join(rows) + f'\n{pad}]'
    one = head + compact(value)
    if len(one) <= WIDTH or not isinstance(value, (dict, list)) or not value:
        return one
    if isinstance(value, dict):
        rows = [dump(v, indent + 2, k) for k, v in value.items()]
        return head + '{\n' + ',\n'.join(rows) + f'\n{pad}}}'
    rows = [dump(v, indent + 2) for v in value]
    return head + '[\n' + ',\n'.join(rows) + f'\n{pad}]'


def migrate(data):
    joints = data.get('joints')
    if not isinstance(joints, list):
        raise ValueError('缺少 joints 数组')
    if not any(isinstance(j, dict) and any(f in j for f in OLD_FIELDS) for j in joints):
        return None
    values = []
    for j in joints:
        lim = j.get('limit') if isinstance(j.get('limit'), dict) else {}
        values.append({
            'vel_limit': lim.get('vel_fault', j.get('vel_limit', DEFAULTS['vel_limit'])),
            'tor_limit': lim.get('torque_fault', j.get('tor_limit', DEFAULTS['tor_limit'])),
        })
    base = dict(data.get('joint_defaults') or DEFAULTS)
    base.setdefault('pos_limit', DEFAULTS['pos_limit'])
    for field in ('vel_limit', 'tor_limit'):
        seen = [v[field] for v in values]
        base[field] = max(set(seen), key=seen.count)
    out_joints = []
    for j, v in zip(joints, values):
        row = {k: x for k, x in j.items() if k not in OLD_FIELDS}
        for field in ('vel_limit', 'tor_limit'):
            row.pop(field, None)
            if v[field] != base[field]:
                row[field] = v[field]
        out_joints.append(row)
    result = {}
    for k, x in data.items():
        if k == 'joint_defaults':
            continue
        if k == 'joints':
            result['joint_defaults'] = base
            result['joints'] = out_joints
        else:
            result[k] = x
    return result


def main(argv):
    if len(argv) < 2 or argv[1] in ('-h', '--help'):
        print(__doc__.strip())
        return 0 if len(argv) >= 2 else 2
    path = pathlib.Path(argv[1])
    dry = '--dry-run' in argv[2:]
    data = json.loads(path.read_text(encoding='utf-8'))
    result = migrate(data)
    if result is None:
        print(f'{path} 已是新格式，无需转换')
        return 0
    text = dump(result) + '\n'
    for old, new in zip(data['joints'], json.loads(text)['joints']):
        if old.get('offset_rad') != new.get('offset_rad'):
            raise SystemExit(f'关节 {old.get("joint")} offset_rad 转换后不一致，未写入')
    if dry:
        sys.stdout.write(text)
        return 0
    backup = path.with_name(path.name + '.bak')
    if backup.exists():
        backup = path.with_name(path.name + time.strftime('.%Y%m%d_%H%M%S.bak'))
    shutil.copy2(path, backup)
    path.write_text(text, encoding='utf-8')
    print(f'已转换 {path}，原文件备份为 {backup}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
