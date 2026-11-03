"""Pure presentation of FPGA local planning diagnostics; never sends control."""


def planning_summary(sample, live):
    if sample is None:
        return None
    if not live:
        return '雷达规划：等待更新 / 已超时'
    mode = '接管' if sample.actual_takeover else '观察'
    if not sample.result_valid and sample.flags & 0x80:
        if sample.flags & 0x800:
            reason = '计算/config故障'
        elif sample.flags & 0x100:
            reason = '无可行路'
        elif sample.flags & 0x200:
            reason = '等待地图'
        elif sample.flags & 0x400:
            reason = '旧结果已撤销'
        else:
            reason = ''
        path = ('保护停车' if sample.actual_takeover else '建议停车')+(f'（{reason}）' if reason else '')
    elif sample.flags & 0x800:
        path = '计算/config故障'
    elif sample.flags & 0x400:
        path = '旧结果已撤销'
    elif sample.flags & 0x100:
        path = '无可行路'
    elif not sample.result_valid:
        path = '未知地图' if sample.flags & 0x200 else '等待有效结果'
    else:
        path = ('左弧', '直行', '右弧')[sample.candidate % 3]
        if not sample.actual_takeover:
            path = '建议' + path
    # A distance from a withdrawn result is not a current safety envelope.
    safety = f'安全{sample.stop_mm/10:g}cm' if sample.result_valid else '安全距离待更新'
    return f'雷达规划：{mode}｜{path}｜{safety}'


def planning_details(sample, live):
    summary = planning_summary(sample, live)
    if summary is None:
        return []
    if not live:
        return [summary]
    flags = sample.flags
    calibration = '已验收' if flags & 4 else '未验收（名义模型，待实测标定）'
    enabled = '已启用' if flags & 2 else '未启用'
    prefix = ('保护停车目标' if not sample.result_valid and flags & 0x80 else '接管目标') if sample.actual_takeover else '建议目标（仅观察）'
    candidate = str(sample.candidate) if sample.result_valid else '--'
    states = [name for bit, name in (
        (0, '能力支持'), (3, '地图新鲜'), (4, '结果有效'),
        (6, '动态减速'), (7, '动态hold'), (8, '无可行路'),
        (9, '未知地图'), (10, '旧结果撤销'), (11, '计算/config故障'),
        (12, '跟随'), (13, '原雷达控制使能')) if flags & (1 << bit)]
    return [summary,
            f'规划详情：候选 {candidate}/{sample.candidate_count}，计算 {sample.cycles} 周期，数据年龄 {sample.age_ms}ms',
            f'规划模型：算法{enabled}，模型{calibration}；'+('，'.join(states) or '无有效状态'),
            f'{prefix} 左 {sample.target_l} / 右 {sample.target_r} counts/20ms；净空 {sample.clearance_mm}mm，速度上界 {sample.speed_mm_s}mm/s',
            f'规划距离：停车 {sample.stop_mm}mm / 减速 {sample.slow_mm}mm，延迟预算 {sample.latency_budget_ms}ms',
            f'规划可行 0x{sample.admissible_mask:03X} / 未知 0x{sample.unknown_mask:03X}，目标航向 {sample.goal_cd/100:.2f}°',
            f'规划扫描 {sample.scan_id}，代次 {sample.generation}，命令代次 {sample.command_epoch}，时间戳 {sample.timestamp_ms}ms，序号 {sample.seq}，flags 0x{flags:04X}']


def planning_snapshot(sample, live):
    if sample is None:
        return None
    snapshot = dict(vars(sample), live=bool(live), actual_takeover=bool(live and sample.actual_takeover),
                    result_valid=bool(live and sample.result_valid),
                    mode=('takeover' if sample.actual_takeover else 'observation') if live else 'waiting',
                    summary=planning_summary(sample, live), details=planning_details(sample, live))
    if not live:
        # Raw old flags/targets must not be consumed as the current suggestion.
        for field in ('candidate','target_l','target_r','admissible_mask','unknown_mask',
                      'stop_mm','slow_mm','clearance_mm','speed_mm_s','goal_cd','flags'):
            snapshot[field] = None
    return snapshot
