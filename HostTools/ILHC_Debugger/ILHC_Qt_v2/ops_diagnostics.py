"""OPS固件诊断文字；uint32字段保存精确值，不由24通道反推会话状态。"""
CAUSES={1:'会话号变化',2:'OPS时间戳回退',4:'OPS有效帧间隔超过200ms',8:'协议V2切换为V1',
        16:'IMU重新建立航向',32:'USART2接收重启失败',64:'OPS接收队列溢出',
        128:'OPS接收队列故障',256:'OPS排队数据超过200ms',512:'USART2硬件错误',
        1024:'OPS接收恢复超过200ms',2048:'OPS接收已恢复（续行）'}
FIELDS={
    'OPSE':('event_id','cause_mask','device_tick_ms','previous_session_id','session_id',
            'previous_timestamp_ms','timestamp_ms','ops_sequence'),
    'OPSX':('event_id','flags','frame_gap_ms','valid_pose_age_ms','rx_packet_age_ms',
            'uart_error','crc_errors','rx_overflows'),
    'OPSD':('device_tick_ms','session_id','ops_sequence','timestamp_ms','flags','pose_valid',
            'valid_pose_age_ms','crc_errors','format_errors'),
    'OPSR':('device_tick_ms','uart_error_count','last_uart_error','rx_overflows',
            'stale_packets','restart_failures','diagnostic_drops'),
}


def parse_ops_text(text):
    parts=text.split()
    if not parts or parts[0] not in FIELDS:return None
    names=FIELDS[parts[0]]
    if len(parts)!=len(names)+1 or any(not p.isascii() or not p.isdecimal() for p in parts[1:]):return None
    values=[int(p) for p in parts[1:]]
    if any(v>0xffffffff for v in values):return None
    result=dict(kind=parts[0],**dict(zip(names,values)))
    if 'flags' in result:
        if result['flags']>255:return None
        result['imu_rebased']=bool(result['flags']&4)
        result['imu_online']=bool(result['flags']&2)
    if 'ops_sequence' in result and result['ops_sequence']>65535:return None
    if 'pose_valid' in result and result['pose_valid'] not in (0,1):return None
    if 'cause_mask' in result:
        mask=result['cause_mask'];result['causes']=[name for bit,name in CAUSES.items() if mask&bit]
        result['unknown_cause_bits']=mask&~sum(CAUSES)
    return result


def summarize(records):
    """事件与现场按ID关联；丢失的另一半明确标记，不套用后续状态。"""
    faults={};snapshots=[]
    for record in records:
        if record['kind'] in ('OPSE','OPSX'):
            fault=faults.setdefault(record['event_id'],dict(event_id=record['event_id']))
            fault[record['kind']]=record
        else:snapshots.append(record)
    for fault in faults.values():
        fault['complete']=all(k in fault for k in ('OPSE','OPSX'))
        fault['causes']=fault.get('OPSE',{}).get('causes',[])
    return dict(faults=list(faults.values()),snapshots=snapshots,
                subcause_recorded=any('OPSE' in fault for fault in faults.values()))
