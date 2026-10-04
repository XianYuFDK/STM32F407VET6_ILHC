"""电机内部 ACC/DEC：失能整定与事务回读，独立于PC参考曲线。"""
import math
import secrets
import time
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QWidget, QVBoxLayout, QGridLayout, QHBoxLayout, QLabel, QPushButton, QDoubleSpinBox
import core


class DriveSettings(QWidget):
    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.seq = secrets.randbelow(65535)+1
        self.pending = self.actual = None
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        title = QLabel('电机内置梯形加减速')
        title.setObjectName('DmCardTitle')
        layout.addWidget(title)
        fields = QGridLayout()
        self.acc, self.dec = QDoubleSpinBox(), QDoubleSpinBox()
        for row, (label, spin) in enumerate((('加速度 ACC · Krad/s²', self.acc),
                                             ('减速度大小 · Krad/s²', self.dec))):
            spin.setRange(0.000001, 1000)
            spin.setDecimals(6)
            spin.setSingleStep(0.001)
            spin.setValue(2)
            fields.addWidget(QLabel(label), row, 0)
            fields.addWidget(spin, row, 1)
        layout.addLayout(fields)
        self.readback = QLabel('电机回读：—')
        self.readback.setObjectName('DmFeedback')
        self.readback.setWordWrap(True)
        layout.addWidget(self.readback)
        actions = QHBoxLayout()
        self.read_btn, self.write_btn = QPushButton('读取 ACC / DEC'), QPushButton('写入并回读')
        self.write_btn.setObjectName('SmallPrimary')
        self.read_btn.clicked.connect(lambda: self.request(False))
        self.write_btn.clicked.connect(lambda: self.request(True))
        actions.addWidget(self.read_btn)
        actions.addWidget(self.write_btn)
        layout.addLayout(actions)
        self.status = QLabel('失能后读写；减速度自动取负。写入电机 RAM，掉电恢复原保存值。')
        self.status.setWordWrap(True)
        self.status.setObjectName('HintLabel')
        layout.addWidget(self.status)
        hint = QLabel('关闭“上位机五次曲线”时使用内置梯形。Krad/s² = 1000 rad/s²；'
                      '驱动原始单位不按回转轴角度或额外传动比换算。')
        hint.setWordWrap(True)
        hint.setObjectName('HintLabel')
        layout.addWidget(hint)

    def lock(self, moving=False):
        for widget in (self.acc, self.dec, self.read_btn, self.write_btn):
            widget.setEnabled(not moving and self.pending is None)

    def reset(self, reason='需要重新读取电机参数'):
        self.pending = self.actual = None
        self.readback.setText('电机回读：—')
        self.status.setText(reason)
        self.lock(self.panel.active is not None or self.panel.flow != 'idle')

    def request(self, write):
        try:
            self.panel._idle()
            values, now = self.panel.snapshot()
            if int(values[16]) != 0:
                raise ValueError('请先失能电机，再整定加减速参数')
            self.seq = self.seq % 65535+1
            self.actual = None
            self.readback.setText('电机回读：等待响应')
            # 刷新真实CAN失能反馈；固件同时检查反馈新鲜度。
            self.panel.issue('DMOFF')
            self.pending = dict(seq=self.seq, id=int(values[12]), write=write,
                                acc=self.acc.value(), dec=-self.dec.value(), deadline=now+3)
            self.lock()
            self.status.setText('正在写入并读取实际值…' if write else '正在读取电机参数…')
            QTimer.singleShot(100, lambda seq=self.seq: self.submit(seq))
        except ValueError as exc:
            self.status.setText(str(exc))

    def submit(self, seq):
        pending = self.pending
        if pending is None or pending['seq'] != seq:
            return
        try:
            values, _ = self.panel.snapshot()
            if int(values[12]) != pending['id'] or int(values[16]) != 0:
                raise ValueError('电机 ID 或失能状态变化，事务取消')
            text = ('DMACCDEC=%d,%.6f,%.6f' % (seq, pending['acc'], pending['dec'])
                    if pending['write'] else 'DMREAD=%d' % seq)
            self.panel.issue(text)
        except ValueError as exc:
            self.reset(str(exc))

    def handle_reply(self, text):
        reply = core.dm_register_feedback(text)
        p = self.pending
        if reply is None or p is None or (reply['seq'], reply['id']) != (p['seq'], p['id']):
            return False
        self.pending = None
        status = reply['status']
        if status == 0 and not (math.isfinite(reply['acc']) and math.isfinite(reply['dec'])
                                and reply['acc'] > 0 and reply['dec'] < 0):
            status = 6
        if status == 0 and p['write'] and any(
                abs(reply[key]-p[key]) > abs(p[key])*1e-5+1e-6 for key in ('acc', 'dec')):
            status = 4
        if status:
            self.actual = None
            message = {1: '请求被拒绝：失能反馈过期、CAN忙或另一事务未结束',
                       2: 'CAN发送失败', 3: '电机回读超时', 4: '写入与回读不一致',
                       5: '事务已取消', 6: '电机返回非法参数'}[status]
            self.status.setText(message+'；可能部分写入，请失能后重新读取')
            self.readback.setText('电机回读：未确认')
        else:
            self.actual = reply
            prefix = '模拟回读' if reply['simulated'] else '电机回读'
            self.readback.setText('%s ACC %.6g · DEC %.6g Krad/s²' % (prefix, reply['acc'], reply['dec']))
            if not p['write']:
                self.acc.setValue(reply['acc'])
                self.dec.setValue(-reply['dec'])
            self.status.setText(('模拟接口已确认' if reply['simulated'] else '电机实际回读已确认')+
                                '；可使能并执行位置目标，掉电后需重新读取')
        self.lock()
        return True

    def tick(self):
        if self.pending and time.monotonic() > self.pending['deadline']:
            self.reset('未收到有效回读；检查STM32固件版本、CAN接线与电机寄存器支持')

    def require_actual(self, motor_id):
        if self.pending or self.actual is None or self.actual['id'] != motor_id:
            raise ValueError('内置梯形调试前，请失能并读取或写入 ACC / DEC，确认回读')
        if self.actual['simulated'] != (self.panel.owner.sim is not None):
            raise ValueError('参数回读来自其他连接会话，请重新读取')
