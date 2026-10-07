"""位置速度模式独立调试页；复用现有串口协议，不伪造夹爪或 CAN 确认。"""
from __future__ import annotations

import math
import time
from pathlib import Path

from PySide6.QtCore import QTimer, Signal, Qt
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                              QLabel, QPushButton, QDoubleSpinBox, QCheckBox, QComboBox,
                              QFrame)

import core
from dm_drive import DriveSettings
from dm_motion import (POSITIONS, ReferenceMove, Arrival, motor_rad, validate_target,
                       load_profile, save_profile)


class PositionPanel(QWidget):
    commandRequested = Signal(str)

    def __init__(self, owner, path):
        super().__init__()
        self.owner, self.path = owner, Path(path)
        self.positions = {}
        self.profile_id = 3
        self.requested_mode = None
        self.mode_time = 0.0
        self.active = None
        self.flow = "idle"
        self.flow_slot = None
        self.issuing = False
        self.inputs = {}
        self.setObjectName("DmPositionPanel")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(14)
        columns = QGridLayout()
        columns.setSpacing(14)
        left = QVBoxLayout()
        left.setSpacing(12)
        motion_card, motion = self.card("回转调试", "角度以回转轴为准，位置决定旋转方向")
        left.addWidget(motion_card)
        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(10)
        self.advanced_card, advanced = self.card()
        self.advanced_toggle = QPushButton("＋ 曲线、限位与停稳设置")
        self.advanced_toggle.setObjectName("DmDisclosure")
        self.advanced_toggle.setCheckable(True)
        advanced.addWidget(self.advanced_toggle)
        self.advanced_body = QWidget()
        self.advanced_body.setObjectName("DmAdvancedBody")
        advanced_body = QVBoxLayout(self.advanced_body)
        advanced_body.setContentsMargins(0, 8, 0, 0)
        advanced_body.setSpacing(12)
        advanced_grid = QGridLayout()
        advanced_grid.setHorizontalSpacing(14)
        advanced_grid.setVerticalSpacing(10)
        specs = [("target", "目标角度 °", -716, 716, 0, 0.1),
                 ("speed", "速度上限 °/s", 0.01, 360, 15, 1),
                 ("acceleration", "参考加速度 °/s²", 0.01, 720, 30, 1),
                 ("step", "点动步长 °", 0.01, 30, 1, 0.1),
                 ("lower", "软限位下限 °", -716, 716, -180, 1),
                 ("upper", "软限位上限 °", -716, 716, 180, 1),
                 ("tolerance", "到位误差 °", 0.01, 10, 0.5, 0.1),
                 ("velocity", "停稳速度 °/s", 0.01, 30, 1, 0.1),
                 ("dwell", "连续停稳 s", 0.1, 5, 0.3, 0.1),
                 ("timeout", "曲线结束后超时 s", 1, 120, 10, 1),
                 ("ratio", "额外传动比", 0.01, 100, 1, 0.1)]
        advanced_keys = ["acceleration", "ratio", "lower", "upper",
                         "tolerance", "velocity", "dwell", "timeout"]
        for key, label, lo, hi, value, step in specs:
            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setDecimals(3)
            spin.setSingleStep(step)
            spin.setValue(value)
            spin.setMinimumWidth(90)
            self.inputs[key] = spin
            caption = QLabel(label)
            caption.setObjectName("DmFieldLabel")
            if key in ("target", "speed", "step"):
                row = ("target", "speed", "step").index(key)
                grid.addWidget(caption, row, 0)
                grid.addWidget(spin, row, 1)
            else:
                i = advanced_keys.index(key)
                field = QVBoxLayout()
                field.setSpacing(5)
                field.addWidget(caption)
                field.addWidget(spin)
                advanced_grid.addLayout(field, i//4, i % 4)
        grid.setColumnStretch(1, 1)
        motion.addLayout(grid)
        self.smooth = QCheckBox("上位机五次曲线（关闭时使用内置梯形）")
        self.smooth.setToolTip("五次参考曲线，50 ms 更新；参考加速度不是电机驱动寄存器")
        self.smooth.setChecked(False)
        self.inputs["acceleration"].setEnabled(False)
        self.smooth.toggled.connect(lambda checked: self.inputs["acceleration"].setEnabled(checked))
        motion.addWidget(self.smooth)
        self.conversion = QLabel()
        self.conversion.setObjectName("HintLabel")
        self.conversion.setWordWrap(True)
        motion.addWidget(self.conversion)
        actions = QGridLayout()
        actions.setSpacing(8)
        for label, action in (("读取实际为目标", self.use_actual),
                              ("− 点动", lambda: self.jog(-1)),
                              ("＋ 点动", lambda: self.jog(1)),
                              ("执行位置目标", lambda: self.move(self.value("target"))),
                              ("停止 / 失能", self.stop)):
            btn = QPushButton(label)
            if label == "停止 / 失能":
                btn.setObjectName("EmergencyButton")
            elif label == "执行位置目标":
                btn.setObjectName("PrimaryButton")
            btn.clicked.connect(action)
            if label == "读取实际为目标":
                actions.addWidget(btn, 0, 0)
            elif label == "执行位置目标":
                actions.addWidget(btn, 0, 1, 1, 2)
            else:
                col = {"− 点动": 0, "＋ 点动": 1, "停止 / 失能": 2}[label]
                actions.addWidget(btn, 1, col)
        motion.addLayout(actions)
        self.feedback = QLabel("实际角度 — · 目标误差 —")
        self.feedback.setObjectName("DmFeedback")
        self.feedback.setWordWrap(True)
        motion.addWidget(self.feedback)
        self.status = QLabel("失能 → 应用位置速度模式 → 读取 ACC/DEC → 使能 → 执行目标")
        self.status.setObjectName("DmWorkflowStatus")
        self.status.setWordWrap(True)
        motion.addWidget(self.status)
        advanced_body.addLayout(advanced_grid)
        hint = QLabel("额外传动比不包含电机内置减速器。参考加速度只作用于上位机五次曲线；内部 PID 仍使用电机设置。")
        hint.setWordWrap(True)
        hint.setObjectName("HintLabel")
        advanced_body.addWidget(hint)
        advanced.addWidget(self.advanced_body)
        self.advanced_body.setVisible(False)
        self.advanced_toggle.toggled.connect(self.show_advanced)
        left.addStretch(1)
        drive_card, drive_layout = self.card()
        self.drive = DriveSettings(self)
        drive_layout.addWidget(self.drive)
        left.insertWidget(1, drive_card)
        columns.addLayout(left, 0, 0)
        preset_card, preset_layout = self.card("物料示教", "记录四个实际位置，保存后可随时前往")
        self.reference = QCheckBox("已核对电机 ID、零点及传动比")
        self.reference.setToolTip("当前电机 ID、零点和传动比必须与保存的四个示教位置一致")
        preset_layout.addWidget(self.reference)
        self.preset_inputs, self.preset_status = {}, {}
        for key, label in POSITIONS.items():
            row = QFrame()
            row.setObjectName("DmPresetRow")
            row_layout = QVBoxLayout(row)
            row_layout.setContentsMargins(10, 8, 10, 8)
            row_layout.setSpacing(6)
            row_head = QHBoxLayout()
            row_head.addWidget(QLabel(label))
            row_head.addStretch(1)
            spin = QDoubleSpinBox()
            spin.setRange(-716, 716)
            spin.setDecimals(3)
            spin.setSingleStep(0.1)
            spin.setSuffix(" °")
            spin.setMinimumWidth(92)
            lb = QLabel("未示教")
            lb.setObjectName("DmPresetState")
            self.preset_inputs[key], self.preset_status[key] = spin, lb
            row_head.addWidget(lb)
            row_layout.addLayout(row_head)
            row_actions = QHBoxLayout()
            row_actions.setSpacing(6)
            row_actions.addWidget(spin, 1)
            for text, callback in (
                    ("记录实际", lambda _=False, k=key: self.teach(k)),
                    ("保存输入", lambda _=False, k=key: self.store(k)),
                    ("前往", lambda _=False, k=key: self.goto_preset(k))):
                btn = QPushButton(text)
                if text == "前往":
                    btn.setObjectName("SmallPrimary")
                btn.clicked.connect(callback)
                row_actions.addWidget(btn)
            row_layout.addLayout(row_actions)
            preset_layout.addWidget(row)
        files = QHBoxLayout()
        for text, action in (("保存设置", self.save), ("重新加载", self.load)):
            button = QPushButton(text)
            button.clicked.connect(action)
            files.addWidget(button)
        self.file_status = QLabel("保存在本机 · " + self.path.name)
        self.file_status.setObjectName("HintLabel")
        self.file_status.setToolTip(str(self.path))
        self.file_status.setWordWrap(True)
        files.addWidget(self.file_status, 1)
        preset_layout.addLayout(files)
        columns.addWidget(preset_card, 0, 1, alignment=Qt.AlignTop)
        columns.setColumnStretch(0, 1)
        columns.setColumnStretch(1, 1)
        lay.addLayout(columns)
        flow_card, flow_layout = self.card("搬运流程", "抓取并抬升 → 回转到载盘 → 放料并抬升 → 返回抓取位")
        flow = QHBoxLayout()
        flow.setSpacing(10)
        self.slot = QComboBox()
        self.slot.setMinimumWidth(134)
        for key in ("slot1", "slot2", "slot3"):
            self.slot.addItem(POSITIONS[key], key)
        flow.addWidget(self.slot)
        self.to_slot = QPushButton("已抓稳并抬升 → 转到载盘")
        self.to_slot.setObjectName("PrimaryButton")
        self.to_slot.clicked.connect(self.start_flow)
        flow.addWidget(self.to_slot, 1)
        self.release = QPushButton("已放妥并抬升 → 返回抓取位")
        self.release.setObjectName("SuccessButton")
        self.release.setEnabled(False)
        self.release.clicked.connect(self.return_flow)
        flow.addWidget(self.release, 1)
        flow_layout.addLayout(flow)
        tip = QLabel("载盘停稳后才能确认返回。夹爪、升降及伸缩由操作者完成。")
        tip.setObjectName("HintLabel")
        tip.setWordWrap(True)
        flow_layout.addWidget(tip)
        lay.addWidget(flow_card)
        lay.addWidget(self.advanced_card)
        lay.addStretch(1)
        self.inputs["ratio"].valueChanged.connect(self.invalidate_reference)
        for spin in self.inputs.values():
            spin.valueChanged.connect(self.update_conversion)
        self.load()
        self.update_conversion()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    @staticmethod
    def card(title=None, subtitle=None):
        """统一卡片的留白与标题层级，内容控件独立维护。"""
        frame = QFrame()
        frame.setObjectName("DmCard")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)
        if title:
            label = QLabel(title)
            label.setObjectName("DmCardTitle")
            layout.addWidget(label)
        if subtitle:
            label = QLabel(subtitle)
            label.setObjectName("HintLabel")
            label.setWordWrap(True)
            layout.addWidget(label)
        return frame, layout

    def show_advanced(self, expanded):
        self.advanced_body.setVisible(expanded)
        self.advanced_toggle.setText(("－" if expanded else "＋") + " 曲线、限位与停稳设置")

    def value(self, key):
        return self.inputs[key].value()

    def say(self, text):
        self.status.setText(text)
        if hasattr(self.owner, "console"):
            self.owner.log("DM 位置：" + text, "info")

    def issue(self, text):
        self.issuing = True
        try:
            self.commandRequested.emit(text)
        finally:
            self.issuing = False

    def invalidate_reference(self, *_):
        self.reference.setChecked(False)

    def session_reset(self):
        self.cancel("连接会话结束；需要重新应用模式和确认零点")
        self.requested_mode = None
        self.invalidate_reference()
        self.drive.reset()

    def observe_command(self, text):
        cmd, _, arg = text.upper().partition("=")
        if self.drive.pending and not self.issuing and cmd.startswith('DM') and cmd not in {'DMOFF', 'DMSTOP'}:
            self.say('加减速参数事务正在执行，请等待回读或停止后重试')
            return False
        if (not self.issuing and (self.active or self.flow != "idle")
                and cmd in {"DMZERO", "DMID", "DMMODE", "DMPOS", "DMVEL",
                            "DMKP", "DMKD", "DMTOR", "DMEN", "DMACCDEC", "DMREAD"}):
            self.cancel("外部 DM 命令终止了回转流程")
            self.issue("DMOFF")
            self.say("已请求失能；等待停稳后重新执行 " + cmd)
            return False
        if cmd in {"STOP", "DMOFF", "DMSTOP", "DMZERO", "DMID", "DMMODE"}:
            self.cancel(cmd + "：旧回转流程已取消")
        elif not self.issuing and cmd in {"DMPOS", "DMVEL", "DMKP", "DMKD", "DMTOR", "DMEN"}:
            if self.active or self.flow != "idle":
                self.cancel("外部 DM 命令替换了回转流程")
                self.issue("DMOFF")
        if cmd in {"DMZERO", "DMID"}:
            self.invalidate_reference()
        if cmd in {'DMID', 'DMZERO', 'DMMODE', 'DMACCDEC', 'DMREAD'} and not (self.issuing and cmd in {'DMACCDEC', 'DMREAD'}):
            self.drive.reset()
        if cmd in {'STOP', 'DMOFF', 'DMSTOP'} and not self.issuing and self.drive.pending:
            self.drive.reset('事务已取消；写入可能部分完成，请重新读取')
        if cmd == "DMMODE":
            try:
                self.requested_mode = int(arg) if int(arg) in (1, 2) else None
            except ValueError:
                self.requested_mode = None
            self.mode_time = time.monotonic()
        return True

    def snapshot(self, require_enabled=False):
        w = self.owner
        v = w.latest
        now = time.monotonic()
        if w.worker is None and w.sim is None:
            raise ValueError("未连接；请先连接 STM32 或开启模拟")
        if (v is None or now-getattr(v,'full_state_monotonic',w.latest_received_monotonic) > core.TELEMETRY_WARN_S
                or not all(math.isfinite(v[i]) for i in (12, 13, 14, 16))):
            raise ValueError("遥测不可用或已超时")
        if v[12] != w.dm_rows["DMID"].spin.value():
            raise ValueError("实际电机 ID 与选定 ID 不一致")
        if v[16] not in (0, 1) or (require_enabled and v[16] != 1):
            raise ValueError("电机未使能或有故障")
        return v, now

    def apply_mode(self, mode):
        try:
            self._idle()
            v, _ = self.snapshot()
            if v[16] != 0:
                raise ValueError("先点击失能，收到失能反馈后再应用模式")
            self.issue("DMMODE=%d" % mode)
            self.issue("DMPOS=%.6f" % v[13])
            self.issue("DMVEL=0")
            if mode == 1:
                for cmd in ("DMKP", "DMKD", "DMTOR"):
                    self.issue(cmd + "=0")
                    self.owner.dm_rows[cmd].spin.setValue(0)
            self.say("已请求模式 %d；尚无模式回读。%s" %
                     (mode, "内置梯形请先读取 ACC/DEC，再使能。" if mode == 2 else "等待切换后再使能。"))
        except ValueError as exc:
            self.say(str(exc))

    def enable(self):
        if self.active or self.flow != "idle" or self.drive.pending:
            self.say("先完成或停止当前回转流程，再重新使能")
            return
        try:
            v, now = self.snapshot()
            mode = int(self.owner.dm_mode_combo.currentData())
            if self.requested_mode != mode or now-self.mode_time < 0.3:
                raise ValueError("先应用所选模式并等待切换")
            if mode == 1:
                self.issue("DMEN")
                return
            self.issue("DMVEL=0")
            self.issue("DMPOS=%.6f" % v[13])
            self.issue("DMEN")
            self.say("已请求使能；目标先保持当前角度，等待使能反馈后执行移动")
        except ValueError as exc:
            self.say(str(exc))

    def use_actual(self):
        try:
            v, _ = self.snapshot()
            self.inputs["target"].setValue(math.degrees(v[13])/self.value("ratio"))
        except ValueError as exc:
            self.say(str(exc))

    def jog(self, direction):
        try:
            v, _ = self.snapshot(True)
            target = math.degrees(v[13])/self.value("ratio") + direction*self.value("step")
            self.move(target)
        except ValueError as exc:
            self.say(str(exc))

    def move(self, target, completion=None):
        try:
            if self.drive.pending:
                raise ValueError('等待加减速参数回读完成后再运动')
            if self.active is not None or self.flow not in ("idle", "await_release"):
                raise ValueError("先完成或停止当前回转流程")
            v, now = self.snapshot(True)
            if (self.requested_mode != 2 or int(self.owner.dm_mode_combo.currentData()) != 2
                    or now-self.mode_time < 0.3):
                raise ValueError("本次连接需要先应用位置速度模式")
            if self.flow == "await_release" and completion != "pickup":
                raise ValueError("先确认放妥并返回，或停止当前流程")
            ratio = self.value("ratio")
            if not self.smooth.isChecked():
                self.drive.require_actual(int(v[12]))
            validate_target(target, ratio, self.value("lower"), self.value("upper"))
            start = math.degrees(v[13])/ratio
            validate_target(start, ratio, self.value("lower"), self.value("upper"))
            speed = math.radians(self.value("speed"))*ratio
            if speed > 30:
                raise ValueError("速度超过 STM32 30 rad/s 限幅")
            move = ReferenceMove(start, target, self.value("speed"),
                                 self.value("acceleration"), self.smooth.isChecked())
            core.discard_dm_targets(self.owner.line_q)
            self.inputs["target"].setValue(target)
            self.issue("DMVEL=%.6f" % speed)
            self.issue("DMPOS=%.6f" % motor_rad(move.position(0), ratio))
            self.active = dict(move=move, start=now, last_tick=now, ratio=ratio,
                               id=int(v[12]), complete=completion,
                               lower=self.value("lower"), upper=self.value("upper"),
                               deadline=now+move.duration+self.value("timeout"),
                               arrival=Arrival(self.value("tolerance"), self.value("velocity"),
                                               self.value("dwell")))
            self._lock(True)
            self.say("正在转到 %.3f°；参考时间 %.2fs，等待反馈停稳" % (target, move.duration))
            return True
        except ValueError as exc:
            self.say(str(exc))
            return False

    def _lock(self, moving):
        locked = moving or self.flow != "idle"
        for spin in list(self.inputs.values()) + list(self.preset_inputs.values()):
            spin.setEnabled(not locked)
        self.inputs["acceleration"].setEnabled(not locked and self.smooth.isChecked())
        self.smooth.setEnabled(not locked)
        self.reference.setEnabled(not locked)
        self.slot.setEnabled(not moving and self.flow == "idle")
        self.to_slot.setEnabled(not moving and self.flow == "idle")
        self.release.setEnabled(not moving and self.flow == "await_release")
        self.drive.lock(locked)

    def cancel(self, reason):
        had = self.active is not None or self.flow != "idle"
        self.active, self.flow, self.flow_slot = None, "idle", None
        core.discard_dm_targets(self.owner.line_q)
        self._lock(False)
        if had:
            self.say(reason)

    def stop(self):
        if self.drive.pending:
            self.drive.reset('事务已取消；请重新读取实际参数')
        self.cancel("回转流程已停止；已请求电机失能")
        self.issue("DMOFF")

    def tick(self):
        self.drive.tick()
        active = self.active
        try:
            v, now = self.snapshot(bool(active) or self.flow == "await_release")
            angle = math.degrees(v[13])/self.value("ratio")
            self.feedback.setText("实际 %.3f° · %.5f rad\n速度 %.3f°/s · 目标误差 %+.3f°" %
                                  (angle, v[13], math.degrees(v[14])/self.value("ratio"),
                                   self.value("target")-angle))
            if not active:
                return
            if (now-active["last_tick"] > 0.35 or now > active["deadline"]
                    or int(v[12]) != active["id"] or self.requested_mode != 2):
                raise ValueError("回转超时、调度中断或电机状态变化")
            active["last_tick"] = now
            validate_target(angle, active["ratio"], active["lower"], active["upper"])
            elapsed = now-active["start"]
            ref = active["move"].position(elapsed)
            if active["move"].smooth:
                core.put_dm_reference(self.owner.line_q, "DMPOS=%.6f" % motor_rad(ref, active["ratio"]))
            feedback_stamp=getattr(self.owner.latest,'full_state_monotonic',self.owner.latest_received_monotonic)
            done = active["arrival"].update(feedback_stamp,
                                           active["move"].target-angle,
                                           math.degrees(v[14])/active["ratio"],
                                           feedback_stamp > active["start"]
                                           and elapsed >= active["move"].duration)
            if done:
                self.active = None
                if active["complete"] == "slot":
                    self.flow = "await_release"
                    self.say("载盘已停稳；请降下并释放物料，再抬升并确认返回")
                elif active["complete"] == "pickup":
                    self.flow, self.flow_slot = "idle", None
                    self.say("已返回待抓取位并停稳；本件搬运完成，可以抓取下一件")
                else:
                    self.say("位置反馈已连续满足到位条件；电机保持目标位置")
                self._lock(False)
        except ValueError as exc:
            if active or self.flow != "idle":
                self.cancel(str(exc) + "；流程取消并请求失能")
                self.issue("DMOFF")
                self.invalidate_reference()

    def update_conversion(self, *_):
        try:
            rad = validate_target(self.value("target"), self.value("ratio"),
                                  self.value("lower"), self.value("upper"))
            speed = math.radians(self.value("speed"))*self.value("ratio")
            self.conversion.setText("目标 %.4f rad · 速度上限 %.4f rad/s\n额外传动比 %.3f" %
                                    (rad, speed, self.value("ratio")))
        except ValueError as exc:
            self.conversion.setText(str(exc))

    def _idle(self):
        if self.active or self.flow != "idle" or self.drive.pending:
            raise ValueError("运行中不能修改示教或配置，请先停止")

    def teach(self, key):
        try:
            self._idle()
            v, _ = self.snapshot()
            if abs(math.degrees(v[14])/self.value("ratio")) > self.value("velocity"):
                raise ValueError("先停稳再记录实际位置")
            self.preset_inputs[key].setValue(math.degrees(v[13])/self.value("ratio"))
            self.store(key)
        except ValueError as exc:
            self.say(str(exc))

    def store(self, key):
        try:
            self._idle()
            angle = self.preset_inputs[key].value()
            validate_target(angle, self.value("ratio"), self.value("lower"), self.value("upper"))
            motor_id = int(self.owner.dm_rows["DMID"].spin.value())
            if self.positions and self.profile_id != motor_id:
                raise ValueError("文件属于其他电机 ID，请备份并移走示教文件后重新示教")
            candidate = self.profile()
            candidate["motor_id"] = motor_id
            candidate["positions"][key] = angle
            save_profile(self.path, candidate)
            self.positions, self.profile_id = candidate["positions"], motor_id
            self.preset_status[key].setText("已保存 %.3f°" % angle)
            self.say(POSITIONS[key] + "已保存；不会自动运动")
        except (ValueError, OSError) as exc:
            self.say(str(exc))

    def check_preset(self, key):
        if (not self.reference.isChecked()
                or int(self.owner.dm_rows["DMID"].spin.value()) != self.profile_id):
            raise ValueError("先确认电机 ID、零点和传动比与示教一致")
        if key not in self.positions:
            raise ValueError(POSITIONS[key] + "尚未示教保存")
        return self.positions[key]

    def goto_preset(self, key):
        try:
            self.move(self.check_preset(key))
        except ValueError as exc:
            self.say(str(exc))

    def start_flow(self):
        try:
            self._idle()
            key = self.slot.currentData()
            pickup, target = self.check_preset("pickup"), self.check_preset(key)
            v, _ = self.snapshot(True)
            if (abs(math.degrees(v[13])/self.value("ratio")-pickup) > self.value("tolerance")
                    or abs(math.degrees(v[14])/self.value("ratio")) > self.value("velocity")):
                raise ValueError("先前往待抓取位并停稳，再抓取、抬升和确认")
            if self.move(target, "slot"):
                self.flow, self.flow_slot = "to_slot", key
                self._lock(True)
        except ValueError as exc:
            self.say(str(exc))

    def return_flow(self):
        try:
            if self.flow != "await_release":
                raise ValueError("载盘尚未到位停稳，不能确认放料返回")
            target = self.check_preset("pickup")
            v, _ = self.snapshot(True)
            slot = self.check_preset(self.flow_slot)
            if (abs(math.degrees(v[13])/self.value("ratio")-slot) > self.value("tolerance")
                    or abs(math.degrees(v[14])/self.value("ratio")) > self.value("velocity")):
                raise ValueError("载盘位置已偏离或仍在转动，请停止并重新检查")
            if self.move(target, "pickup"):
                self.flow = "to_pickup"
        except ValueError as exc:
            self.say(str(exc))

    def profile(self):
        settings = {key: spin.value() for key, spin in self.inputs.items() if key != "target"}
        settings["smooth"] = self.smooth.isChecked()
        return dict(version=1, motor_id=self.profile_id, settings=settings,
                    positions=dict(self.positions))

    def save(self):
        try:
            self._idle()
            save_profile(self.path, self.profile())
            self.say("调试设置和示教位置已保存到电脑")
        except (ValueError, OSError) as exc:
            self.say(str(exc))

    def load(self):
        try:
            self._idle()
            if not self.path.exists():
                return
            data = load_profile(self.path)
            for key, value in data["settings"].items():
                if key in self.inputs:
                    self.inputs[key].setValue(value)
            self.smooth.setChecked(data["settings"]["smooth"])
            self.positions, self.profile_id = data["positions"], data["motor_id"]
            for key in POSITIONS:
                if key in self.positions:
                    self.preset_inputs[key].setValue(self.positions[key])
                    self.preset_status[key].setText("已保存 %.3f°" % self.positions[key])
                else:
                    self.preset_status[key].setText("未示教")
            self.invalidate_reference()
            self.say("已加载电机 %d 的示教；确认当前零点一致后可使用" % self.profile_id)
        except (ValueError, OSError, KeyError, TypeError, AttributeError) as exc:
            self.say("示教文件无法加载：" + str(exc))
