# v2.1.22 日志开关、作业点、回读输入与车辆暂停

## 使用

- 比赛地图「记录运行日志」同时控制实机与模拟。默认勾选，取消立即结束当前运行日志（RECORDING_DISABLED），后续运行不建目录；重新勾选后记录下一次运行。暂停不结束日志，DONE/STOP仍结束。顶部「记录」手动CSV按钮独立。
- 展开「作业点坐标设置」，填写二维码、原料、粗加工、暂存的场地 X左/Y前，单位cm。默认分别(105,7)、(10,105)、(190,105)、(110,190)。应用后取消旧规划/运行；三站保留平行设备边缘的作业航向，更新站点、匹配lane_node、标定元数据。新站点按当前车体与障碍校验，QR入口仍由路径规划完整复检。没有改变点位时保留路线。
- 「保存当前地图…」保存四点和当前地图；默认文件 navigation_map.json 供下次启动使用，自定义文件可通过加载地图打开。保存采用临时文件替换。设备轮廓不会随停车点挪动。
- 七项底盘参数输入等待回读，收到后同步数值与滑块。未发送编辑不被周期遥测覆盖；「重新回读并填入」放弃编辑、重新获取实际值。未知参数禁止发送，重连清除旧设备读数。OPS安装偏移仍是无回读的独立输入，不伪称已读取。
- 「暂停车辆」用于正在执行的实机整批轨迹、模拟点到点路径及比赛；确认后变为「继续车辆」。保留批次、目标索引、轨迹进度。顶部原暂停已改名「暂停波形」。STOP取消任务，不能继续；手动遥控/单独GOTO不属于轨迹暂停。

## STM32协议

CCAPS9，新增 TPAUSE=id、TCONTINUE=id，TSTAT状态9=PAUSED；原 TRESUME=id,index 仍只表示站点作业完成。PC等待控制端确认后显示已暂停，3s无确认或状态超时发TABORT/STOP。旧CCAPS8仍可运行原有路径，暂停按钮明确提示升级。

TPAUSE高优先级发送。控制周期停止输出并保留点表；暂停时间不占180s运行期限，不累加停稳时间。继续重置速度/测量时间基准，从零速度重新控制。暂停期间保留OPS有效性、主机心跳、参数/地图、使能保护；偏离暂停位置超过50mm或航向15度故障退出。不要把可继续理解为允许随意搬动车辆。日志包含USER_MOTION_PAUSE请求与BATCH_STATE确认。

## 验证与产物

- EIDE构建成功：0 error、0 warning；HEX：MDK-ARM/build/STM32F407VET6_ILHC/STM32F407VET6_ILHC.hex。构建日志 RTOS_APP/validation/user-controls-20261007。
- tests/test_user_controls + test_run_journal + UploaderTests：23通过（含SIM位置/进度/作业时钟暂停，日志关闭零运行文件，坐标校验/保存，真实回读与编辑保护）。user_controls_targeted_tests_20261007.log。
- 原Qt实机控制29测试通过，串口均为替身、日志隔离到临时目录。
- Tests/hardware/test_user_pause.py：4通过，真实C引擎+电机模型：暂停190s再完成同一批次，OPS过期/主机失效/位姿跳变仍停车，STOP不能恢复、CCAPS8拒绝、确认超时。
- 新能力版本断言更新为CCAPS9后test_protocol_crc_and_parameters通过。真实C桥接STOP/心跳/快照测试通过。恢复测试连同继承用例25项首次24通过、1项旧CCAPS8断言失败；修正该版本断言后单项复跑通过，未重复整组。
- 扩展运行旧稠密轨迹整场测试发现既有入口compile_match(coordinate_mode=False)在原料区圆盘处拒绝旋转：legacy全场8子用例、MatchPackingTests.setUpClass未通过。该拒绝发生在未修改的旧路径构建分支，未放宽碰撞保护，不能宣称全仓测试通过。当前上位机使用coordinate模式。
- audit_user_controls_20261007.py：两地图×三模式×两组参数12轮完整模拟及量化编码通过。固定缓存72路段SHA验证，新manifest已发布，旧清单备份user_controls_previous_manifest_20261007.json。缓存算法0108623dd6088177411af5b606f6696e9602950b7bd7d20438fb3ea57778fcc9。
- 本机磁盘缓存整场约0.05–0.15s，未在Jetson Nano测时。

没有打开实车串口、烧录、移动小车或修改实测站点默认数值。本功能不修复另案USART2 FE/NE电气错误或OPS故障竞态。
