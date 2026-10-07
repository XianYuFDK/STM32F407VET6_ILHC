"""串口接收端自动运行记录；后台落盘，日志故障不接管运动控制。"""
import copy
import csv
import datetime
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
import uuid


def clean(value):
    """JSON严格保存非有限值为null，原帧仍保留通道位置。"""
    if callable(getattr(value,'as_dict',None)):
        return clean(value.as_dict())
    if isinstance(value, dict):
        return {str(k): clean(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class RunJournal:
    def __init__(self, root, channels, *, capacity=8192):
        self.root = Path(os.environ.get('ILHC_RUN_LOG_DIR') or root)
        self.channels = copy.deepcopy(channels)
        self.enabled = True
        self.capacity = capacity
        self.path = None
        self.error = ''
        self.dropped = 0
        self._lock = threading.RLock()
        self._queue = None
        self._thread = None
        self._run = None
        self.active_kind = None
        self._sequence = 0
        self._stop = None
        self._tail = None
        self._writers = []
        self._source = None
        self._metadata = None
        self._prepared = False

    @property
    def active_run(self):
        return self._run

    def start(self, source, metadata):
        """绑定数据源；待机不创建文件，首次运行才启动写盘。"""
        self.close()
        with self._lock:
            self._source = source
            self._metadata = copy.deepcopy(metadata)
            self.error = '';self.dropped = 0;self._sequence = 0
            self._prepare_path()
            return self.path

    def _prepare_path(self):
        stamp = datetime.datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        self.path = self.root/(stamp+'_'+self._source+'_'+uuid.uuid4().hex[:8])
        self._prepared = True

    def _start_writer(self, metadata):
        if not self._prepared:
            self._prepare_path()
        self._prepared = False
        self._queue = queue.Queue(self.capacity)
        self._stop = threading.Event()
        self._tail = []
        self._sequence = 0
        header = dict(schema_version=1, source=self._source, physical_data=self._source=='REAL',
            recording_scope='single_run',
            wallclock=datetime.datetime.now().astimezone().isoformat(), channels=self.channels,
            units=dict(position='OPS cm', angle='degrees', monotonic='host seconds', device_tick='MCU milliseconds'),
            firmware_feedback_limits='24 channels include wheel1 target, not all wheel encoder feedback',
            metadata=copy.deepcopy(metadata))
        self._thread = threading.Thread(target=self._write, args=(self.path,header,self._queue,self._stop,self._tail),
                                        daemon=True, name='ILHC-run-log')
        self._writers = [thread for thread in self._writers if thread.is_alive()]
        self._writers.append(self._thread)
        self._thread.start()

    def _record(self, event, monotonic=None, **data):
        self._sequence += 1
        return dict(event=event, sequence=self._sequence, run_id=self._run,
            monotonic=time.monotonic() if monotonic is None else monotonic,
            wall_time=time.time(), **data)

    def emit(self, event, monotonic=None, **data):
        with self._lock:
            if self._queue is None:
                return False
            record = self._record(event, monotonic, **data)
            try:
                self._queue.put_nowait(('event',record))
                return True
            except queue.Full:
                self.dropped += 1
                return False

    def set_enabled(self, enabled):
        with self._lock:
            self.enabled = bool(enabled)
            if not self.enabled:
                self.end_run('RECORDING_DISABLED', '用户关闭日志记录')

    def begin_run(self, kind, metadata, plan=None):
        with self._lock:
            if not self.enabled or self._source is None:
                return None
            self.end_run('SUPERSEDED', '新运行接管')
            self._start_writer(metadata or self._metadata)
            self._run = uuid.uuid4().hex
            self.active_kind = kind
            self.emit('RUN_START', kind=kind, metadata=copy.deepcopy(metadata))
            if plan is not None:
                try:
                    self._queue.put_nowait(('plan',(self._run,copy.deepcopy(plan))))
                except queue.Full:
                    self.dropped += 1
            return self._run

    def end_run(self, status, reason='', **details):
        with self._lock:
            if self._run is not None:
                # 终止记录独立于有界遥测队列，满队列也必须保存结果。
                self._tail.append(('event',self._record('RUN_END',status=status,reason=reason,**copy.deepcopy(details))))
                self._run = None
                self.active_kind = None
                self._tail.append(('event',self._record('SESSION_END',dropped_events=self.dropped)))
                # 立即切断新事件；旧队列异步排空并关闭文件，不阻塞串口/运动线程。
                self._queue = None
                self._stop.set()

    def close(self):
        with self._lock:
            self.end_run('SESSION_CLOSED', '连接或窗口关闭')
            self._source = self._metadata = None
            writers = list(self._writers)
        # 仅断连/窗口退出有界等待；每次运行结束不等待写盘。
        deadline = time.monotonic()+2
        for thread in writers:
            thread.join(timeout=max(0,deadline-time.monotonic()))
        if any(thread.is_alive() for thread in writers):
            self.error = '日志磁盘尚未完成写入'

    def _write(self, folder, header, q, stop, tail):
        files = [];runs = {};last_flush = time.monotonic()
        try:
            folder.mkdir(parents=True,exist_ok=False)
            (folder/'session.json').write_text(json.dumps(clean(header),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
            events = (folder/'events.jsonl').open('w',encoding='utf-8');files.append(events)
            telemetry = (folder/'telemetry.csv').open('w',newline='',encoding='utf-8-sig');files.append(telemetry)
            columns = ['host_monotonic_s','host_wall_s','device_sequence','device_tick_ms','protocol','run_id']+[c[1] for c in self.channels]
            writer=csv.writer(telemetry);writer.writerow(columns)
            while True:
                try:
                    kind,value = q.get(timeout=0 if stop.is_set() else .25)
                except queue.Empty:
                    if stop.is_set():
                        if not tail:break
                        kind,value = tail.pop(0)
                    else:
                        kind,value = 'flush',None
                if kind=='close':
                    break
                if kind=='plan':
                    run_id,plan=value
                    path=folder/'runs'/run_id;path.mkdir(parents=True,exist_ok=True)
                    (path/'plan.json').write_text(json.dumps(clean(plan),ensure_ascii=False,allow_nan=False),encoding='utf-8')
                elif kind=='event':
                    value=clean(value)
                    events.write(json.dumps(value,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')
                    run_id=value['run_id']
                    if value['event']=='RUN_START':
                        path=folder/'runs'/run_id;path.mkdir(parents=True,exist_ok=True)
                        (path/'metadata.json').write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
                        f=(path/'telemetry.csv').open('w',newline='',encoding='utf-8-sig');files.append(f)
                        runs[run_id]=(f,csv.writer(f));runs[run_id][1].writerow(columns)
                    if value['event']=='FRAME':
                        row=[value['monotonic'],value['wall_time'],value.get('device_sequence'),
                             value.get('device_tick_ms'),value.get('protocol'),run_id,*value['values']]
                        writer.writerow(row)
                        if run_id in runs:runs[run_id][1].writerow(row)
                    if value['event']=='RUN_END' and run_id in runs:
                        path=folder/'runs'/run_id
                        (path/'result.json').write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
                        f=runs.pop(run_id)[0];f.close();files.remove(f)
                if time.monotonic()-last_flush>=1:
                    for f in files:f.flush()
                    last_flush=time.monotonic()
        except Exception as exc:
            self.error = str(exc)
            # 消费至关闭，避免磁盘失败后队列永久阻塞控制线程。
            while True:
                try:
                    if q.get(timeout=.25)[0]=='close':break
                except queue.Empty:
                    if stop.is_set():break
        finally:
            for f in files:
                try:f.close()
                except OSError:pass
