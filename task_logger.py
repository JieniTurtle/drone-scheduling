import json
import os
from datetime import datetime


def _fmt_ts(ts):
    """将 Unix 时间戳格式化为可读字符串，None 时返回 None"""
    if ts is None:
        return None
    return datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


def _round_sec(val, ndigits=3):
    """保留 3 位小数，None 时返回 None"""
    return round(val, ndigits) if val is not None else None


class TaskLogger:
    """
    将已完成任务的信息与时延以 JSONL 格式追加写入文件。
    后续如需保存更多字段，只需在 _build_record() 中为返回的 dict 添加新键即可。
    """

    def __init__(self, file_path='task_log.jsonl'):
        self.file_path = file_path

    def _build_record(self, task):
        """
        从 Task 对象构建一条记录 dict。
        需要新增字段时，直接在此方法的 record 中添加新键。
        """
        latency = task.get_latency_info()
        src = task.source if task.source else (None, None)
        dst = task.destination if task.destination else (None, None)

        record = {
            'task_id':          task.task_id,
            'weight':           task.weight,
            'source':           {'x': src[0], 'y': src[1]},
            'destination':      {'x': dst[0], 'y': dst[1]},
            'status':           task.status,
            'timestamps': {
                'created_at':   _fmt_ts(task.created_at),
                'assigned_at':  _fmt_ts(task.assigned_at),
                'completed_at': _fmt_ts(task.completed_at),
            },
            'latency_s': {
                'wait_time':      _round_sec(latency['wait_time']),
                'execution_time': _round_sec(latency['execution_time']),
                'total_time':     _round_sec(latency['total_time']),
            },
        }
        return record

    def log_task(self, task, extra_fields=None):
        """
        将单条任务追加写入 JSONL 文件（一行一条记录）。

        :param task:         Task 实例（应已调用 mark_completed()）
        :param extra_fields: dict，可选，合并到记录的顶层
        """
        record = self._build_record(task)
        if extra_fields:
            record.update(extra_fields)

        with open(self.file_path, 'a', encoding='utf-8') as f:
            f.write(json.dumps(record, ensure_ascii=False) + '\n')

    def log_tasks(self, tasks, extra_fields=None):
        """批量写入多条任务"""
        for task in tasks:
            self.log_task(task, extra_fields)
