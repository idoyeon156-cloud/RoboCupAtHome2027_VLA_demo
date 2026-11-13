"""Local JSONL diagnostics. Never publish commands or change control state."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time


class Recorder:
    def __init__(self, node):
        self.node = node
        self.failed = False
        self.sequence = 0
        root = Path(os.environ.get('ROS_LOG_DIR', str(Path.home()/'.ros/log')))
        run_id = os.environ.get('CEREAL_RUN_ID', 'unassigned')
        self.directory = root/'diagnostics'/run_id
        self.stream = None
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            path = self.directory/(node.get_name()+'.jsonl')
            self.stream = path.open('a', encoding='utf-8', buffering=1)
            node.get_logger().info('Detailed diagnostics: '+str(path))
        except OSError as error:
            self.error(error)

    def error(self, error):
        if not self.failed:
            self.node.get_logger().error('DIAGNOSTIC_WRITE_FAILED: '+str(error))
        self.failed = True

    def event(self, kind, data):
        if self.stream is None or self.failed:
            return
        try:
            self.sequence += 1
            row = dict(time_utc=datetime.now(timezone.utc).isoformat(),
                       ros_time_ns=self.node.get_clock().now().nanoseconds,
                       monotonic_ns=time.monotonic_ns(), node=self.node.get_name(),
                       pid=os.getpid(), sequence=self.sequence, event=kind, data=data)
            self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
            self.stream.flush()
        except (OSError, ValueError, TypeError) as error:
            self.error(error)
