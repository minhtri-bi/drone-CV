"""One-run offline provenance driver; never initializes ROS or a simulator."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

root = Path('/workspace')
out = root/'round2-planes'
out.mkdir(exist_ok=False)
script = root/'spike/freespace/replay_semantics.py'
args = [str(script), '--recording', str(root/'support06'), '--mode', 'planes',
        '--output', str(out/'planes.json')]
guard = '''import runpy, sys
import rclpy
from rclpy.node import Node
def forbidden(*args, **kwargs):
    raise RuntimeError("OFFLINE_GUARD: ROS/control initialization forbidden")
rclpy.init = rclpy.create_node = Node.create_publisher = forbidden
sys.path[:0] = ["/workspace/src", "/workspace/spike/freespace"]
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name="__main__")
'''
argv = [sys.executable, '-c', guard, *args]
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
source_hashes = {str(path.relative_to(root)): sha(path)
                 for folder in ('src', 'spike/freespace')
                 for path in sorted((root/folder).rglob('*.py'))}
input_hashes = {str(path.relative_to(root/'support06')): sha(path)
                for path in sorted((root/'support06').rglob('*')) if path.is_file()}
start = time.time_ns()
with (out/'stdout.txt').open('x') as stdout, (out/'stderr.txt').open('x') as stderr:
    child = subprocess.Popen(argv, cwd=root, stdout=stdout, stderr=stderr, start_new_session=True)
    timed_out = False
    try:
        code = child.wait(timeout=600)
    except subprocess.TimeoutExpired:
        timed_out = True
        os.killpg(child.pid, signal.SIGTERM)
        try:
            code = child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            code = child.wait(timeout=5)
report = out/'planes.json'
provenance = {
    'scope': 'OFFLINE_PINNED_EXISTING_RECORDING_DIAGNOSTIC_NOT_ACCEPTANCE',
    'image': 'sha256:061a39905d2d84d40859bb46b67048241b6dacf2ed3983889da7f5fd3e718285',
    'command': argv, 'source_sha256': source_hashes, 'input_sha256': input_hashes,
    'started_unix_ns': start, 'finished_unix_ns': time.time_ns(),
    'child_pid': child.pid, 'exit_code': code, 'process_exit_observed': child.poll() is not None,
    'deadline_s': 600, 'deadline_exceeded': timed_out,
    'owned_child_reaped': child.poll() is not None,
    'report_sha256': sha(report) if report.is_file() else None,
    'stdout_sha256': sha(out/'stdout.txt'), 'stderr_sha256': sha(out/'stderr.txt'),
    'driver_sha256': sha(Path(__file__)),
    'prior_unobserved_report_sha256': '04f69883f90a870a866ac6e9173ef09119819bcb9faf6d5fbbfb9b87628a3a7d',
    'prior_evidence_overwritten': False,
    'guard': 'ROS init, create_node and Node.create_publisher raise before replay imports',
    'operational_authority': False,
}
(out/'provenance.json').write_text(json.dumps(provenance, indent=2, allow_nan=False)+'\n')
print(json.dumps({key: provenance[key] for key in ('exit_code', 'process_exit_observed',
      'deadline_exceeded', 'owned_child_reaped', 'report_sha256')}), flush=True)
raise SystemExit(124 if timed_out else code)
