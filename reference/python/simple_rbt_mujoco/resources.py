"""Linux process accounting and on-disk size, without third-party monitors."""
import os
from pathlib import Path
import resource
import time


def cpu_seconds():
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return usage.ru_utime + usage.ru_stime


class ResourceMeter:
    def __init__(self):
        self.wall = time.perf_counter()
        self.cpu = cpu_seconds()

    def report(self):
        ended = time.perf_counter()
        elapsed = ended - self.wall
        consumed = cpu_seconds() - self.cpu
        cores = consumed / elapsed if elapsed else 0.
        return {
            "wall_seconds": elapsed,
            "interval_start_monotonic_s": self.wall,
            "interval_end_monotonic_s": ended,
            "process_cpu_seconds": consumed,
            "mean_cpu_cores": cores,
            "mean_cpu_percent_one_core": 100. * cores,
            "host_logical_cpus": os.cpu_count(),
            "mean_cpu_percent_machine": 100. * cores / (os.cpu_count() or 1),
            "process_peak_rss_mib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.,
            "peak_rss_scope": "whole process high-water mark, including imports and initialization",
            "cpu_scope": "this process within the measured interval; GPU time is separate",
        }


def directory_size(path):
    """Count actual regular files, not symlink targets; de-duplicate hardlinks."""
    logical = allocated = files = 0
    seen = set()
    for current, dirs, names in os.walk(Path(path), followlinks=False):
        dirs[:] = [d for d in dirs if not (Path(current) / d).is_symlink()]
        for name in names:
            p = Path(current) / name
            if p.is_symlink():
                continue
            st = p.stat()
            key = (st.st_dev, st.st_ino)
            if key in seen:
                continue
            seen.add(key)
            files += 1
            logical += st.st_size
            allocated += st.st_blocks * 512
    return {"files": files, "logical_bytes": logical, "allocated_bytes": allocated}
