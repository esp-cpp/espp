import sys
import time
import threading

import espp

# ---------------------------------------------------------------------------
# Basic usage: submit jobs, wait for completion, read stats
# ---------------------------------------------------------------------------
start = time.time()
def elapsed():
    return time.time() - start

counter_lock = threading.Lock()
completed = [0]
done = threading.Event()
TOTAL_JOBS = 8

pool = espp.ThreadPool(espp.ThreadPool.Config(
    worker_count=2,
    max_queue_size=0,
    auto_start=True,
    log_level=espp.Logger.Verbosity.warn,
))

print(f"[{elapsed():.3f}] Pool started, worker_count={pool.worker_count()}, is_running={pool.is_running()}")

for i in range(TOTAL_JOBS):
    def _job(i=i):
        time.sleep(0.2)
        with counter_lock:
            completed[0] += 1
            print(f"[{elapsed():.3f}] Job {i} done ({completed[0]}/{TOTAL_JOBS})")
            if completed[0] >= TOTAL_JOBS:
                done.set()
    pool.submit(_job)

print(f"[{elapsed():.3f}] All {TOTAL_JOBS} jobs submitted, queue_size={pool.queue_size()}")

done.wait()
s = pool.stats()
print(f"[{elapsed():.3f}] All jobs complete — submitted={s.submitted} executed={s.executed} rejected={s.rejected}")

pool.stop()
print(f"[{elapsed():.3f}] Pool stopped, is_running={pool.is_running()}")

sys.exit(0)
