"""ThreadPool Python test.

Mirrors the 9 test sections of the C++ thread_pool_example.cpp, exercising
the full espp.ThreadPool Python binding API.

Exit code 0 on full pass, 1 on any failure.
"""

import sys
import threading
import time
from typing import List, Tuple

import espp

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

results: List[Tuple[str, bool]] = []


def check(test: str, condition: bool, desc: str) -> bool:
    if condition:
        print(f"  PASS [{test}]: {desc}")
    else:
        print(f"  FAIL [{test}]: {desc}")
    return condition


def wait_for_jobs(event: threading.Event, counter, expected: int, timeout: float = 5.0) -> None:
    event.wait(timeout=timeout)


# ---------------------------------------------------------------------------
# 1. Lifecycle: start / stop / is_running / worker_count
# ---------------------------------------------------------------------------
name = "lifecycle: start/stop/is_running/worker_count"
print(f"--- {name} ---")
passed = True

pool = espp.ThreadPool(espp.ThreadPool.Config(worker_count=3, auto_start=False))
passed &= check(name, not pool.is_running(),       "pool should not be running before start()")
passed &= check(name, pool.worker_count() == 3,    "worker_count() should be 3")
passed &= check(name, pool.start(),                "start() should return True on first call")
passed &= check(name, pool.is_running(),           "pool should be running after start()")
passed &= check(name, pool.start(),                "start() should return True when already running (no-op)")
passed &= check(name, pool.is_running(),           "pool should still be running after duplicate start()")
pool.stop()
passed &= check(name, not pool.is_running(),       "pool should not be running after stop()")
results.append((name, passed))

# ---------------------------------------------------------------------------
# 2. submit() + queue_size() + stats()
# ---------------------------------------------------------------------------
name = "submit: normal dispatch + queue_size + stats"
print(f"--- {name} ---")
passed = True

TOTAL_JOBS = 8
counter = [0]
counter_lock = threading.Lock()
all_done = threading.Event()

pool = espp.ThreadPool(espp.ThreadPool.Config(worker_count=2))

accepted = 0
for i in range(TOTAL_JOBS):
    def _job(i=i):
        time.sleep(0.05)
        with counter_lock:
            counter[0] += 1
            if counter[0] >= TOTAL_JOBS:
                all_done.set()
    if pool.submit(_job):
        accepted += 1

passed &= check(name, accepted == TOTAL_JOBS, "all jobs should be accepted (unbounded queue)")
all_done.wait(timeout=10.0)
s = pool.stats()
print(f"  stats: {s.submitted} submitted, {s.executed} executed, {s.rejected} rejected")
passed &= check(name, s.submitted == TOTAL_JOBS, "submitted count should equal total_jobs")
passed &= check(name, s.executed == TOTAL_JOBS,  "executed count should equal total_jobs")
passed &= check(name, s.rejected == 0,           "rejected count should be 0")
passed &= check(name, pool.queue_size() == 0,    "queue should be empty after all jobs finish")
pool.stop()
results.append((name, passed))

# ---------------------------------------------------------------------------
# 3. try_submit() — deterministic rejection when queue is full
# ---------------------------------------------------------------------------
name = "try_submit: rejection when queue full"
print(f"--- {name} ---")
passed = True

# 1 worker, capacity 2: 1 executing + 2 queued = 3 slots total.
# Gate workers with a barrier so the queue stays full during the rejection check.
# Track ALL 3 fill-job completions before calling pool.stop() to guarantee
# no worker thread is executing Python code when stop() joins the thread.
barrier = threading.Event()
first_started = threading.Event()
all_fill_done = threading.Event()
fill_done_count = [0]
fill_done_lock = threading.Lock()
TOTAL_FILL = 3

def _make_fill_job(signal_start: bool = False):
    def _job():
        if signal_start:
            first_started.set()
        barrier.wait()
        with fill_done_lock:
            fill_done_count[0] += 1
            if fill_done_count[0] >= TOTAL_FILL:
                all_fill_done.set()
    return _job

pool = espp.ThreadPool(espp.ThreadPool.Config(worker_count=1, max_queue_size=2))

# Submit first job and wait until it is executing (off the queue)
pool.try_submit(_make_fill_job(signal_start=True))
first_started.wait(timeout=2.0)

# Fill the 2-slot queue
fill_accepted = 1  # first job counted above
for _ in range(2):
    if pool.try_submit(_make_fill_job()):
        fill_accepted += 1

passed &= check(name, fill_accepted == 3, "first 3 try_submit calls should be accepted")

# Queue is now provably full
rejected_count = sum(1 for _ in range(3) if not pool.try_submit(lambda: None))
passed &= check(name, rejected_count == 3, "try_submit when full should return False")
s = pool.stats()
print(f"  stats: {s.submitted} submitted, {s.executed} executed, {s.rejected} rejected")
passed &= check(name, s.rejected == 3, "stats.rejected should be 3")

barrier.set()
# Wait for ALL 3 fill jobs to finish their Python callbacks before calling
# stop(). Waiting only for the first job is not enough — after it finishes,
# the worker immediately picks up the queued jobs and needs the GIL to execute
# them. stop() would deadlock if called while those callbacks are running.
all_fill_done.wait(timeout=5.0)
pool.stop()
results.append((name, passed))

# ---------------------------------------------------------------------------
# 4. Queue backpressure: all jobs accepted despite bounded queue
# ---------------------------------------------------------------------------
# NOTE: ThreadPool.submit() is bound with a GIL-release guard, so blocking submit does not
# inherently deadlock Python worker callbacks due to the GIL. This section uses a try_submit()
# + sleep-retry loop to apply backpressure without blocking the calling thread.
# backpressure semantics safely.
name = "submit: queue backpressure (try_submit retry)"
print(f"--- {name} ---")
passed = True

TOTAL_JOBS = 6
counter2 = [0]
counter2_lock = threading.Lock()
all_done2 = threading.Event()

pool = espp.ThreadPool(espp.ThreadPool.Config(
    worker_count=1,
    max_queue_size=2,
    block_on_submit_when_full=False,
))

accepted2 = 0
for i in range(TOTAL_JOBS):
    def _job2(i=i):
        time.sleep(0.03)
        with counter2_lock:
            counter2[0] += 1
            if counter2[0] >= TOTAL_JOBS:
                all_done2.set()
    # Retry until accepted, releasing the GIL on each sleep so workers can run
    while not pool.try_submit(_job2):
        time.sleep(0.001)
    accepted2 += 1

passed &= check(name, accepted2 == TOTAL_JOBS, "all jobs should be accepted (try_submit retry)")
all_done2.wait(timeout=10.0)
s = pool.stats()
print(f"  stats: {s.submitted} submitted, {s.executed} executed, {s.rejected} rejected")
passed &= check(name, s.submitted == TOTAL_JOBS, "submitted count should equal total_jobs")
passed &= check(name, s.executed == TOTAL_JOBS,  "executed count should equal total_jobs")
passed &= check(name, s.rejected > 0,            "rejected reflects try_submit retries (expected > 0)")
pool.stop()
results.append((name, passed))

# ---------------------------------------------------------------------------
# 5. submit() after stop() — rejected via is_running() guard
# ---------------------------------------------------------------------------
name = "submit: rejected after stop()"
print(f"--- {name} ---")
passed = True

pool = espp.ThreadPool(espp.ThreadPool.Config(worker_count=1))
pool.stop()
accepted_after_stop = pool.submit(lambda: None)
passed &= check(name, not accepted_after_stop,       "submit() after stop() should return False")
passed &= check(name, pool.stats().submitted == 0,   "submitted count should be 0")
passed &= check(name, pool.stats().rejected == 1,    "rejected count should be 1")
s = pool.stats()
print(f"  stats: {s.submitted} submitted, {s.executed} executed, {s.rejected} rejected")
results.append((name, passed))

# ---------------------------------------------------------------------------
# 6. Concurrent start/stop from multiple threads
# ---------------------------------------------------------------------------
name = "concurrent: start/stop from multiple threads"
print(f"--- {name} ---")
passed = True

pool = espp.ThreadPool(espp.ThreadPool.Config(worker_count=2, auto_start=False))

NUM_THREADS = 4
ITERATIONS = 10

def _lifecycle_thread(t: int) -> None:
    for i in range(ITERATIONS):
        if (t + i) % 2 == 0:
            pool.start()
        else:
            pool.stop()

threads = [threading.Thread(target=_lifecycle_thread, args=(t,)) for t in range(NUM_THREADS)]
for t in threads:
    t.start()
for t in threads:
    t.join()

pool.stop()
passed &= check(name, not pool.is_running(), "pool should reach a clean stopped state")
s = pool.stats()
print(f"  stats: {s.submitted} submitted, {s.executed} executed, {s.rejected} rejected")
passed &= check(name, s.submitted == 0 and s.executed == 0 and s.rejected == 0,
                "stats should all be zero (no jobs submitted)")
results.append((name, passed))

# ---------------------------------------------------------------------------
# 7. Concurrent submit and try_submit from multiple producer threads
# ---------------------------------------------------------------------------
name = "concurrent: multi-thread submit and try_submit"
print(f"--- {name} ---")
passed = True

NUM_SUBMIT_THREADS = 3
NUM_TRY_SUBMIT_THREADS = 2
JOBS_PER_THREAD = 10
TOTAL = (NUM_SUBMIT_THREADS + NUM_TRY_SUBMIT_THREADS) * JOBS_PER_THREAD

c7 = [0]
c7_lock = threading.Lock()
c7_done = threading.Event()
total_accepted7 = [0]
ta7_lock = threading.Lock()

pool = espp.ThreadPool(espp.ThreadPool.Config(worker_count=4))

def _submit_producer():
    for _ in range(JOBS_PER_THREAD):
        def _w():
            time.sleep(0.01)
            with c7_lock:
                c7[0] += 1
                if c7[0] >= TOTAL:
                    c7_done.set()
        if pool.submit(_w):
            with ta7_lock:
                total_accepted7[0] += 1

def _try_submit_producer():
    for _ in range(JOBS_PER_THREAD):
        def _w():
            time.sleep(0.01)
            with c7_lock:
                c7[0] += 1
                if c7[0] >= TOTAL:
                    c7_done.set()
        if pool.try_submit(_w):
            with ta7_lock:
                total_accepted7[0] += 1

producers = (
    [threading.Thread(target=_submit_producer) for _ in range(NUM_SUBMIT_THREADS)] +
    [threading.Thread(target=_try_submit_producer) for _ in range(NUM_TRY_SUBMIT_THREADS)]
)
for p in producers:
    p.start()
for p in producers:
    p.join()

c7_done.wait(timeout=10.0)
s = pool.stats()
print(f"  stats: {s.submitted} submitted, {s.executed} executed, {s.rejected} rejected")
passed &= check(name, s.submitted + s.rejected == TOTAL,
                "submitted + rejected should equal total attempted")
passed &= check(name, s.executed == s.submitted,
                "all accepted jobs should be executed (unbounded queue)")
passed &= check(name, s.rejected == 0, "unbounded queue should not reject any jobs")
pool.stop()
results.append((name, passed))

# ---------------------------------------------------------------------------
# 8. Chained pools: a job in pool_a submits work to pool_b
# ---------------------------------------------------------------------------
name = "chained: job in pool_a submits to pool_b"
print(f"--- {name} ---")
passed = True

NUM_A_JOBS = 5
B_JOBS_PER_A = 2
TOTAL_B = NUM_A_JOBS * B_JOBS_PER_A

c8 = [0]
c8_lock = threading.Lock()
c8_done = threading.Event()

pool_b = espp.ThreadPool(espp.ThreadPool.Config(worker_count=2))
pool_a = espp.ThreadPool(espp.ThreadPool.Config(worker_count=2))

for _ in range(NUM_A_JOBS):
    def _a_job():
        for _ in range(B_JOBS_PER_A):
            def _b_job():
                time.sleep(0.02)
                with c8_lock:
                    c8[0] += 1
                    if c8[0] >= TOTAL_B:
                        c8_done.set()
            pool_b.submit(_b_job)
    pool_a.submit(_a_job)

c8_done.wait(timeout=10.0)
sa = pool_a.stats()
sb = pool_b.stats()
print(f"  pool_a stats: {sa.submitted} submitted, {sa.executed} executed, {sa.rejected} rejected")
print(f"  pool_b stats: {sb.submitted} submitted, {sb.executed} executed, {sb.rejected} rejected")
passed &= check(name, sa.executed == NUM_A_JOBS,  "pool_a should execute all A jobs")
passed &= check(name, sb.executed == TOTAL_B,     "pool_b should execute all chained B jobs")
passed &= check(name, sb.rejected == 0,           "pool_b should not reject any jobs")
pool_a.stop()
pool_b.stop()
results.append((name, passed))

# ---------------------------------------------------------------------------
# 9. Self-submit: a job submits another job back to the same pool
# ---------------------------------------------------------------------------
name = "self-submit: job submits to its own pool"
print(f"--- {name} ---")
passed = True

NUM_INITIAL = 4
TOTAL_EXEC = NUM_INITIAL * 2  # each initial job spawns one follow-up

c9 = [0]
c9_lock = threading.Lock()
c9_done = threading.Event()

pool = espp.ThreadPool(espp.ThreadPool.Config(worker_count=2))

for _ in range(NUM_INITIAL):
    def _initial():
        with c9_lock:
            c9[0] += 1
            if c9[0] >= TOTAL_EXEC:
                c9_done.set()
        def _followup():
            time.sleep(0.01)
            with c9_lock:
                c9[0] += 1
                if c9[0] >= TOTAL_EXEC:
                    c9_done.set()
        pool.submit(_followup)
    pool.submit(_initial)

c9_done.wait(timeout=5.0)
s = pool.stats()
print(f"  stats: {s.submitted} submitted, {s.executed} executed, {s.rejected} rejected")
passed &= check(name, s.submitted == TOTAL_EXEC,
                "all initial + follow-up jobs should be submitted")
passed &= check(name, s.executed == TOTAL_EXEC,
                "all initial + follow-up jobs should be executed")
passed &= check(name, s.rejected == 0, "no jobs should be rejected")
pool.stop()
results.append((name, passed))

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print("==================== Results ====================")
total_passed = sum(1 for _, p in results if p)
for r_name, r_passed in results:
    tag = "PASS" if r_passed else "FAIL"
    print(f"  {tag}  {r_name}")
print("=================================================")
print(f"{total_passed}/{len(results)} tests passed")
if total_passed == len(results):
    print("All tests passed!")
    sys.exit(0)
else:
    print(f"{len(results) - total_passed} test(s) FAILED")
    sys.exit(1)
