"""Run one function in a fresh spawned process: MuJoCo's compile and OCCT hold the GIL."""
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeout
from concurrent.futures.process import BrokenProcessPool

_spawn = multiprocessing.get_context("spawn")


class ChildFailed(Exception):
    pass


def run_in_child(fn, *args, timeout: float = 60.0):
    """fn(*args) in a new process. Exceptions fn raises come back as-is; a crash or timeout is ChildFailed."""
    pool = ProcessPoolExecutor(max_workers=1, mp_context=_spawn)
    try:
        return pool.submit(fn, *args).result(timeout=timeout)
    except BrokenProcessPool as exc:
        raise ChildFailed("crashed") from exc
    except FutureTimeout as exc:
        raise ChildFailed("timed out") from exc
    finally:
        for p in list((getattr(pool, "_processes", None) or {}).values()):  # a timed-out child must not linger
            if p.is_alive():
                p.kill()
        pool.shutdown(wait=False, cancel_futures=True)
