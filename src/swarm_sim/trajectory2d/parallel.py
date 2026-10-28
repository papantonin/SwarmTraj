"""Deterministic process-based catalogue construction, never flight execution."""

from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import os
import sys
import warnings


def available_cpus():
    try:
        return max(1, len(os.sched_getaffinity(0)))
    except (AttributeError, OSError):
        return os.cpu_count() or 1


def worker_count(requested, task_count):
    return max(1, min(requested or available_cpus(), available_cpus(), task_count))


def generate_catalogs(function, tasks, requested):
    count = worker_count(requested, len(tasks))
    main_file = getattr(sys.modules.get("__main__"), "__file__", None)
    if count > 1 and (not main_file or main_file.startswith("<")):
        warnings.warn("Parallel planning requires an importable __main__ file with a main guard; "
                      "using serial execution in this interactive session.", RuntimeWarning, stacklevel=2)
        count = 1
    if count == 1:
        return [function(task) for task in tasks]
    # spawn avoids inheriting matplotlib/Tk locks, RNG state or an active BLAS
    # runtime from the editor. map returns ID-ordered results, not finish order.
    # Worker exceptions propagate: never return a partial/unchecked mission.
    with ProcessPoolExecutor(max_workers=count, mp_context=multiprocessing.get_context("spawn")) as pool:
        return list(pool.map(function, tasks))
