from pybricks import set_async_mode
from pybricks.tools import wait as _wait


def all(*tasks):
    completed = {t: False for t in tasks}
    all_done = False
    while not all_done:
        all_done = True
        for t in tasks:
            if not completed[t]:
                all_done = False
                try:
                    next(t)
                except StopIteration:
                    completed[t] = True
                yield


# Revisit: Implement completely separately from pybricks.tools.wait
def wait(time):
    yield from _wait(time)


def run(main_task):
    set_async_mode(True)
    try:
        for _ in main_task:
            pass
    finally:
        set_async_mode(False)
