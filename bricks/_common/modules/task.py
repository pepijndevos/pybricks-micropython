from pybricks import set_async_mode

set_async_mode(True)


def run_parallel(*tasks):
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


main_done = False


def main(main_task):
    # Make sure this is used only once.
    global main_done
    if main_done:
        raise RuntimeError("Can have only one main program.")
    main_done = True

    # Run the main program
    for _ in main_task():
        pass
