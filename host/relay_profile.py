"""Opt-in main-thread C-call timing for relay diagnosis; no signal handlers."""
import json
import sys
import time


def run():
    from host.ros_tunnel import main
    active = []

    def profile(frame, event, arg):
        if event == 'c_call':
            active.append((arg, time.monotonic(), time.thread_time()))
        elif event in ('c_return', 'c_exception') and active:
            call, started, cpu = active.pop()
            elapsed = time.monotonic()-started
            if elapsed >= .1:
                print(json.dumps({'relay_c_call': getattr(call, '__qualname__', str(type(call))),
                                  'caller': frame.f_code.co_name, 'elapsed_s': elapsed,
                                  'cpu_s': time.thread_time()-cpu, 'wall_ns': time.time_ns()}), flush=True)

    sys.setprofile(profile)
    try:
        main()
    finally:
        sys.setprofile(None)
