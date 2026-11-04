"""Test helper: a TCP relay that can stall, delay (in order) or cut the Nano->PC direction.

Stalling stops *reading* from the upstream socket, so the hub's send buffer fills exactly as
it does when a hotspot stops delivering. ``rcvbuf`` is applied to the upstream socket before
connect so the relay itself holds little data.
"""

import collections
import socket
import threading
import time


class _Pair(object):
    def __init__(self, relay, down, up):
        self.relay, self.down, self.up = relay, down, up
        self.paused = threading.Event()
        self.closed = threading.Event()
        self.queue = collections.deque()
        self.cv = threading.Condition()
        self.last_release = 0.0
        self.threads = [threading.Thread(target=f, daemon=True)
                        for f in (self._c2u, self._u2c, self._deliver)]

    def start(self):
        """Started only after the relay has put the pair in its table (v5 fix): before, the threads
        ran first, the client's handshake could already pass through them, and a pause() issued
        right after found an empty table (test_hub positive control, ~2/60 on py3.8)."""
        for t in self.threads:
            t.start()

    def _c2u(self):
        try:
            while not self.closed.is_set():
                data = self.down.recv(65536)
                if not data:
                    break
                self.up.sendall(data)
        except OSError:
            pass
        self.close()

    def _u2c(self):
        try:
            while not self.closed.is_set():
                if self.paused.is_set():
                    time.sleep(0.005)
                    continue
                self.up.settimeout(0.05)
                try:
                    data = self.up.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                delay = self.relay.delay_fn() if self.relay.delay_fn else 0.0
                with self.cv:
                    self.last_release = max(self.last_release, time.monotonic() + delay)
                    self.queue.append((self.last_release, data))
                    self.cv.notify()
        except OSError:
            pass
        self.close()

    def _deliver(self):
        try:
            while not self.closed.is_set():
                with self.cv:
                    while not self.queue and not self.closed.is_set():
                        self.cv.wait(0.05)
                    if self.closed.is_set():
                        return
                    due, data = self.queue[0]
                if self.paused.is_set():            # a chunk read just before pause is held too
                    time.sleep(0.005)
                    continue
                wait = due - time.monotonic()
                if wait > 0:
                    time.sleep(min(wait, 0.05))
                    continue
                with self.cv:
                    self.queue.popleft()
                self.down.sendall(data)
        except OSError:
            self.close()

    def close(self):
        if self.closed.is_set():
            return
        self.closed.set()
        with self.cv:
            self.cv.notify_all()
        for s in (self.down, self.up):
            try:
                s.close()
            except OSError:
                pass


class Relay(object):
    def __init__(self, upstream_port, upstream_host="127.0.0.1", rcvbuf=4096):
        self.upstream = (upstream_host, upstream_port)
        self.rcvbuf = rcvbuf
        self.delay_fn = None
        self.pairs = []
        self.lock = threading.Lock()
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(8)
        self.port = self.listener.getsockname()[1]
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._accept, daemon=True)
        self.thread.start()

    def _accept(self):
        self.listener.settimeout(0.1)
        while not self._stop.is_set():
            try:
                down, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            up = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if self.rcvbuf:
                up.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.rcvbuf)
            try:
                up.connect(self.upstream)
            except OSError:
                down.close()
                up.close()
                continue
            down.settimeout(None)
            pair = _Pair(self, down, up)
            with self.lock:
                self.pairs.append(pair)        # in the table first ...
            pair.start()                       # ... then forwarding starts

    def _snapshot(self):
        with self.lock:
            return list(self.pairs)

    def pause(self):
        for p in self._snapshot():
            p.paused.set()

    def resume(self):
        for p in self._snapshot():
            p.paused.clear()

    def kill(self):
        for p in self._snapshot():
            p.close()

    def close(self):
        self._stop.set()
        self.kill()
        self.listener.close()
        self.thread.join(timeout=5)
        for p in self._snapshot():
            for t in p.threads:
                if t.is_alive() or t.ident is not None:
                    t.join(timeout=5)
