"""Lean subscription-only executor for the hub node (rclpy humble), the hub's default.

Why: rclpy humble's SingleThreadedExecutor rebuilds its wait set in Python on every wake-up
(every subscription, QoS-event waitable, service, guard: context managers, entity counting,
Task/coroutine per callback). Measured in the fpga-dev1-sim:humble container, hub-like node
(15 subscriptions, 3 publishers), 60 msg/s: 1.37 ms CPU per message with the stock executor,
0.76 ms with QoS default event handlers and parameter services off, 0.24 ms with this loop
(results/hub/bench/exec_bench2.txt). On the host chain the spin thread was 2/3 of the hub's CPU.

What it does -- the same as the stock executor for a node that has only subscriptions:
one wait set built once, re-filled each round with every subscription of the node plus a
guard condition; per round, for every ready subscription, take ONE message (as rclpy does)
and call its callback directly. Same callbacks, same per-topic order, same QoS (the
subscriptions are created by the node as before).

What it refuses (SystemExit from add_node, i.e. in the main thread before spinning; never
silently ignored): timers, services, clients, waitables (e.g. QoS event handlers), guard
conditions on the node, or a coroutine callback -- anything the stock executor would serve and
this loop would not. One exception, named: the default incompatible-QoS handler of rclpy's
built-in /parameter_events publisher (see tolerated_waitables). So the hub node
is created with ``start_parameter_services=False`` and QoS default event callbacks off.

Depends on rclpy humble internals (``rclpy.impl.implementation_singleton.rclpy_implementation``
WaitSet / take_message / handle context managers), checked in the humble images only;
``--executor rclpy`` (or FPGA_HUB_EXECUTOR=rclpy) gives back the stock executor.

Start-up self check (2026-10-12 v5): ``LeanExecutor.probe(node)`` runs in the main thread before
spinning and exercises exactly those internals once (import, WaitSet built with the arity used
here, clear/add/wait(0)/get_ready_entities, take_message present on every subscription handle) --
nothing is taken. ``SelectingExecutor`` (the hub's default) uses the lean loop only when the probe
passes, else the stock executor, and reports which one and why (hub STATE ``hub.executor``, EVENT
``hub_executor_fallback``). Before, a missing/changed internal only showed up as an exception in
the spin thread: the hub kept serving TCP with no ROS input.
"""

import inspect
import threading
from contextlib import ExitStack


class LeanExecutor(object):
    def __init__(self):
        self.node = None
        self.context = None
        self._guard = None
        self._stop = False
        self._done = threading.Event()
        self._started = False
        self.rounds = 0
        self.taken = 0

    def add_node(self, node):
        if self.node is not None:
            raise ValueError("LeanExecutor serves exactly one node")
        from rclpy.guard_condition import GuardCondition
        self.check_node(node)                      # in the caller's (main) thread: loud, not a dead spin thread
        self.node = node
        self.context = node.context
        self._guard = GuardCondition(None, None, self.context)
        self.context.on_shutdown(self._wake)      # as rclpy's Executor: wake on context shutdown
        return True

    def _wake(self):
        try:
            self._guard.trigger()
        except Exception:                          # guard already destroyed
            pass

    @staticmethod
    def tolerated_waitables(node):
        """The default incompatible-QoS handler(s) of rclpy's built-in /parameter_events publisher
        (every Node creates it; start_parameter_services=False does not remove it). Not served by
        this loop: its only effect would be a warn log, and the hub declares no parameters and
        publishes no parameter events after start-up."""
        out = set()
        for pub in getattr(node, "publishers", ()) or ():
            if pub.topic_name in ("/parameter_events", "parameter_events"):
                out.update(id(h) for h in (getattr(pub, "event_handlers", None) or ()))
        return out

    def check_node(self, node):
        """Raise SystemExit naming what this loop would not serve."""
        extra = []
        ok_waitables = self.tolerated_waitables(node)
        for kind in ("timers", "services", "clients", "waitables", "guards"):
            items = list(getattr(node, kind, ()) or ())
            if kind == "waitables":
                items = [w for w in items if id(w) not in ok_waitables]
            if items:
                extra.append("%s=%d" % (kind, len(items)))
        for sub in node.subscriptions:
            if inspect.iscoroutinefunction(sub.callback):
                extra.append("coroutine callback on %s" % sub.topic_name)
        if extra:
            raise SystemExit("LeanExecutor serves subscriptions only; node has " + ", ".join(extra)
                             + " (use --executor rclpy)")

    WAITSET_METHODS = ("clear_entities", "add_subscription", "add_guard_condition", "wait",
                       "get_ready_entities")

    @classmethod
    def probe(cls, node, impl=None, guard_factory=None):
        """None if the rclpy internals spin() uses are there and callable as spin() calls them,
        else a short reason (``import:...``, ``missing:...``, ``signature:...``, ``probe:...``)."""
        try:
            if impl is None:
                from rclpy.impl.implementation_singleton import rclpy_implementation as impl
            if guard_factory is None:
                from rclpy.guard_condition import GuardCondition
                guard_factory = (lambda ctx: GuardCondition(None, None, ctx))
        except Exception as exc:                   # ImportError and friends
            return "import:%s: %s" % (type(exc).__name__, exc)
        if not hasattr(impl, "WaitSet"):
            return "missing:WaitSet"
        subs = list(getattr(node, "subscriptions", ()) or ())
        ctx = getattr(node, "context", None)
        if ctx is None or not hasattr(ctx, "handle"):
            return "missing:context.handle"
        for sub in subs:
            for attr in ("handle", "msg_type", "raw", "callback"):
                if not hasattr(sub, attr):
                    return "missing:subscription.%s" % attr
            for attr in ("pointer", "take_message", "__enter__", "__exit__"):
                if not hasattr(sub.handle, attr):
                    return "missing:subscription.handle.%s" % attr
        try:
            guard = guard_factory(ctx)
        except Exception as exc:
            return "probe:guard:%s: %s" % (type(exc).__name__, exc)
        try:
            with ExitStack() as stack:
                for sub in subs:
                    stack.enter_context(sub.handle)
                stack.enter_context(guard.handle)
                stack.enter_context(ctx.handle)
                ws = impl.WaitSet(len(subs), 1, 0, 0, 0, 0, ctx.handle)
                for m in cls.WAITSET_METHODS:
                    if not hasattr(ws, m):
                        return "missing:WaitSet.%s" % m
                ws.clear_entities()
                for sub in subs:
                    ws.add_subscription(sub.handle)
                ws.add_guard_condition(guard.handle)
                ws.wait(0)
                ready = ws.get_ready_entities("subscription")
                for sub in subs:
                    sub.handle.pointer in ready     # noqa: B015 -- the membership test spin() does
        except TypeError as exc:
            return "signature:%s" % (exc,)
        except Exception as exc:
            return "probe:%s: %s" % (type(exc).__name__, exc)
        finally:
            try:
                guard.destroy()
            except Exception:
                pass
        return None

    def spin(self):
        from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
        self._started = True
        try:
            if self._stop:
                return
            node, ctx = self.node, self.context
            self.check_node(node)
            subs = list(node.subscriptions)
            with ExitStack() as stack:
                for s in subs:
                    stack.enter_context(s.handle)
                stack.enter_context(self._guard.handle)
                stack.enter_context(ctx.handle)
                ws = _rclpy.WaitSet(len(subs), 1, 0, 0, 0, 0, ctx.handle)
                table = [(s.handle, s.handle.pointer, s.msg_type, s.raw, s.callback) for s in subs]
                guard = self._guard.handle
                while not self._stop:
                    ws.clear_entities()
                    for h, _, _, _, _ in table:
                        ws.add_subscription(h)
                    ws.add_guard_condition(guard)
                    try:
                        ws.wait(-1)
                    except Exception:
                        if self._stop or not ctx.ok():
                            break                  # context shut down under us: leave quietly
                        raise
                    if self._stop or not ctx.ok():
                        break
                    ready = ws.get_ready_entities("subscription")
                    self.rounds += 1
                    if not ready:
                        continue
                    for h, ptr, msg_type, raw, callback in table:
                        if ptr in ready:
                            info = h.take_message(msg_type, raw)
                            if info is not None:
                                self.taken += 1
                                callback(info[0])
        finally:
            guard, self._guard = self._guard, None
            if guard is not None:                  # handles released above (ExitStack)
                guard.destroy()
            self._done.set()

    def shutdown(self, timeout_sec=None):
        """Stop the loop and wait (<= timeout_sec) for spin() to return; True if it did (or
        it never started -- then a later spin() returns at once)."""
        self._stop = True
        self._wake()
        return self._done.wait(timeout_sec) if self._started else True


class SelectingExecutor(object):
    """The hub's executor: LeanExecutor when ``LeanExecutor.probe(node)`` passes, else the stock
    one from ``fallback_factory`` (rclpy SingleThreadedExecutor). ``on_choice(info)`` is called
    once from add_node (main thread) with {"requested", "used", "fallback_reason"}. Everything
    else is delegated. ``probe`` is injectable for tests."""

    def __init__(self, fallback_factory, on_choice=None, probe=None, lean_factory=LeanExecutor):
        self.fallback_factory = fallback_factory
        self.on_choice = on_choice
        self.probe = probe or LeanExecutor.probe
        self.lean_factory = lean_factory
        self.inner = None
        self.info = None

    def add_node(self, node):
        reason = self.probe(node)
        if reason is None:
            self.inner = self.lean_factory()
            self.info = {"requested": "lean", "used": "lean", "fallback_reason": None}
        else:
            self.inner = self.fallback_factory()
            self.info = {"requested": "lean", "used": "rclpy", "fallback_reason": reason}
        if self.on_choice is not None:
            self.on_choice(dict(self.info))
        return self.inner.add_node(node)

    def spin(self):
        return self.inner.spin()

    def shutdown(self, timeout_sec=None):
        return True if self.inner is None else self.inner.shutdown(timeout_sec=timeout_sec)
