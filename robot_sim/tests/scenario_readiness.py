"""Pure-Python helpers of robot_sim/tests/bottle_scenario.py (line D2, 2026-10-14; no ROS import, unit-tested).

D1 run_b2 sat 420 s in the scenario's "remember" stage although the runtime had the bottle
current_confirmed: one of the scenario's own readiness conditions (navigate_to_pose action server
ready, velocity_smoother lifecycle state active, /odom received) never held, and the records did not
say which. Suspect: the GetState request had no timeout -- one lost reply left its future pending
forever, no new request was ever sent and ``navigation_active`` stayed False.

ServiceStatePoller: one request in flight at most; a request unanswered for ``timeout_s`` is
abandoned (the caller drops the pending request) and a new one is sent at the next due time.
ReadinessLog: named conditions; every change is a row with the time it was seen.
"""


class ServiceStatePoller:
    """Request/answer bookkeeping with a timeout and resend (time in seconds, caller's clock).

    Caller loop:  if poller.due(now) and service_ready: poller.sent(now, client.call_async(req))
                  outcome = poller.check(now)   # ('answer', future) / ('timeout', future) / None
                  on 'timeout' the caller removes the pending request (client.remove_pending_request)
    """

    def __init__(self, period_s=0.5, timeout_s=2.0):
        if not (period_s > 0 and timeout_s > 0):
            raise ValueError('period_s and timeout_s must be positive')
        self.period_s, self.timeout_s = float(period_s), float(timeout_s)
        self.future = None
        self.sent_at = None
        self.last_send = None
        self.counts = {'sent': 0, 'answered': 0, 'timeouts': 0}

    def due(self, now):
        return self.future is None and (self.last_send is None or now - self.last_send >= self.period_s)

    def sent(self, now, future):
        if self.future is not None:
            raise RuntimeError('a request is already in flight')
        self.future, self.sent_at, self.last_send = future, now, now
        self.counts['sent'] += 1

    def check(self, now):
        if self.future is None:
            return None
        future = self.future
        if future.done():
            self.future = None
            self.counts['answered'] += 1
            return ('answer', future)
        if now - self.sent_at >= self.timeout_s:
            self.future = None
            self.counts['timeouts'] += 1
            return ('timeout', future)
        return None


class ReadinessLog:
    """Named readiness conditions; update() returns the change rows (empty when nothing changed)."""

    def __init__(self, names):
        self.state = {name: None for name in names}
        self.since = {name: None for name in names}

    def update(self, now, clock=None, **values):
        rows = []
        for name, value in values.items():
            if name not in self.state:
                raise KeyError(name)
            value = bool(value)
            if self.state[name] != value:
                rows.append({'item': name, 'value': value, 'previous': self.state[name],
                             'elapsed_wall': now, 'clock': clock})
                self.state[name] = value
                self.since[name] = now
        return rows

    def snapshot(self, now):
        return {name: {'value': self.state[name],
                       'for_s': None if self.since[name] is None else now - self.since[name]}
                for name in self.state}

    def all_true(self):
        return all(v is True for v in self.state.values())
