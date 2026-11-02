"""Bounded lifecycle observation; no lifecycle transitions or safety authority."""
class CMStatePoll:
    def __init__(self, *, response_timeout_s=2.0, retry_interval_s=0.2, max_attempts=5):
        self.timeout = response_timeout_s
        self.interval = retry_interval_s
        self.max_attempts = max_attempts
        self.future = None
        self.sequence = 0
        self.attempts = 0
        self.dispatched = None
        self.response_at = None
        self.state_id = None
        self.next_request = 0.0
        self.discovered = False
        self.exception = None

    def step(self, client, request_factory, now):
        events = []
        def event(kind, **fields):
            events.append(dict(event=kind, monotonic=now, sequence=self.sequence, **fields))
        available = client.service_is_ready()
        if available != self.discovered:
            event('service_discovery', available=available)
            if available:
                self.attempts = 0
            self.discovered = available
        if not available:
            self.state_id = None
        if self.future is not None:
            age = now - self.dispatched
            if self.future.done():
                try:
                    response = self.future.result()
                    self.state_id = int(response.current_state.id) if available else None
                    self.response_at = now
                    self.exception = None
                    self.attempts = 0
                    event('response', response_monotonic=now, state_id=self.state_id,
                          dispatch_monotonic=self.dispatched, pending_age_s=age)
                except Exception as error:
                    self.state_id = None
                    self.exception = f'{type(error).__name__}: {error}'
                    event('exception', exception=self.exception, pending_age_s=age)
                self.future = None
                self.next_request = now + self.interval
            elif age >= self.timeout or not available:
                try:
                    # Remove the rclpy client request as well as cancelling its
                    # future; late responses cannot populate current state.
                    client.remove_pending_request(self.future)
                except Exception as error:
                    self.exception = f'{type(error).__name__}: {error}'
                    event('remove_pending_exception', exception=self.exception)
                try:
                    self.future.cancel()
                    event('cancel', reason='timeout' if available else 'service_unavailable',
                          dispatch_monotonic=self.dispatched, pending_age_s=age)
                except Exception as error:
                    self.exception = f'{type(error).__name__}: {error}'
                    event('cancel_exception', exception=self.exception)
                self.future = None
                self.state_id = None
                self.next_request = now + self.interval
        if available and self.future is None and now >= self.next_request:
            if self.attempts < self.max_attempts:
                self.sequence += 1
                self.attempts += 1
                self.dispatched = now
                try:
                    self.future = client.call_async(request_factory())
                    event('dispatch', dispatch_monotonic=now, attempt=self.attempts)
                except Exception as error:
                    self.state_id = None
                    self.exception = f'{type(error).__name__}: {error}'
                    self.next_request = now + self.interval
                    event('exception', exception=self.exception)
        fresh = (available and self.response_at is not None
                 and 0 <= now - self.response_at <= 1.0 and self.state_id is not None)
        return dict(service_available=available, sequence=self.sequence,
                    dispatch_monotonic=self.dispatched, response_monotonic=self.response_at,
                    pending_age_s=(now-self.dispatched if self.future is not None else None),
                    state_id=self.state_id, fresh=fresh, active=fresh and self.state_id == 3,
                    attempts=self.attempts, attempts_exhausted=self.attempts >= self.max_attempts,
                    exception=self.exception, events=events)
