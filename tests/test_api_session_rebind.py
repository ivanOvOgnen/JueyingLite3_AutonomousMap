"""Subscription replacement must run on the ROS executor, not an HTTP worker."""

import threading
import time


def test_rebind_runs_in_executor_callback(api, api_module):
    api._rebind_lock = threading.Lock()
    api._rebind_requested = api._rebind_completed = 0
    api._rebind_error = None
    api._rebind_event = threading.Event()
    failures = []

    def request():
        try:
            api_module.RobotApi._request_session_rebind(api)
        except Exception as error:
            failures.append(error)

    worker = threading.Thread(target=request, name='http-worker')
    worker.start()
    deadline = time.monotonic() + 1
    while api._rebind_requested == 0 and time.monotonic() < deadline:
        time.sleep(0.001)
    assert api._rebind_requested == 1
    assert not api._bind_session_topics.called
    api_module.RobotApi._service_session_rebind(api)
    worker.join(timeout=1)
    assert not worker.is_alive()
    assert failures == []
    api._bind_session_topics.assert_called_once_with()
    assert api._rebind_completed == 1


def test_no_rebind_when_no_request(api, api_module):
    api._rebind_lock = threading.Lock()
    api._rebind_requested = api._rebind_completed = 0
    api_module.RobotApi._service_session_rebind(api)
    api._bind_session_topics.assert_not_called()


def test_spin_loop_discards_wait_set_before_rebind(api, api_module, monkeypatch):
    api._rebind_lock = threading.Lock()
    api._rebind_requested = api._rebind_completed = 0
    api._rebind_error = None
    api._rebind_event = threading.Event()
    events = []

    class FakeExecutor:
        def add_node(self, _node):
            events.append('add')

        def spin_once(self, timeout_sec):
            assert timeout_sec == 0.05
            events.append('spin')
            if events.count('spin') == 1:
                api._rebind_requested = 1

        def remove_node(self, _node):
            events.append('remove')

        def shutdown(self):
            events.append('shutdown')

    api._bind_session_topics.side_effect = lambda: events.append('bind')
    monkeypatch.setattr(api_module, 'SingleThreadedExecutor', FakeExecutor)
    monkeypatch.setattr(api_module.rclpy, 'ok', lambda: events.count('spin') < 2, raising=False)
    api_module.spin_api(api)
    assert events[:6] == ['add', 'spin', 'remove', 'shutdown', 'bind', 'add']
    assert api._rebind_completed == 1
