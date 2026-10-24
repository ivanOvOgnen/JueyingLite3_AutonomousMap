"""Commissioning-gated ownership supervision and transactional command arming.

The vendor ownership adapter is an external prerequisite. Missing evidence
never becomes a manual-clear or autonomy permit. No UDP/vendor mode changes
are made by this node.
"""
import hashlib
import json
from pathlib import Path
import threading
import time


class SupervisorPolicy:
    def __init__(self, commissioning=None, *, clock=time.monotonic, wall_clock=time.time_ns):
        self.clock, self.wall_clock = clock, wall_clock
        self.lock = threading.RLock()
        self.commissioning = commissioning if isinstance(commissioning, dict) else {}
        self.vendor, self.context = {}, {}
        self.vendor_rx = self.context_rx = None
        self.state, self.reason = 'stopped', 'Explicit arm and commissioning evidence required'
        self.generation = 0
        self.stages = {}

    def update_context(self, value):
        with self.lock:
            self.context = value if isinstance(value, dict) else {}
            self.context_rx = self.clock()

    def update_vendor(self, value):
        with self.lock:
            try:
                if not isinstance(value, dict):
                    raise ValueError('Vendor ownership is not an object')
                if not isinstance(value['sequence'], int) or value['sequence'] < 0:
                    raise ValueError('Invalid vendor sequence')
                if self.vendor and (value['instance_id'] != self.vendor['instance_id'] or
                                    value['sequence'] <= self.vendor['sequence']):
                    self.stop('Vendor source restarted or replayed')
                    raise ValueError('Vendor instance/sequence changed')
                self.vendor = value
                self.vendor_rx = self.clock()
            except (KeyError, ValueError, TypeError):
                self.vendor = {}
                self.stop('Malformed/replayed vendor ownership state')

    def inhibit_reason(self):
        with self.lock:
            now = self.clock()
            c = self.commissioning
            if c.get('format') != 'lite3_motion_commissioning/v1' or not c.get('commissioning_id'):
                return 'No valid commissioning record'
            for field in ('vendor_exclusivity_verified', 'independent_stop_verified',
                          'downstream_timeout_verified', 'footprint_measured', 'visibility_reviewed'):
                if c.get(field) is not True:
                    return 'Unverified commissioning gate: ' + field
            if self.vendor_rx is None or not 0 <= now-self.vendor_rx <= 0.3:
                return 'Vendor ownership state missing or stale'
            vendor = self.vendor
            if (vendor.get('mode') != 'autonomy' or vendor.get('posture') != 'standing' or
                    vendor.get('manual_input_active') is not False or vendor.get('stop_clear') is not True or
                    vendor.get('exclusive_owner') != 'lite3_autonomy' or
                    vendor.get('commissioning_id') != c['commissioning_id']):
                return 'Vendor ownership/manual/stop/posture conditions are not satisfied'
            stamp = vendor.get('source_unix_ns')
            if not isinstance(stamp, int) or not -0.05 <= (self.wall_clock()-stamp)/1e9 <= 0.3:
                return 'Vendor ownership acquisition timestamp invalid or stale'
            if self.context_rx is None or not 0 <= now-self.context_rx <= 0.3:
                return 'API navigation context missing or stale'
            context = self.context
            if context.get('ready') is not True or context.get('operator_lease_valid') is not True:
                return 'API readiness/operator lease not valid'
            if (context.get('map_id') not in c.get('allowed_map_ids', []) or
                    context.get('software_id') != c.get('software_id')):
                return 'Map/software do not match the commissioning record'
            if self.state == 'autonomous':
                for name in ('gate', 'arbiter', 'relay'):
                    stage = self.stages.get(name, {})
                    if stage.get('enabled') is not True or not 0 <= now-stage.get('rx', -1e9) <= 0.3:
                        return 'Command stage disabled or stale: ' + name
            return None

    def stop(self, reason):
        with self.lock:
            self.generation += 1
            self.state, self.reason = 'stopped', reason

    def tick(self):
        with self.lock:
            reason = self.inhibit_reason()
            if reason and self.state in ('arming', 'autonomous'):
                self.stop(reason)
            return self.state in ('arming', 'autonomous') and reason is None

    def arm_transaction(self, enable):
        """enable(name, bool) waits for an actual stage response; gate is last."""
        with self.lock:
            if self.state in ('arming', 'autonomous'):
                raise ValueError('An arm transaction is already active')
            reason = self.inhibit_reason()
            if reason:
                raise ValueError(reason)
            self.generation += 1
            generation = self.generation
            self.state, self.reason = 'arming', 'Enabling downstream protections first'
        try:
            for name in ('relay', 'arbiter', 'gate'):
                with self.lock:
                    if generation != self.generation or self.inhibit_reason():
                        raise ValueError('Arming interrupted or prerequisite lost')
                if not enable(name, True):
                    raise ValueError('Stage refused enable: ' + name)
            with self.lock:
                if generation != self.generation or self.inhibit_reason():
                    raise ValueError('Arming interrupted')
                self.state = 'autonomous'
                # Actual heartbeat confirmation is separate from service ACK.
                reason = self.inhibit_reason()
                if reason:
                    raise ValueError(reason)
                self.reason = 'Exclusive commissioned command path armed'
        except Exception as error:
            self.stop(str(error))
            for name in ('gate', 'arbiter', 'relay'):
                try:
                    enable(name, False)
                except Exception:
                    pass
            raise

    def status(self):
        with self.lock:
            permit = self.tick()
            return {'enabled': self.state == 'autonomous' and permit, 'state': self.state,
                    'ownership_permit': permit, 'reason': self.inhibit_reason() or self.reason,
                    'commissioning_id': self.commissioning.get('commissioning_id'),
                    'generation': self.generation}


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool

    class MotionSupervisor(Node):
        def __init__(self):
            super().__init__('lite3_motion_supervisor')
            self.declare_parameter('commissioning_file', str(Path.home()/'.config/lite3_autonomy/commissioning.json'))
            try:
                record = json.loads(Path(self.get_parameter('commissioning_file').value).read_text())
            except (OSError, ValueError):
                record = {}
            self.policy = SupervisorPolicy(record)
            self.permit = self.create_publisher(Bool, '/autonomy/ownership_permit', 1)
            self.manual = self.create_publisher(Bool, '/autonomy/manual_override', 1)
            self.status_pub = self.create_publisher(String, '/autonomy/supervisor/status', 1)
            self.clients = {name: self.create_client(SetBool, topic) for name, topic in (
                ('gate', '/autonomy/set_enabled'), ('arbiter', '/autonomy/arbiter/set_enabled'),
                ('relay', '/autonomy/vendor_bridge/set_enabled'))}
            self.create_subscription(String, '/vendor/ownership_state',
                                     lambda msg: self.receive(msg, self.policy.update_vendor), 1)
            self.create_subscription(String, '/autonomy/context',
                                     lambda msg: self.receive(msg, self.policy.update_context), 1)
            for name, topic in (('gate', '/autonomy/gate/status'), ('arbiter', '/autonomy/motion_status'),
                                ('relay', '/autonomy/vendor_bridge/status')):
                self.create_subscription(String, topic, lambda msg, key=name: self.stage(key, msg), 1)
            self.create_service(SetBool, '/autonomy/supervisor/set_enabled', self.set_enabled)
            self.create_timer(0.05, self.tick)
            self._was_permitted = False

        def receive(self, msg, callback):
            try:
                callback(json.loads(msg.data))
            except (ValueError, TypeError):
                callback({})
            self.tick()

        def stage(self, name, msg):
            try:
                value = json.loads(msg.data)
                if not isinstance(value, dict):
                    raise ValueError('Invalid stage')
                with self.policy.lock:
                    self.policy.stages[name] = {**value, 'rx': time.monotonic()}
            except (ValueError, TypeError):
                self.policy.stop('Malformed command stage status')

        def enable(self, name, value):
            client = self.clients[name]
            if not client.service_is_ready():
                return False
            if value:
                self.tick()
                deadline = time.monotonic()+0.3
                while time.monotonic() < deadline:
                    with self.policy.lock:
                        ready = self.policy.stages.get(name, {}).get('ownership_fresh') is True
                    if ready:
                        break
                    time.sleep(0.01)
                else:
                    return False
            event = threading.Event()
            future = client.call_async(SetBool.Request(data=value))
            future.add_done_callback(lambda _: event.set())
            if not event.wait(1.0):
                # A late enable response must trigger a fresh disable.
                if value:
                    future.add_done_callback(lambda _: client.call_async(SetBool.Request(data=False)))
                return False
            accepted = bool(future.result().success)
            if accepted and value:
                deadline = time.monotonic()+0.2
                while time.monotonic() < deadline:
                    with self.policy.lock:
                        if self.policy.stages.get(name, {}).get('enabled') is True:
                            return True
                    time.sleep(0.01)
                return False
            return accepted

        def disable_all(self):
            for client in self.clients.values():
                if client.service_is_ready():
                    client.call_async(SetBool.Request(data=False))

        def set_enabled(self, request, response):
            if not request.data:
                self.policy.stop('Explicit software disable')
                self.tick()
                self.disable_all()
                response.success, response.message = True, 'Ownership revoked; stage disable requested'
                return response
            reason = self.policy.inhibit_reason()
            if reason or self.policy.state in ('arming', 'autonomous'):
                response.success, response.message = False, reason or 'Already arming/armed'
                return response
            def arm():
                try:
                    self.policy.arm_transaction(self.enable)
                except Exception as error:
                    self.get_logger().error('Arming failed: ' + str(error))
            threading.Thread(target=arm, daemon=True).start()
            response.success, response.message = True, 'Arming requested; inspect supervisor status for confirmation'
            return response

        def tick(self):
            value = self.policy.status()
            self.permit.publish(Bool(data=value['ownership_permit']))
            self.manual.publish(Bool(data=not value['ownership_permit']))
            msg = String()
            msg.data = json.dumps(value)
            self.status_pub.publish(msg)
            if self._was_permitted and not value['ownership_permit']:
                self.disable_all()
            self._was_permitted = value['ownership_permit']

    rclpy.init(args=args)
    node = MotionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.policy.stop('Supervisor exiting')
        node.tick()
        node.disable_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
