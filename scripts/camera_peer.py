"""One off-thread Unix peer watcher for each camera node's control peer."""
import ctypes
import os
import random
import select
import struct
import threading
import time

import rospy
from xgc2_xrpc import Client, Runtime, ServiceRef


class CameraPeer:
    def __init__(self, endpoint, target_id, service, reconcile):
        self.endpoint, self.target_id, self.service = endpoint, target_id, service
        self.reconcile = reconcile
        self._closed = False
        self.status = {"state": "waiting", "attempts": 0, "desired_revision": 1, "applied_revision": 0}
        self.stop = threading.Event()
        self._runtime = Runtime(blocking_workers=1, max_calls=4, max_connections=4)
        self._discovery = Client(endpoint, runtime=self._runtime)
        self._client = None
        self._wake_read, self._wake_write = os.pipe2(os.O_NONBLOCK | os.O_CLOEXEC)
        libc = ctypes.CDLL(None, use_errno=True)
        self._events = libc.inotify_init1(os.O_NONBLOCK | os.O_CLOEXEC)
        # Core owns the runtime directory before starting any of its peers.
        if self._events < 0 or libc.inotify_add_watch(self._events, os.fsencode(os.path.dirname(endpoint)),
                                                    0x100 | 0x200 | 0x80 | 0x40) < 0:
            self.close()
            raise OSError(ctypes.get_errno(), "camera peer directory watch failed")
        self._thread = threading.Thread(target=self._work, name=service + "-watch")
        self._thread.start()

    def _wait(self, timeout=None):
        deadline = None if timeout is None else time.monotonic() + timeout
        while not self.stop.is_set():
            remaining = None if deadline is None else max(0, deadline - time.monotonic())
            readable, _, _ = select.select([self._wake_read, self._events], [], [], remaining)
            if not readable:
                return
            if self._wake_read in readable:
                os.read(self._wake_read, 65536)
                if self.stop.is_set() or deadline is None:
                    return
                continue
            data = os.read(self._events, 65536)
            offset = 0
            while offset < len(data):
                _, _, _, length = struct.unpack_from("iIII", data, offset)
                name = data[offset + 16:offset + 16 + length].split(b"\0", 1)[0]
                offset += 16 + length
                if name == os.fsencode(os.path.basename(self.endpoint)):
                    return

    def _work(self):
        ceiling, last_log, suppressed = 0.25, None, 0
        try:
            while not self.stop.is_set():
                # Socket appearance is an event, not a stream of failed connects.
                if not os.path.exists(self.endpoint):
                    self.status["state"] = "waiting"
                    self._wait()
                    continue
                try:
                    self.status["attempts"] += 1
                    description = self._discovery.json("/v1/describe?wait_ready_ms=30000", method="GET", timeout=31)
                    if not description["ready"]:
                        raise RuntimeError("peer is not ready")
                    reference = ServiceRef.from_dict(description.get("facts", {}).get("service_ref", description.get("service_ref"))).validate()
                    if (reference.service != self.service or reference.api_version != "v1" or
                            reference.target_id != self.target_id or reference.endpoint.address != self.endpoint):
                        raise ValueError("camera peer does not match the allocated endpoint and target grant")
                    self._client = Client.from_service(reference, runtime=self._runtime, local_target=self.target_id)
                    observation = self.reconcile(self._client, description, self.stop)
                    self.status.update(state="connected", instance_id=reference.instance_id, applied_revision=1)
                    self.status.pop("reason", None)
                    ceiling, last_log, suppressed = 0.25, None, 0
                    if observation is None:
                        self._wait()
                    else:
                        route, revision = observation
                        while not self.stop.is_set():
                            status = self._client.json(route + str(revision), method="GET", timeout=8)
                            revision = status["event_revision"]
                except Exception as error:
                    if self.stop.is_set():
                        break
                    self.status.update(state="unavailable", reason=str(error))
                    suppressed += 1
                    now = time.monotonic()
                    if last_log is None or now - last_log >= 10:
                        rospy.logwarn("%s unavailable: %s (%d failed attempts since last summary)",
                                      self.service, error, suppressed)
                        last_log, suppressed = now, 0
                    # A ready host with no registered source has no roster event
                    # until that source exists; use the same bounded backoff.
                    self._wait(random.uniform(0, ceiling))
                    ceiling = min(30, ceiling * 2)
                finally:
                    if self._client is not None:
                        self._client.close()
                        self._client = None
        finally:
            self._discovery.close()

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.stop.set()
        if hasattr(self, "_wake_write"):
            self.wake()
        # Cancel held calls before joining their sole worker.
        client = self._client
        if client is not None:
            client.close()
        self._discovery.close()
        if hasattr(self, "_thread"):
            self._thread.join()
        self._runtime.close()
        for name in ("_events", "_wake_read", "_wake_write"):
            fd = getattr(self, name, -1)
            if fd >= 0:
                os.close(fd)
                setattr(self, name, -1)

    def wake(self):
        try:
            os.write(self._wake_write, b"x")
        except BlockingIOError:
            # An unread wake already covers every newer desired revision.
            pass
