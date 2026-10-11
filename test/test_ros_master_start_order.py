#!/usr/bin/env python3
"""Exercise the shared camera/robot master watcher against a fake ROS master."""
import json
import os
from pathlib import Path
import random
import select
import socket
import subprocess
import tempfile
import threading
import time
import unittest
from xmlrpc.server import SimpleXMLRPCServer

from test_peer_start_order import resources


def private_port():
    for _ in range(200):
        port = random.randrange(20000, 40001)
        with socket.socket() as sock:
            try:
                sock.bind(("127.0.0.1", port))
            except OSError:
                continue
        return port
    raise RuntimeError("no private ROS master port available")


class Master:
    def __init__(self, port, identity):
        self.server = SimpleXMLRPCServer(("127.0.0.1", port), allow_none=True, logRequests=False)
        self.server.register_function(lambda caller: [1, "pid", identity], "getPid")
        self.server.register_function(lambda caller, name: [1, "run", str(identity)], "getParam")
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": .05})
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.thread.join()
        self.server.server_close()


class RosMasterStartOrderTest(unittest.TestCase):
    def exercise(self, duration):
        executable = os.environ.get("XGC_MASTER_FIXTURE")
        self.assertTrue(executable, "set XGC_MASTER_FIXTURE to the standalone C++ fixture")
        with tempfile.TemporaryDirectory(prefix="gazebo-master-test-") as directory:
            port = private_port()
            environment = dict(os.environ, ROS_MASTER_URI="http://127.0.0.1:%d" % port, ROS_HOME=directory)
            with (Path(directory) / "fixture.log").open("w+") as log:
                process = subprocess.Popen([executable], env=environment, stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE, stderr=log, universal_newlines=True)
                master = None
                def status(command="status"):
                    process.stdin.write(command + "\n")
                    process.stdin.flush()
                    self.assertTrue(select.select([process.stdout], [], [], 5)[0])
                    return json.loads(process.stdout.readline())
                def connected():
                    deadline = time.monotonic() + 35
                    while time.monotonic() < deadline:
                        state = status()
                        if state["first"] == state["second"] == 1:
                            return state
                        time.sleep(.05)
                    self.fail("master did not bind: " + str(state))
                try:
                    self.assertEqual(status("set 7")["first"], 0)
                    time.sleep(.5)
                    before = resources(process.pid)
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        time.sleep(min(.25, max(0, deadline - time.monotonic())))
                    state = status()
                    after = resources(process.pid)
                    log.flush()
                    log.seek(0)
                    lines = log.read().splitlines()
                    self.assertLessEqual(state["attempts"], 18 if duration == 60 else 8)
                    self.assertLessEqual(len(lines), 7 if duration == 60 else 2)
                    self.assertEqual(after[1:], before[1:])
                    if duration == 60:
                        self.assertLess(after[0] - before[0], 1.2)
                    master = Master(port, 101)
                    state = connected()
                    self.assertEqual(state["cancelled"], 0)
                    self.assertEqual(state["applied_revision"], 7)
                    master.close()
                    master = None
                    master = Master(port, 101)
                    time.sleep(.3)
                    self.assertEqual(status()["first"], 1)
                    master.close()
                    master = None
                    master = Master(port, 102)
                    self.assertEqual(process.wait(timeout=35), 0)
                    print(json.dumps({"peer": "ROS master", "outage_seconds": duration,
                        "cpu_seconds": after[0] - before[0], "attempts": state["attempts"],
                        "log_lines": len(lines), "threads": [before[1], after[1]],
                        "fds": [before[2], after[2]], "applied_revision": 7}))
                finally:
                    if process.poll() is None:
                        process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    process.stdin.close()
                    process.stdout.close()
                    if master is not None:
                        master.close()

    def test_late_master_and_replacement(self):
        self.exercise(1)

    @unittest.skipUnless(os.environ.get("XGC_LONG_TESTS") == "1", "60-second outage cell")
    def test_master_down_for_sixty_seconds(self):
        self.exercise(60)


if __name__ == "__main__":
    unittest.main()
