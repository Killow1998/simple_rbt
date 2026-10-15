"""Passive IPC telemetry for native-input acceptance; sends no control requests."""
import json
import socket
import struct
import threading
import time
import uuid


def read_exact(connection, count):
    data = bytearray()
    while len(data) < count:
        chunk = connection.recv(count-len(data))
        if not chunk:
            raise EOFError
        data.extend(chunk)
    return bytes(data)


class Observer:
    def __init__(self, directory, role="gui"):
        self.lock = threading.Lock()
        self.metadata, self.velocity, self.closed = {}, [0, 0, 0], False
        self.connections = []
        hello = json.dumps({"role": role, "token": uuid.uuid4().hex,
                            "subscriptions": ["metadata", "odom"]}).encode()
        for name in ("control", "data"):
            connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            connection.connect(str(directory / (name+".sock")))
            self.connections.append(connection)
            connection.sendall(struct.pack("<4sHHQII", b"SRBT", 1, 1, 0, len(hello), 0)+hello)
            if name == "control":
                kind, _, data = self.packet(connection)
                assert kind == 1
                self.metadata = json.loads(data).get("metadata", {})
        for connection in self.connections:
            threading.Thread(target=self.read, args=(connection,), daemon=True).start()

    @staticmethod
    def packet(connection):
        magic, version, kind, stamp, count, reserved = struct.unpack("<4sHHQII", read_exact(connection, 24))
        assert magic == b"SRBT" and version == 1 and reserved == 0 and count <= 32*1024*1024
        return kind, stamp, read_exact(connection, count)

    def read(self, connection):
        try:
            while not self.closed:
                kind, _, data = self.packet(connection)
                with self.lock:
                    if kind == 11:
                        self.metadata = json.loads(data)
                    elif kind == 7:
                        self.velocity = list(struct.unpack("<13d", data)[7:10])
        except (EOFError, OSError):
            pass

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.metadata)), self.velocity[:]

    def wait(self, predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            metadata, velocity = self.snapshot()
            if predicate(metadata, velocity):
                return metadata, velocity
            time.sleep(.05)
        raise RuntimeError("Native input did not produce expected core telemetry: " + repr(self.snapshot()))

    def close(self):
        self.closed = True
        for connection in self.connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
