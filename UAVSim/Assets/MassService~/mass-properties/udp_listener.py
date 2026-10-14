"""
udp_listener.py

Standalone UDP server for the drone mass-properties calculation service.

Listens for a "compute mass properties" request from Unity (STL path +
motor/battery/electronics node list), runs the geometry + composition math
(the masscompose package, then mass_composer.py for the Unity-specific
eigen/quaternion/armLength step), and sends back the computed system center
of mass and inertia tensor.

This is intentionally a SEPARATE process/port from the existing drone_core
sync channel (port 5065). It does not import or depend on drone_core_sim.py
or physics_sim.py at all -- it only needs to agree with Unity on the same
Header byte values and wire format.

WIRE FORMAT (must match the C# side exactly):

Request (Unity -> Python), header byte value 40:
    [header: uint8 = 40]
    [density: float32, little-endian]   -- material density, kg/m^3. Must be > 0.
    [pathByteCount: uint32, little-endian]
    [stlPathUtf8: pathByteCount bytes]
    [nodeCount: uint32, little-endian]
    repeat nodeCount times:
        [nodeType: uint8]  -- 0=motor, 1=battery, 2=flight_controller, 3=companion_computer
        [x: float32][y: float32][z: float32][massKg: float32]  (17 bytes each)

Response (Python -> Unity), header byte value 41:
    [header: uint8 = 41]
    [comX: float32][comY: float32][comZ: float32]
    [principalIxx: float32][principalIyy: float32][principalIzz: float32]
    [quatX: float32][quatY: float32][quatZ: float32][quatW: float32]
    [armLength: float32]   -- avg distance from COM to motor nodes (name starts with "motor")
    Total: 1 + 3*4 + 3*4 + 4*4 + 1*4 = 45 bytes

    principalIxx/Iyy/Izz map directly to Rigidbody.inertiaTensor.
    The quaternion maps directly to Rigidbody.inertiaTensorRotation.
    This keeps ALL eigen-decomposition and rotation-matrix math on the
    Python side (already written and tested in mass_composer.py), so the
    C# side only needs to unpack floats and assign them -- no linear
    algebra required in Unity.

Error response, header byte value 255 (reserved for this service --
distinct from drone_core's own Header.error=0, since this is a separate
protocol channel):
    [header: uint8 = 255]
    [messageByteCount: uint32, little-endian]
    [messageUtf8: messageByteCount bytes]

BUNDLED COPY: this file was copied from the MassCompose repo
(src/mass-properties/udp_listener.py) into UAVSim, which launches it from a
bundled Python (see MassServiceLauncher.cs). Differences from that original:
  * binds 127.0.0.1 instead of 0.0.0.0 (Unity only uses loopback, and a
    wildcard bind is what normally triggers the Windows Firewall prompt)
  * optional --parent-pid: exit when that process (Unity) exits, so a crashed
    Unity cannot leave the service running
The wire format is unchanged.
"""

import argparse
import os
import socket
import struct
import sys
import threading
import time
import traceback

from masscompose import PointMass, load_mesh
from mass_composer import compute_unity_payload

HEADER_COMPUTE_MASS_PROPERTIES = 40
HEADER_MASS_PROPERTIES_RESULT = 41
HEADER_ERROR = 255

# Separate port from the existing drone_core sync channel (5065), to avoid
# any conflict with the existing student-facing protocol. Unity's C# side
# must send its request to this exact port. Loopback only: the client is
# always on this machine (CustomFrameLoader sends to 127.0.0.1).
LISTEN_IP = "127.0.0.1"
LISTEN_PORT = 5070

# Fallback used only if a request somehow sends density <= 0 (shouldn't
# happen if the C# side validates its UI input, but the server should not
# trust the network blindly). PETG, a common 3D-printed frame material.
FALLBACK_DENSITY_KG_PER_M3 = 1250.0


def _parse_request(data: bytes) -> tuple:
    """
    Parses a mass-properties request payload (excluding the header byte,
    which the caller has already read/checked).

    Returns: (density: float, stl_path: str, nodes: list[PointMass])
    Raises: struct.error or IndexError if the payload is malformed/truncated,
            or ValueError if a node's mass is negative or non-finite --
            PointMass rejects those at construction, which is stricter than
            this parser used to be (a NaN mass off the wire previously
            travelled all the way into the response as a NaN payload).
    """
    offset = 0

    [density] = struct.unpack_from("<f", data, offset)
    offset += 4

    [path_byte_count] = struct.unpack_from("<I", data, offset)
    offset += 4

    stl_path_bytes = data[offset:offset + path_byte_count]
    if len(stl_path_bytes) != path_byte_count:
        raise ValueError(
            f"Truncated STL path: expected {path_byte_count} bytes, got {len(stl_path_bytes)}"
        )
    stl_path = stl_path_bytes.decode("utf-8")
    offset += path_byte_count

    [node_count] = struct.unpack_from("<I", data, offset)
    offset += 4

    NODE_TYPE_LABELS = {0: "motor", 1: "battery", 2: "flight_controller", 3: "companion_computer"}

    nodes = []
    for i in range(node_count):
        [node_type] = struct.unpack_from("<B", data, offset)
        offset += 1

        chunk = data[offset:offset + 16]
        if len(chunk) != 16:
            raise ValueError(
                f"Truncated node data at node index {i}: expected 16 bytes, got {len(chunk)}"
            )
        x, y, z, mass = struct.unpack("<ffff", chunk)
        label = NODE_TYPE_LABELS.get(node_type, f"unknown_type_{node_type}")
        nodes.append(
            PointMass(position=[x, y, z], mass=mass, name=f"{label}_{i}")
        )
        offset += 16

    return density, stl_path, nodes


def _build_result_payload(payload: dict) -> bytes:
    """
    Packs a compute_unity_payload() result dict into the 45-byte response
    payload: header, COM (3), principal moments (3), quaternion (4), armLength (1).
    """
    com = payload["com"]
    pm = payload["principal_moments"]
    qx, qy, qz, qw = payload["quaternion"]
    arm_length = payload["arm_length"]
    return struct.pack(
        "<Bfffffffffff",
        HEADER_MASS_PROPERTIES_RESULT,
        com[0], com[1], com[2],
        pm[0], pm[1], pm[2],
        qx, qy, qz, qw,
        arm_length,
    )


def _build_error_payload(message: str) -> bytes:
    message_bytes = message.encode("utf-8")
    return struct.pack("<BI", HEADER_ERROR, len(message_bytes)) + message_bytes


def handle_request(data: bytes) -> bytes:
    """
    Handles one raw incoming UDP payload (including its header byte) and
    returns the raw bytes to send back. Pure function, no socket I/O --
    kept separate from run_server() so it can be unit tested directly.
    """
    if len(data) < 1:
        return _build_error_payload("Empty packet received")

    header = data[0]
    if header != HEADER_COMPUTE_MASS_PROPERTIES:
        return _build_error_payload(f"Unexpected header byte: {header}")

    try:
        density, stl_path, nodes = _parse_request(data[1:])
        if density <= 0:
            print(
                f">> WARNING: received non-positive density ({density}); "
                f"falling back to {FALLBACK_DENSITY_KG_PER_M3} kg/m^3"
            )
            density = FALLBACK_DENSITY_KG_PER_M3
        # mesh_units is pinned to "mm" because the wire format has no units
        # field: Unity sends a path and a density, and the STL is written by
        # the student's CAD export in millimetres. Declaring it here rather
        # than assuming it inside the loader is the whole point of the
        # masscompose API -- if the protocol ever grows a units field, this
        # is the single line that reads it.
        frame = load_mesh(stl_path, mesh_units="mm", density=density)
        payload = compute_unity_payload(frame, nodes)
        return _build_result_payload(payload)
    except Exception as e:
        # Catch-all is intentional here: malformed requests, missing files,
        # non-watertight meshes, etc. should all reach the client as a
        # readable error message rather than silently crashing this
        # long-running server process.
        traceback.print_exc()
        return _build_error_payload(str(e))


def _suppress_udp_conn_reset(sock: socket.socket) -> None:
    """
    Windows only: stop stale ICMP port-unreachable notifications from killing
    the server.

    When a datagram we sent provokes an ICMP port-unreachable -- the client
    closed its socket after getting its reply, i.e. Unity exited -- Winsock
    reports it as WSAECONNRESET/10054 on the NEXT recv/recvfrom on this socket.
    It refers to a client that is already gone and means nothing for UDP
    correctness, but left alone it terminates run_server(). SIO_UDP_CONNRESET
    turns that behaviour off at the source.

    Best-effort only. This is an optimisation that keeps the error from being
    raised in the first place; the ConnectionResetError handler in run_server()
    is what actually guarantees survival. Never let this stop the server from
    starting.

    No-op on Linux and macOS.
    """
    if sys.platform != "win32":
        return

    # This CANNOT go through sock.ioctl(). CPython's socket.ioctl only accepts
    # SIO_RCVALL, SIO_KEEPALIVE_VALS and SIO_LOOPBACK_FAST_PATH and raises
    # ValueError("invalid ioctl command") for anything else -- and the socket
    # module never defines SIO_UDP_CONNRESET on any platform, so a
    # hasattr(socket, "SIO_UDP_CONNRESET") guard is a permanent no-op. Call
    # Winsock's WSAIoctl directly instead, with the literal control code:
    #   SIO_UDP_CONNRESET = _WSAIOW(IOC_VENDOR, 12)
    #                     = IOC_IN | IOC_VENDOR | 12
    #                     = 0x80000000 | 0x18000000 | 12 = 0x9800000C
    SIO_UDP_CONNRESET = 0x9800000C

    try:
        import ctypes
        import ctypes.wintypes as wintypes

        # SOCKET is UINT_PTR: 64-bit under 64-bit Python, 32-bit under 32-bit.
        socket_t = (ctypes.c_uint64 if ctypes.sizeof(ctypes.c_void_p) == 8
                    else ctypes.c_uint32)

        ws2_32 = ctypes.windll.ws2_32
        WSAIoctl = ws2_32.WSAIoctl
        WSAIoctl.argtypes = [
            socket_t, wintypes.DWORD,
            ctypes.c_void_p, wintypes.DWORD,      # in buffer
            ctypes.c_void_p, wintypes.DWORD,      # out buffer (unused)
            ctypes.POINTER(wintypes.DWORD),       # bytes returned
            ctypes.c_void_p, ctypes.c_void_p,     # overlapped, completion
        ]
        WSAIoctl.restype = ctypes.c_int

        disable = wintypes.DWORD(0)   # FALSE -- stop reporting the resets
        returned = wintypes.DWORD(0)
        rc = WSAIoctl(
            sock.fileno(), SIO_UDP_CONNRESET,
            ctypes.byref(disable), ctypes.sizeof(disable),
            None, 0,
            ctypes.byref(returned), None, None,
        )
        if rc != 0:
            print(f">> WARNING: SIO_UDP_CONNRESET failed "
                  f"(WSAGetLastError={ws2_32.WSAGetLastError()}); relying on "
                  f"the ConnectionResetError handler instead")
    except Exception as e:
        # Catch-all is deliberate, same reasoning as handle_request(): a
        # missing ws2_32, a restricted ctypes build or an unexpected ValueError
        # must degrade to "keep the exception handler" -- never to a server
        # that refuses to start.
        print(f">> WARNING: could not disable SIO_UDP_CONNRESET ({e}); "
              f"relying on the ConnectionResetError handler instead")


def _wait_for_process_exit(pid: int) -> None:
    """
    Blocks until process `pid` has exited (or returns at once if it does not
    exist).

    Windows: opens a SYNCHRONIZE handle and waits on it. Holding the handle
    pins the process object, so a recycled PID cannot be mistaken for the
    original parent. Do NOT use os.kill(pid, 0) here: on Windows signal 0 is
    CTRL_C_EVENT, not an existence check.

    Elsewhere: polls os.kill(pid, 0) every 2 s.
    """
    if sys.platform == "win32":
        import ctypes
        import ctypes.wintypes as wintypes

        SYNCHRONIZE = 0x00100000
        INFINITE = 0xFFFFFFFF
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        kernel32.OpenProcess.restype = wintypes.HANDLE
        kernel32.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        kernel32.WaitForSingleObject.restype = wintypes.DWORD

        handle = kernel32.OpenProcess(SYNCHRONIZE, False, pid)
        if not handle:
            return
        kernel32.WaitForSingleObject(handle, INFINITE)
        return

    while True:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return
        except PermissionError:
            pass  # exists, owned by someone else
        time.sleep(2.0)


def _start_parent_watch(pid: int) -> None:
    """
    Exits this whole process once `pid` exits. Runs on a daemon thread
    because the main thread sits in a blocking recvfrom(). os._exit rather
    than sys.exit, which would only end the watcher thread.
    """
    def watch() -> None:
        _wait_for_process_exit(pid)
        print(f">> Parent process {pid} has exited; shutting down.", flush=True)
        os._exit(0)

    threading.Thread(target=watch, name="parent-watch", daemon=True).start()


def run_server() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _suppress_udp_conn_reset(sock)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f">> Mass properties service listening on {LISTEN_IP}:{LISTEN_PORT}")

    while True:
        # Belt-and-braces alongside SIO_UDP_CONNRESET above: older Windows
        # builds, and sockets handed to us by some launchers, can still raise
        # here. Never fatal -- drop the stale notification and keep listening.
        try:
            data, addr = sock.recvfrom(65535)
        except ConnectionResetError:
            print(">> (debug) ignored stale ConnectionResetError from a closed client")
            continue

        print(f">> Received {len(data)} bytes from {addr}")
        response = handle_request(data)

        # Same story on the way out: the client may already be gone by the time
        # we reply (Unity stopped, scene reloaded). Losing one reply is fine;
        # dying is not.
        try:
            sock.sendto(response, addr)
        except ConnectionResetError:
            print(f">> (debug) client {addr} went away before the reply was sent")
            continue

        print(f">> Sent {len(response)} bytes to {addr}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone mass-properties UDP service")
    parser.add_argument(
        "--parent-pid", type=int, default=None,
        help="exit when the process with this PID exits (Unity passes its own PID)",
    )
    args = parser.parse_args()
    if args.parent_pid is not None:
        _start_parent_watch(args.parent_pid)

    try:
        run_server()
    except KeyboardInterrupt:
        print("\n>> Shutting down.")
        sys.exit(0)