#!/usr/bin/env python3
"""
bt_bridge.py — HC-05 ↔ WebSocket Bridge
=========================================
Runs on your PC.  Pairs with the HC-05 COM port (Windows Bluetooth)
and exposes a local WebSocket so the browser dashboard can talk to
your STM32 line-follower robot.

Requirements
------------
    pip install pyserial websockets

Quick start
-----------
1.  Pair HC-05 with Windows (Settings → Bluetooth → Add device).
    It will appear as two COM ports; use the OUTGOING one.
2.  Run:   python bt_bridge.py
    (You will be shown a port picker if --port is omitted.)
3.  Open linefollower_dashboard_v2.html in Chrome / Edge.
4.  Click CONNECT on the dashboard.

Optional flags
--------------
    -p / --port  COM3          Specify COM port directly
    -b / --baud  9600          Baud rate (default 9600, match HC-05 AT config)
    -l / --list                List available COM ports and exit
"""

import asyncio
import argparse
import sys
import time
import threading
import os
import webbrowser
import http.server
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import serial
import serial.tools.list_ports
import websockets
import websockets.exceptions

# ── Configuration ────────────────────────────────────────────────────────────
WS_HOST      = "localhost"
WS_PORT      = 8765
HTTP_PORT    = 8080
DASHBOARD    = "linefollower_dashboard_v2.html"
DEFAULT_BAUD = 9600

# ── Globals ───────────────────────────────────────────────────────────────────
clients: set = set()
ser_conn: serial.Serial | None = None
_serial_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="serial")


# ── Helpers ───────────────────────────────────────────────────────────────────
def ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def cprint(msg: str, icon: str = "·") -> None:
    print(f"[{ts()}] {icon}  {msg}", flush=True)


async def broadcast(msg: str) -> None:
    """Send a text message to every connected browser client."""
    dead: set = set()
    for client in set(clients):          # iterate a copy to allow mutation
        try:
            await client.send(msg)
        except Exception:
            dead.add(client)
    clients.difference_update(dead)      # in-place removal — avoids rebinding the global


# ── WebSocket handler (one per browser tab) ───────────────────────────────────
async def ws_handler(websocket) -> None:
    global ser_conn

    clients.add(websocket)
    cprint(f"Browser connected  ({len(clients)} client(s))", "🌐")

    # If HC-05 is already up, tell this new client immediately
    if ser_conn and ser_conn.is_open:
        try:
            await websocket.send("__BRIDGE_CONNECTED__")
            cprint("Sent __BRIDGE_CONNECTED__ to new browser client", "📡")
        except Exception as e:
            cprint(f"Failed to send __BRIDGE_CONNECTED__: {e}", "⚠ ")
    else:
        cprint("Serial not ready yet — browser must wait for HC-05 to connect", "⏳")

    try:
        async for raw_msg in websocket:
            msg = raw_msg.strip()
            if not msg:
                continue
            if ser_conn and ser_conn.is_open:
                try:
                    ser_conn.write((msg + "\r\n").encode())
                    cprint(f"→ HC-05 : {msg}", "📤")
                except Exception as exc:
                    cprint(f"Serial write error: {exc}", "⚠ ")
            else:
                cprint(f"HC-05 not connected — dropping: {msg}", "⚠ ")

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        clients.discard(websocket)
        cprint(f"Browser disconnected ({len(clients)} client(s))", "🌐")


# ── Serial read loop ──────────────────────────────────────────────────────────
async def serial_loop(port: str, baud: int) -> None:
    """
    Buffered chunked reader — much more reliable than readline() over
    Bluetooth COM ports, which deliver data in unpredictable bursts.

    Also flushes partial lines (no \\n) after 300 ms of silence — this
    handles the case where HAL_UART_Transmit times out before sending \\n.
    """
    global ser_conn

    loop           = asyncio.get_event_loop()
    rx_buf         = b""
    last_byte_time = time.monotonic()   # tracks when we last got any bytes
    warned_partial = False              # avoid spamming the partial warning

    while True:
        # ── (Re-)connect ─────────────────────────────────────────────────────
        if ser_conn is None or not ser_conn.is_open:
            rx_buf         = b""  # clear stale bytes on reconnect
            last_byte_time = time.monotonic()
            warned_partial = False
            try:
                cprint(f"Connecting to {port} @ {baud} baud …", "🔌")
                # timeout=0 → non-blocking: read() returns immediately with
                # whatever bytes are in the OS buffer (may be empty b"").
                # This keeps the asyncio event loop fully responsive.
                ser_conn = serial.Serial(port, baud, timeout=0)
                cprint(f"HC-05 connected on {port} ✓", "✅")
                await broadcast("__BRIDGE_CONNECTED__")
            except serial.SerialException as exc:
                cprint(f"Cannot open {port}: {exc}", "❌")
                cprint("Retrying in 3 s …", "   ")
                await asyncio.sleep(3)
                continue

        # ── Read whatever is waiting, buffer, emit complete lines ─────────────
        try:
            # Non-blocking read: only pull bytes that are already in the
            # OS receive buffer.  If nothing is waiting we sleep 10 ms and
            # loop — this keeps the asyncio event loop free at all times.
            n = ser_conn.in_waiting
            if n > 0:
                # Run in our dedicated executor so pyserial's internal lock
                # never competes with the WebSocket coroutines.
                raw = await loop.run_in_executor(
                    _serial_executor, lambda: ser_conn.read(n)
                )
            else:
                raw = b""

            if raw:
                rx_buf        += raw
                last_byte_time = time.monotonic()
                warned_partial = False
                # Emit every complete \n-terminated line
                while b"\n" in rx_buf:
                    chunk, rx_buf = rx_buf.split(b"\n", 1)
                    line_str = chunk.decode("utf-8", errors="replace").strip()
                    if line_str:
                        cprint(f"← HC-05 : {line_str}", "📥")
                        await broadcast(line_str)
            else:
                # Nothing available — check for stale partial line in buffer
                stale = (time.monotonic() - last_byte_time) > 0.30   # 300 ms
                if rx_buf and stale:
                    partial = rx_buf.decode("utf-8", errors="replace").strip()
                    if partial:
                        if not warned_partial:
                            cprint(
                                "⚠  Partial line (no \\n) — fix: increase HAL_UART_Transmit"
                                " timeout to 200 ms, or raise UART baud to 115200",
                                "❌"
                            )
                            warned_partial = True
                        cprint(f"← HC-05 (partial): {partial}", "📥")
                        # Append \n so the dashboard parser can handle it
                        await broadcast(partial + "\n")
                    rx_buf = b""
                else:
                    # Nothing ready — yield to the event loop for 10 ms
                    # (keeps WebSocket pings/messages flowing smoothly)
                    await asyncio.sleep(0.010)

        except Exception as exc:
            cprint(f"Serial read error: {exc}", "⚠ ")
            try:
                ser_conn.close()
            except Exception:
                pass
            ser_conn = None
            rx_buf   = b""
            await broadcast("__BRIDGE_DISCONNECTED__")
            cprint("HC-05 disconnected — will retry …", "🔌")
            await asyncio.sleep(2)


# ── HTTP server (serves the dashboard so file:// WS restriction is bypassed) ───────
class _SilentHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args) -> None:
        pass  # suppress per-request noise


def start_http_server(directory: str) -> None:
    os.chdir(directory)
    with http.server.HTTPServer(("localhost", HTTP_PORT), _SilentHandler) as httpd:
        httpd.serve_forever()


# ── Main ────────────────────────────────────────────────────────────
async def main(port: str, baud: int) -> None:
    dashboard_dir = os.path.dirname(os.path.abspath(__file__))
    dashboard_url = f"http://localhost:{HTTP_PORT}/{DASHBOARD}"

    print(f"""
╭────────────────────────────────────────────────────────╮
│   HC-05 ⇔ WebSocket Bridge  v2.0                     │
├────────────────────────────────────────────────────────┤
│  Serial  : {port:<45}│
│  Baud    : {baud:<45}│
│  WS      : ws://localhost:{WS_PORT:<37}│
│  Browser : {dashboard_url:<45}│
╰────────────────────────────────────────────────────────╯

  ⚠  Do NOT open the HTML file directly from disk.
      Always use the URL above — Chrome blocks WebSocket
      connections from  file://  pages.

  Press Ctrl+C to stop.
""")

    # Start HTTP file server in background thread
    http_thread = threading.Thread(
        target=start_http_server, args=(dashboard_dir,), daemon=True
    )
    http_thread.start()
    cprint(f"HTTP server ready — serving dashboard at {dashboard_url}", "🌍")

    # Auto-open browser
    webbrowser.open(dashboard_url)
    cprint("Opening dashboard in default browser …", "📲")

    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        cprint(f"WebSocket server ready on ws://localhost:{WS_PORT}", "🚀")
        await serial_loop(port, baud)


# ── Entry point ───────────────────────────────────────────────────────────────
def pick_port() -> str:
    """Interactive COM-port picker."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print(
            "\n❌  No COM ports found.\n"
            "    Make sure HC-05 is paired in Windows Bluetooth settings first.\n"
            "    (Settings → Bluetooth → Add device)\n"
        )
        sys.exit(1)

    print("\n  Available COM ports:\n")
    for i, p in enumerate(ports):
        print(f"    [{i}]  {p.device:10}  {p.description}")
    print()

    raw = input("  Enter index or port name (e.g. 0 or COM3): ").strip()
    if raw.isdigit():
        idx = int(raw)
        if 0 <= idx < len(ports):
            return ports[idx].device
        print("❌  Invalid index.")
        sys.exit(1)
    return raw


def list_ports() -> None:
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No COM ports found.")
        return
    print("\n  COM Ports:\n")
    for p in ports:
        print(f"    {p.device:12}  {p.description}")
    print()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="HC-05 ↔ WebSocket bridge for the line-follower dashboard",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--port", "-p",
        help="COM port to use (e.g. COM3 on Windows, /dev/rfcomm0 on Linux)",
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=DEFAULT_BAUD,
        help=f"UART baud rate (default {DEFAULT_BAUD} — must match your HC-05 AT config)",
    )
    parser.add_argument(
        "--list", "-l",
        action="store_true",
        help="List available COM ports and exit",
    )
    args = parser.parse_args()

    if args.list:
        list_ports()
        sys.exit(0)

    port = args.port if args.port else pick_port()

    try:
        asyncio.run(main(port, args.baud))
    except KeyboardInterrupt:
        print(f"\n[{ts()}] Bridge stopped. Goodbye! 👋\n")
