#!/usr/bin/env python3
"""
WebHID Bridge Server for MindCloud World Fly Simulator

Bridges USB HID devices (RC transmitters) to the browser via WebSocket.
Listens on ws://localhost:8766

Usage:
    python3 hid_server.py

Prerequisites:
    sudo apt-get install python3-hid python3-websockets
    # Plus udev rules for non-root HID access (see setup_udev.sh)
"""

import asyncio
import json
import threading
import sys

try:
    import hid
except ImportError:
    print("ERROR: python3-hid not installed. Run: sudo apt-get install python3-hid")
    sys.exit(1)

try:
    import websockets
except ImportError:
    print("ERROR: python3-websockets not installed. Run: sudo apt-get install python3-websockets")
    sys.exit(1)

HID_WS_PORT = 8766


class HIDBridge:
    def __init__(self):
        self._device = None
        self._device_id = None
        self._running = False
        self._reader_thread = None

    def list_devices(self):
        devices = []
        seen = set()
        for d in hid.enumerate():
            path = d['path'].decode() if isinstance(d['path'], bytes) else str(d['path'])
            if path in seen:
                continue
            seen.add(path)
            name = (d.get('product_string') or '').strip()
            if not name:
                name = f"HID {d['vendor_id']:04X}:{d['product_id']:04X}"
            manufacturer = (d.get('manufacturer_string') or '').strip()
            devices.append({
                'id': path,
                'vendorId': d['vendor_id'],
                'productId': d['product_id'],
                'productName': name,
                'manufacturerName': manufacturer,
                'usagePage': d.get('usage_page', 0),
                'usage': d.get('usage', 0),
                'collections': [],
            })
        return devices

    def open_device(self, device_id, loop, report_queue):
        self.close_device()
        path = device_id.encode() if isinstance(device_id, str) else device_id
        try:
            dev = hid.device()
            dev.open_path(path)
            self._device = dev
            self._device_id = device_id
            self._running = True

            def read_loop():
                while self._running:
                    try:
                        data = dev.read(64, 50)
                        if data and self._running:
                            asyncio.run_coroutine_threadsafe(
                                report_queue.put({
                                    'type': 'report',
                                    'reportId': data[0],
                                    'data': list(data[1:]),
                                }),
                                loop,
                            )
                    except Exception as e:
                        if self._running:
                            print(f"HID read error: {e}")
                        break

            self._reader_thread = threading.Thread(target=read_loop, daemon=True)
            self._reader_thread.start()
            return True
        except Exception as e:
            return str(e)

    def close_device(self):
        self._running = False
        if self._device:
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None
            self._device_id = None


bridge = HIDBridge()


async def handler(websocket, path=None):
    report_queue = asyncio.Queue()
    print(f"Browser connected from {websocket.remote_address}")

    # Map response type -> (resolve, reject) for pending request-response pairs
    pending = {}

    async def send(msg):
        try:
            await websocket.send(json.dumps(msg))
        except Exception:
            pass

    async def forward_reports():
        while True:
            try:
                report = await report_queue.get()
                await send(report)
            except Exception:
                break

    report_forwarder = asyncio.ensure_future(forward_reports())

    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue

            t = msg.get('type')

            if t == 'list':
                devices = bridge.list_devices()
                print(f"Listing {len(devices)} HID device(s)")
                await send({'type': 'devices', 'devices': devices})

            elif t == 'open':
                device_id = msg.get('id', '')
                loop = asyncio.get_event_loop()
                result = bridge.open_device(device_id, loop, report_queue)

                if result is True:
                    # Find full device info
                    info = next(
                        (d for d in bridge.list_devices() if d['id'] == device_id),
                        {'productName': 'HID Device', 'vendorId': 0,
                         'productId': 0, 'collections': []},
                    )
                    print(f"Opened: {info['productName']} "
                          f"({info['vendorId']:04X}:{info['productId']:04X})")
                    await send({
                        'type': 'opened',
                        'id': device_id,
                        'productName': info['productName'],
                        'vendorId': info['vendorId'],
                        'productId': info['productId'],
                        'collections': info['collections'],
                    })
                else:
                    print(f"Failed to open device: {result}")
                    await send({'type': 'error', 'message': str(result)})

            elif t == 'close':
                bridge.close_device()
                print("Device closed by browser")
                await send({'type': 'closed'})

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        report_forwarder.cancel()
        bridge.close_device()
        print("Browser disconnected")


async def main():
    print(f"WebHID Bridge Server  ws://localhost:{HID_WS_PORT}")
    print("Waiting for browser connection...")
    async with websockets.serve(handler, "localhost", HID_WS_PORT):
        await asyncio.Future()  # run forever


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped.")
