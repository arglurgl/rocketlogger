"""
Simple BLE remote for RocketLogger.
Requires: bleak (pip install bleak)

Usage examples:
  python remote_logger.py --addr AA:BB:CC:DD:EE:FF --transfer --out flight.csv
  python remote_logger.py --addr AA:BB:CC:DD:EE:FF --arm

Protocol (matches device expectations):
 - Commands are sent as three-byte packets: b'\x00' + b'T' + sub
   where sub is one of:
     b'A' = Arm
     b'T' = Transfer log
     b'S' = Stop/abort
 - Device will stream text:
     "LOG_START\n"
     "time_seconds,altitude_m\n"  (one per sample)
     "LOG_END\n"
"""
import argparse
import asyncio
from bleak import BleakClient, BleakScanner

# Nordic UART Service (common Adafruit / UART-over-BLE)
UART_SERVICE = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # write here
UART_TX_CHAR = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # notify here

async def run(addr, do_arm, do_transfer, do_stop, do_status, outfile):
    if not addr:
        print("Scanning for RocketLogger device...")
        devices = await BleakScanner.discover(timeout=5.0)
        if not devices:
            print("No BLE devices found")
            return
        
        # Find first device named "rocketlogger"
        rocketlogger = next((d for d in devices if d.name and "rocketlogger" in d.name.lower()), None)
        if not rocketlogger:
            print("RocketLogger device not found!")
            print("Available devices:")
            for d in devices:
                print(f"  - {d.name or '(unnamed)'} ({d.address})")
            return
        
        addr = rocketlogger.address
        print(f"Found RocketLogger: {rocketlogger.name} ({addr})")

    received = []
    in_log = False
    buffer = ""                       # buffer for partial lines
    transfer_done = asyncio.Event()   # signalled when LOG_END seen

    # status reply state
    status_done = asyncio.Event()
    status_line = None

    def handle_notify(sender, data):
        nonlocal in_log, received, buffer, transfer_done, status_done, status_line
        text = data.decode('utf-8', errors='ignore')
        buffer += text

        # debug
        #print(f"[raw] received {len(text)} bytes: [{text}], buffer now: {repr(buffer)}")

        # extract full lines from the buffer
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            l = line.rstrip('\r')

            # status line (single-line reply)
            if l.startswith("STATUS,"):
                status_line = l
                print("[remote] STATUS reply:", l)
                status_done.set()
                continue

            if l == "LOG_START":
                in_log = True
                received = []
                transfer_done.clear()
                print("[remote] LOG_START")
            elif l == "LOG_END":
                in_log = False
                print("[remote] LOG_END, {} lines".format(len(received)))
                if outfile:
                    with open(outfile, "w") as fh:
                        fh.write("\n".join(received) + ("\n" if len(received) > 0 else ""))
                    print("[remote] Written to", outfile)
                transfer_done.set()
            else:
                if in_log:
                    received.append(l)
                    print("[log] ", l)

    async with BleakClient(addr) as client:
        svcs = client.services
        if UART_RX_CHAR not in svcs.characteristics and UART_TX_CHAR not in svcs.characteristics:
            # Some stacks require lower-case UUIDs or discovery; still continue
            pass

        # Register notify handler BEFORE sending any commands
        await client.start_notify(UART_TX_CHAR, handle_notify)

        # Helper to send command: three bytes: 0x00' + b'T' + sub
        async def send_cmd(sub_char: bytes):
            pkt = b'\x00' + b'T' + sub_char
            await client.write_gatt_char(UART_RX_CHAR, pkt)
            print("[remote] Sent command:", pkt)

        try:
            if do_arm:
                await send_cmd(b'A')
            if do_stop:
                await send_cmd(b'S')
            if do_transfer:
                if outfile:
                    open(outfile, "w").close()
                transfer_done.clear()
                await send_cmd(b'T')
                try:
                    await asyncio.wait_for(transfer_done.wait(), timeout=60.0)
                except asyncio.TimeoutError:
                    print("[remote] Transfer timed out waiting for samples")
                    print("[remote] Received {} lines before timeout".format(len(received)))

            if do_status:
                # request status and wait for single-line reply
                status_done.clear()
                status_line = None
                await send_cmd(b'I')
                try:
                    await asyncio.wait_for(status_done.wait(), timeout=5.0)
                except asyncio.TimeoutError:
                    print("[remote] Status request timed out")
                else:
                    print("[remote] Status:", status_line)

            await asyncio.sleep(0.5)
        finally:
            await asyncio.sleep(0.1)
            await client.stop_notify(UART_TX_CHAR)

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--addr", help="BLE device address (optional, will scan if omitted)")
    p.add_argument("--arm", action="store_true", help="Send ARM command")
    p.add_argument("--transfer", action="store_true", help="Request log transfer")
    p.add_argument("--stop", action="store_true", help="Send STOP/abort")
    p.add_argument("--status", action="store_true", help="Request device status")
    p.add_argument("--out", dest="outfile", default="flight.csv", help="Output CSV file for log")
    return p.parse_args()

def main():
    args = parse_args()
    asyncio.run(run(args.addr, args.arm, args.transfer, args.stop, args.status, args.outfile))

if __name__ == "__main__":
    main()
