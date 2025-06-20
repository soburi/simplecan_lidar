#!/usr/bin/env python3
"""
Simple COIN-D4/D4A data receiver and display based on full packet format.
"""
import serial
import argparse
import signal
import sys
import threading
import time

# Command definitions (4-byte commands)
START_CMD       = bytes([0xAA, 0x55, 0xF0, 0x0F])
END_CMD         = bytes([0xAA, 0x55, 0xF5, 0x0A])
HIGH_SPEED_CMD  = bytes([0xAA, 0x55, 0xF2, 0x0D])
LOW_SPEED_CMD   = bytes([0xAA, 0x55, 0xF1, 0x0E])

running = True
ser = None  # Serial object

def signal_handler(signum, frame):
    global running, ser
    running = False
    try:
        ser.write(END_CMD)
        ser.flush()
    except:
        pass
    sys.exit(0)

# Packet constants
HEADER = b"\xAA\x55"  # PH: 0x55AA little endian -> bytes 0xAA,0x55

# Bit masks for M&T byte
TYPE_MASK = 0x01  # bit0: packet type
FREQ_MASK = 0xFE  # bits1-7: scan frequency identifier


def parse_packet(packet):
    # packet: full bytes including header
    # PH already validated
    m_t = packet[2]
    #print("m_t", m_t)
    lsn = packet[3]
    fsa_raw = packet[4] | (packet[5] << 8)
    lsa_raw = packet[6] | (packet[7] << 8)
    cs = packet[8] | (packet[9] << 8)
    samples = packet[10:]
    # Determine packet type
    ptype = m_t & TYPE_MASK
    freq_id = (m_t & FREQ_MASK) >> 1
    # Convert angles to degrees
    start_angle = fsa_raw / 100.0
    end_angle = lsa_raw / 100.0
    points = []
    for i in range(lsn):
        idx = i * 3
        if idx + 3 > len(samples):
            break

        hi_ref = samples[idx] & 0x3
        dist = (samples[idx+1] >> 2) | (samples[idx+2] << 8)  # mm
        intensity = samples[idx] >> 2 | (samples[idx+1] << 6)

        # interpolate angle for this point
        if lsn > 1:
            angle = start_angle + (end_angle - start_angle) * i / (lsn - 1)
        else:
            angle = start_angle

        points.append((dist / 1000.0, angle, intensity))
    return {
        'type': 'start' if ptype == 1 else 'data',
        'frequency_id': freq_id,
        'start_angle': start_angle,
        'end_angle': end_angle,
        'points': points,
        'raw_checksum': cs
    }


def main():
    global ser
    parser = argparse.ArgumentParser(description="Control COIN-D4/D4A via pyserial")
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=230400)
    parser.add_argument('--speed', choices=['high', 'low'], default='high')
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baudrate, timeout=0.1)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("start thread")

    #reader = threading.Thread(target=read_loop, daemon=True)
    #reader.start()

    print("start cmd")

    ser.write(START_CMD)
    ser.flush()
    time.sleep(0.2)

    print("speed cmd")

    speed_cmd = HIGH_SPEED_CMD if args.speed == 'high' else LOW_SPEED_CMD
    ser.write(speed_cmd)
    ser.flush()

    buf = bytearray()
    #print(f"Listening on {args.port} @ {args.baud}bps. Ctrl+C to quit.")
    try:
        while True:
            byte = ser.read(1)
            if not byte:
                continue
            buf += byte
            #print(buf)

            # Process any complete frames
            while len(buf) >= 8:
                # Look for header
                idx = buf.find(HEADER)
                if idx < 0:
                    # drop old data
                    #buf.clear()
                    break
                #print(idx)
                #if idx > 0:
                #    print(buf[idx-7:idx])
                #    buf = buf[idx:]
                # Need at least 10 bytes header
                if len(buf) < (idx + 10):
                    break
                # Extract LSN to know full packet length
                lsn = buf[idx+3]
                packet_length = 10 + lsn * 3
                if len(buf) < idx+packet_length:
                    break

                #print(buf)
                #print("pos={0} ".format(idx))
                #print("hdr={0:x}{1:x} M&T={2:x} SP={3}".format(buf[0], buf[1], buf[2], buf[3]))
                packet = bytes(buf[idx:idx+packet_length])
                buf = buf[idx+packet_length:]
                #print('parse');
                parsed = parse_packet(packet)
                if parsed['type'] == 'start':
                    print(f"-- Start Packet freq_id={parsed['frequency_id']} points={len(parsed['points'])}")
                else:
                    for dist, angle, inten in parsed['points']:
                        #if angle < 10.0:
                        print(f"Distance: {dist:.3f} m, Angle: {angle:.2f}°, Intensity: {inten}")
                buf.clear()

    except KeyboardInterrupt:
        print("\nExiting.")
        ser.close()

if __name__ == '__main__':
    main()
