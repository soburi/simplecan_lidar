#!/usr/bin/env python3
# Copyright (c) 2016 Alex Bencz
# Copyright (c) 2019 Konsulko Group, smurray@konsulko.com
# Copyright (c) 2020 The Linux Foundation, jsmoeller@linuxfoundation.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
# of the Software, and to permit persons to whom the Software is furnished to do
# so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#
# CANSocket from:
#
# https://github.com/abencz/python_socketcan/blob/master/python_socketcan_example.py
#

import sys
import socket
import argparse
import struct
import errno
import threading
import time
import math
import serial

class CANSocket(object):
    FORMAT = "<IB3x8s"
    FD_FORMAT = "<IB3x64s"

    def __init__(self, interface=None):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        if interface is not None:
            self.bind(interface)

    def bind(self, interface):
        self.sock.bind((interface,))
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)

    def send(self, can_id, data, flags=0):
        can_id = can_id | flags
        can_pkt = struct.pack(self.FORMAT, can_id, len(data), data)
        self.sock.send(can_pkt)

    def sendfd(self, can_id, data, flags=0):
        can_id = can_id | flags
        datafd = data.ljust(64, b'\x00');
        can_pkt = struct.pack(self.FD_FORMAT, can_id, len(datafd), datafd)
        self.sock.send(can_pkt)

    def recv(self, flags=0):
        can_pkt = self.sock.recv(72)

        if len(can_pkt) == 16:
            can_id, length, data = struct.unpack(self.FORMAT, can_pkt)
        else:
            can_id, length, data = struct.unpack(self.FD_FORMAT, can_pkt)

        can_id &= socket.CAN_EFF_MASK
        return (can_id, data[:length])

class VehicleSimulator(object):
    DEFAULT_IDLE_RPM = 600

    def __init__(self):
        self.CRUISEMODE = False
        self.CRUISEACTIVE = False
        self.CRUISESPEED = 0
        self.CRUISERPM = 0
        self.freq = 10
        self.vehicle_speed = 0
        self.engine_speed = self.DEFAULT_IDLE_RPM
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.lock = threading.Lock()

    def reset(self):
        with self.lock:
            self.vehicle_speed = 0
            self.engine_speed = self.DEFAULT_IDLE_RPM

    def start(self):
        self.thread.start()

    def get_engine_speed(self):
        with self.lock:
            return int(self.engine_speed)

    def get_vehicle_speed(self):
        with self.lock:
            return int(self.vehicle_speed)

    def accelerate(self, target_speed, target_rpm, duration, bycruise = False):
        if target_speed <= self.vehicle_speed:
            return
        v = (target_speed - self.vehicle_speed) / (duration * self.freq)
        r = (target_rpm - self.engine_speed) / (duration * self.freq)
        while self.vehicle_speed < target_speed and (not self.CRUISEACTIVE or bycruise):
            with self.lock:
                self.vehicle_speed += v;
                self.engine_speed += r;
            time.sleep(1 / self.freq)

    def brake(self, target_speed, target_rpm, duration, bycruise = False):
        if target_speed >= self.vehicle_speed:
            return
        v = (self.vehicle_speed - target_speed) / (duration * self.freq)
        r = (self.engine_speed - target_rpm) / (duration * self.freq)
        while self.vehicle_speed > target_speed and (not self.CRUISEACTIVE or bycruise):
            with self.lock:
                self.vehicle_speed -= v;
                self.engine_speed -= r;
            time.sleep(1 / self.freq)

    def increase(self, bycruise = True):
        if self.CRUISEACTIVE:
            target_speed = self.vehicle_speed + 5
            target_rpm = self.engine_speed * 1.1
            self.accelerate(target_speed, target_rpm, 2, bycruise)

    def decrease(self, bycruise = True):
        if self.CRUISEACTIVE:
            target_speed = self.vehicle_speed - 5
            target_rpm = self.engine_speed * 0.9
            self.brake(target_speed, target_rpm, 2, bycruise)

    def resume(self, bycruise = True):
        target_speed = self.CRUISESPEED
        target_rpm = self.CRUISERPM
        current_speed = self.get_vehicle_speed()
        if target_speed > current_speed:
            self.accelerate(target_speed, target_rpm, 2, bycruise)
        else:
            self.brake(target_speed, target_rpm, 2, bycruise)

    def run(self):
        while True:
            if not self.CRUISEACTIVE:
                self.accelerate(80, 3000, 5)
                self.accelerate(104, 4000, 3)
                self.brake(80, 3000, 3)
                self.accelerate(104, 4000, 6)
                self.brake(40, 2000, 4)
                self.accelerate(90, 3000, 5)
                self.brake(1, 650, 5)
                if not self.CRUISEACTIVE:
                    self.reset()
            time.sleep(5)

class DiagnosticMessageHandler(object):
    def __init__(self, can_sock, simulator, verbose=False):
        self.can_sock = can_sock
        self.simulator = simulator
        self.verbose = verbose
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        while True:
            try:
                can_id, data = self.can_sock.recv()
                #print('%03X#%s' % (can_id, ''.join(format(x, '02X') for x in data)))
                if can_id == 0x7df:
                    # OBD-II request
                    if data[1] == 0x01 and data[2] == 0x0C:
                        # Engine speed
                        speed = self.simulator.get_engine_speed()
                        #print('engine speed = %d' % speed)
                        if speed > 16383.75:
                            speed = 16383.75
                        reply = [ 0x04, 0x41, 0x0C ]
                        reply.append(4 * speed // 256)
                        reply.append(4 * speed % 256)
                        # pad remaining bytes to make 8
                        reply.append(0)
                        reply.append(0)
                        reply.append(0)
                        self.can_sock.send(0x7e8, bytes(reply), 0)
                    elif data[1] == 0x01 and data[2] == 0x0D:
                        # Vehicle speed
                        speed = int(self.simulator.get_vehicle_speed()) % 256
                        #print('vehicle speed = %d' % speed)
                        reply = [ 0x03, 0x41, 0x0D ]
                        reply.append(speed)
                        # pad remaining bytes to make 8
                        reply.append(0)
                        reply.append(0)
                        reply.append(0)
                        reply.append(0)
                        self.can_sock.send(0x7e8, bytes(reply), 0)
            except e:
                print(e)

class SteeringWheelMessageHandler(object):
    def __init__(self, can_sock, simulator, verbose=False):
        self.can_sock = can_sock
        self.simulator = simulator
        self.verbose = verbose
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.buttonpressed = False
        self.buttonenabled = False
        self.buttoncancel = False
        self.buttondec = False
        self.buttoninc = False
        self.cruisemode = False
        self.cruiseactive = False

    def start(self):
        self.thread.start()

    def run(self):
        while True:
            can_id, data = self.can_sock.recv()
            #print('%03X#%s' % (can_id, ''.join(format(x, '02X') for x in data)))
            if can_id == 0x21:
                #print('%03X#%s' % (can_id, ''.join(format(x, '02X') for x in data)))
                if data:
                    #if data[6]:
                        #print('data6: %02X' % (data[6]))
                    if data[6] == 0x80 and not self.buttonpressed:
                        # we do skip any further lin messages
                        # two buttons at the same time won't work
                        # (aka unlikely w/o twisting fingers)
                        self.buttonpressed = True
                        self.buttonenabled = True
                    if data[6] == 0x08 and not self.buttonpressed:
                        self.buttonpressed = True
                        self.buttoncancel = True
                    if data[6] == 0x10 and not self.buttonpressed:
                        self.buttonpressed = True
                        self.buttondec = True
                    if data[6] == 0x40 and not self.buttonpressed:
                        self.buttonpressed = True
                        self.buttoninc = True
                    if data[6] == 0x00 and self.buttonpressed:
                        #now handle it as the button was released
                        if self.buttonenabled:
                            self.buttonenabled = False
                            self.cruisemode = not self.cruisemode
                            #print("set cruisemode to %s" % self.cruisemode)
                            self.simulator.CRUISEMODE = self.cruisemode
                            # disable/reset all if going off
                            if not self.cruisemode:
                                self.cruiseactive = False
                                self.simulator.CRUISEACTIVE = self.cruiseactive
                                self.simulator.CRUISESPEED = 0
                                self.simulator.CRUISERPM = 0
                            #print("set cruiseactive to %s" % self.cruiseactive)
                        if self.buttoncancel:
                            self.buttoncancel = False
                            self.simulator.CRUISESPEED = self.simulator.get_vehicle_speed()
                            self.simulator.CRUISERPM = self.simulator.get_engine_speed()
                            #print("set cruisespeed to %d" % self.simulator.CRUISESPEED )
                            #print("set cruiserpm to %d" % self.simulator.CRUISERPM )
                            self.cruiseactive = False
                            #print("set cruiseactive to %s" % self.cruiseactive )
                            self.simulator.CRUISEACTIVE = self.cruiseactive
                        if self.buttondec:
                            self.buttondec = False
                            if self.cruiseactive:
                                #print("decrease")
                                self.simulator.decrease()
                            else:
                                # set speed
                                #print("set speed")
                                self.simulator.CRUISESPEED = self.simulator.get_vehicle_speed()
                                self.simulator.CRUISERPM = self.simulator.get_engine_speed()
                                #print("set cruisespeed to %d" % self.simulator.CRUISESPEED )
                                #print("set cruiserpm to %d" % self.simulator.CRUISERPM )
                                self.cruiseactive = not self.cruiseactive
                                #print("set cruiseactive to %s" % self.cruiseactive )
                                self.simulator.CRUISEACTIVE = self.cruiseactive
                        if self.buttoninc:
                            self.buttoninc = False
                            if self.cruiseactive:
                                #print("increase")
                                self.simulator.increase()
                            else:
                                if self.simulator.CRUISESPEED > 0:
                                    # resume
                                    self.cruiseactive = not self.cruiseactive
                                    self.simulator.CRUISEACTIVE = self.cruiseactive
                                    #print("set cruiseactive to %s" % self.cruiseactive )
                                    #print("resume")
                                    self.simulator.resume()
                        self.buttonpressed = False


class StatusMessageSender(object):
    def __init__(self, can_sock, simulator, verbose=False):
        self.can_sock = can_sock
        self.simulator = simulator
        self.verbose = verbose
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        while True:
            # Engine speed
            speed = self.simulator.get_engine_speed()
            if self.verbose:
                print('engine speed = %d' % speed)
            if speed > 16383.75:
                speed = 16383.75
            # Message is 1 byte unknown, 1 byte fuel level, 2 bytes engine speed (4x), fuel low @ bit 55
            msg = [ 0, 0 ]
            speed *= 4
            msg.append(speed // 256)
            msg.append(speed % 256)
            # pad remaining bytes to make 8
            msg.append(0)
            msg.append(0)
            msg.append(0)
            msg.append(0)
            #print("can_sock send 0x3d9", file=sys.stderr)
            self.can_sock.send(0x3d9, bytes(msg), 0)

            # Vehicle speed
            speed = int(self.simulator.get_vehicle_speed()) % 256
            if self.verbose:
                print('vehicle speed = %d' % speed)
            # Message is 15 bits speed (64x), left aligned
            msg = [ ]
            # Note: extra 2x to yield required left-alignment
            speed *= 128
            msg.append(speed // 256)
            msg.append(speed % 256)
            # pad remaining bytes to make 8
            msg.append(0)
            msg.append(0)
            msg.append(0)
            msg.append(0)
            msg.append(0)
            msg.append(0)
            self.can_sock.send(0x3e9, bytes(msg), 0)

            # Sleep 100 ms
            time.sleep(0.1)

class LidarMessageSender(object):
    """Send LIDAR angle/distance pairs either from a COIN-D4 device or a synthetic generator."""

    START_CMD = bytes([0xAA, 0x55, 0xF0, 0x0F])
    END_CMD = bytes([0xAA, 0x55, 0xF5, 0x0A])
    HIGH_SPEED_CMD = bytes([0xAA, 0x55, 0xF2, 0x0D])
    LOW_SPEED_CMD = bytes([0xAA, 0x55, 0xF1, 0x0E])

    HEADER = b"\xAA\x55"
    TYPE_MASK = 0x01
    FREQ_MASK = 0xFE

    def __init__(self, can_sock, verbose=False, port='/dev/ttyACM0', baudrate=230400, speed='high'):
        self.can_sock = can_sock
        self.verbose = verbose
        self.thread = threading.Thread(target=self.run, daemon=True)

        # Serial configuration (None -> use synthetic data)
        print(port)
        self.port = port
        self.baudrate = baudrate
        self.speed = speed

        # Synthetic generation parameters
        self.batch_size = 45
        self._idx = 0
        self._tick = 0

    def start(self):
        self.thread.start()

    def _parse_packet(self, packet):
        """Parse a COIN-D4 packet and return a list of (distance[m], angle[deg])"""
        m_t = packet[2]
        lsn = packet[3]
        fsa_raw = packet[4] | (packet[5] << 8)
        lsa_raw = packet[6] | (packet[7] << 8)
        samples = packet[10:]
        start_angle = fsa_raw / 100.0
        end_angle = lsa_raw / 100.0
        points = []
        for i in range(lsn):
            idx = i * 3
            if idx + 3 > len(samples):
                break
            dist = (samples[idx+1] >> 2) | (samples[idx+2] << 8)  # mm
            if lsn > 1:
                angle = start_angle + (end_angle - start_angle) * i / (lsn - 1)
            else:
                angle = start_angle
            points.append((dist / 1000.0, angle))
        return points

    def run(self):
        if self.port:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

            ser.write(self.START_CMD)
            ser.flush()
            time.sleep(0.2)

            speed_cmd = self.HIGH_SPEED_CMD if self.speed == 'high' else self.LOW_SPEED_CMD
            ser.write(speed_cmd)
            ser.flush()

            buf = bytearray()
            while True:
                byte = ser.read(1)
                if not byte:
                    continue
                buf += byte
                while len(buf) >= 8:
                    idx = buf.find(self.HEADER)
                    if idx < 0:
                        buf.clear()
                        break
                    if len(buf) < idx + 10:
                        break
                    lsn = buf[idx+3]
                    packet_length = 10 + lsn * 3
                    if len(buf) < idx + packet_length:
                        break
                    packet = bytes(buf[idx:idx+packet_length])
                    del buf[:idx+packet_length]
                    for dist, angle in self._parse_packet(packet):
                        angle_id = int(angle) % 360
                        distance_raw = int(dist / 0.001)
                        data = struct.pack('<H', distance_raw)
                        if self.verbose:
                            print(f'lidar angle = {angle:.2f} distance = {dist:.3f}')
                        self.can_sock.send(0x680 + angle_id, data, 0)
        else:
            while True:
                for _ in range(self.batch_size):
                    angle = self._idx % 360
                    distance = 5 + 5 * math.sin(math.radians(angle + self._tick))
                    if self.verbose:
                        print(f'lidar angle = {angle} distance = {distance}')
                    distance_raw = int(distance / 0.001)
                    data = struct.pack('<H', distance_raw)
                    self.can_sock.send(0x680 + angle, data, 0)
                    self._idx += 1
                self._tick = (self._tick + 1) % 360
                time.sleep(0.2)

def main():
    parser = argparse.ArgumentParser(description='Simple CAN vehicle simulator.')
    parser.add_argument('interface', type=str, help='interface name (e.g. vcan0)')
    parser.add_argument('--lin-interface', help='Separate LIN interface name (e.g. sllin0)')
    parser.add_argument('--lidar-port', type=str, default='/dev/ttyUSB0', help='Serial port for COIN-D4 LIDAR')
    parser.add_argument('--lidar-baudrate', type=int, default=230400, help='LIDAR serial baudrate')
    parser.add_argument('--lidar-speed', choices=['high', 'low'], default='low', help='LIDAR scan speed')
    parser.add_argument('-v', '--verbose', help='increase output verbosity', action='store_true')
    args = parser.parse_args()

    lin_interface = args.lin_interface
    if lin_interface == None:
        lin_interface = args.interface

    try:
        can_sock = CANSocket(args.interface)
        diag_can_sock = CANSocket(args.interface)
        steeringwheel_can_sock = CANSocket(lin_interface)
    except OSError as e:
        sys.stderr.write('Could not listen on interface {0}\n'.format(args.interface))
        sys.exit(e.errno)

    print('Using {0}'.format(args.interface))
    sim = VehicleSimulator()
    status_sender = StatusMessageSender(can_sock, sim, args.verbose)
    lidar_sender = LidarMessageSender(
        can_sock,
        args.verbose,
        port=args.lidar_port,
        baudrate=args.lidar_baudrate,
        speed=args.lidar_speed,
    )
    diag_handler = DiagnosticMessageHandler(diag_can_sock, sim, args.verbose)
    steeringwheel_handler = SteeringWheelMessageHandler(steeringwheel_can_sock, sim, args.verbose)
    sim.start()
    status_sender.start()
    lidar_sender.start()
    diag_handler.start()
    steeringwheel_handler.start()
    try:
        while True:
            time.sleep(60)
    except (KeyboardInterrupt, SystemExit):
        #sim.stop()
        sys.exit(0)

if __name__ == '__main__':
    main()
