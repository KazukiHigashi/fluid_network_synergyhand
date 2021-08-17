#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import time
import struct
import numpy

FT_RAW_MAX = 10000.0
FXYZ_MAX   = 50.0
MXYZ_MAX   = 1.0
COEFF_GC   = 9.81

target_ip = "10.0.1.103"
target_port = 10001
buffer_size = 4096

tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_client.connect((target_ip, target_port))


def get_model_num():
    # display command of model name
    cmd = bytearray([0x10, 0x02, 0x04, 0xff, 0x2a, 0x00, 0x10, 0x03, 0xd2])
    tcp_client.send(cmd)
    
    response = tcp_client.recv(buffer_size)
    #print(response)
    model_num = bytearray(14)
    
    i = 0
    
    for d in response:
        
        if i >= 6 and i < 19:
            model_num[i - 6] = d
        i = i + 1

    model_num = model_num.decode()
    print(model_num)

    if 'SFS055YA500R6' in model_num:
        return 0
    else:
        return 1

# ----- 6軸値の取得 ---- #
def get_latest_data():

    # hand-shake mode command
    cmd = bytearray([0x10, 0x02, 0x04, 0xff, 0x30, 0x00, 0x10, 0x03, 0xc8])
    tcp_client.send(cmd)

    response = tcp_client.recv(buffer_size)


    # for byte in response:
    #     print(hex(byte), end=", ")
    # print(" $$ length: {}".format(len(response)))
    # print(response)
    if len(response) != 25:
        return None
    
    fx = bytes(response[6:7+1])
    fy = bytes(response[8:9 + 1])
    fz = bytes(response[10:11 + 1])
    mx = bytes(response[12:13 + 1])
    my = bytes(response[14:15 + 1])
    mz = bytes(response[16:17 + 1])

    status = response[20]

    fx = struct.unpack("<h", fx)
    fy = struct.unpack("<h", fy)
    fz = struct.unpack("<h", fz)
    mx = struct.unpack("<h", mx)
    my = struct.unpack("<h", my)
    mz = struct.unpack("<h", mz)

    ft_val = numpy.array([fx, fy, fz, mx, my, mz])
    ft_val = ft_val / FT_RAW_MAX * FXYZ_MAX / COEFF_GC * 1000.0

    return ft_val

def stream():

    cmd = bytearray([0x10, 0x02, 0x04, 0xff, 0x32, 0x00, 0x10, 0x03, 0xca])
    tcp_client.send(cmd)

    while(1):
        tcp_client.recv(buffer_size)
        response = tcp_client.recv(buffer_size)

        # print(response)
        # print("\n")
        # time.sleep(1.0)

        fx = bytes(response[6:7 + 1])
        fy = bytes(response[8:9 + 1])
        fz = bytes(response[10:11 + 1])
        mx = bytes(response[12:13 + 1])
        my = bytes(response[14:15 + 1])
        mz = bytes(response[16:17 + 1])

        status = response[20]

        fx = struct.unpack("<h", fx)
        fy = struct.unpack("<h", fy)
        fz = struct.unpack("<h", fz)
        mx = struct.unpack("<h", mx)
        my = struct.unpack("<h", my)
        mz = struct.unpack("<h", mz)

        ft_val = numpy.array([fx, fy, fz, mx, my, mz])
        ft_val = numpy.concatenate([ft_val[:3] / FT_RAW_MAX * FXYZ_MAX,
                                    ft_val[-3:] / FT_RAW_MAX * MXYZ_MAX])

        print(ft_val)


def stream_end():
    cmd = bytearray([0x10, 0x02, 0x04, 0xff, 0x33, 0x00, 0x10, 0x03, 0xdb])
    tcp_client.send(cmd)
