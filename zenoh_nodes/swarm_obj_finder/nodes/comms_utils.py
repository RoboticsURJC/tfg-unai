from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from message_utils import *

import struct



def get_str_serializer(s_length: int, fill_char:str=' '): # Returns a function
    return lambda obj: ser_string(obj, s_length, fill_char)

def ser_string(s: str, s_length: int, fill_char:str=' ') -> bytes:
    ns_bytes = bytes(s, "utf-8")
    # Fixed ns length of s_length (fill with space characters):
    return bytes(fill_char, "utf-8") * (s_length - len(ns_bytes)) + ns_bytes

def deser_string(bytes: bytes, fill_chars=' ') -> str:
    return str(bytes.decode('utf-8')).lstrip(fill_chars)


def ser_ros2_msg(ros2_msg) -> bytes:
    check_for_type_support(type(ros2_msg))
    return _rclpy.rclpy_serialize(ros2_msg, type(ros2_msg))

def get_ros2_deserializer(ros2_type): # Returns a function
    check_for_type_support(ros2_type)
    return lambda obj: _rclpy.rclpy_deserialize(obj, ros2_type)


def get_uint_serializer(int_bytes_length: int): # Returns a function
    return lambda i: i.to_bytes(int_bytes_length, 'big')

def deser_uint(int_bytes: bytes) -> int:
    return int.from_bytes(int_bytes, 'big')


def ser_uint_list(l: list, int_bytes_length) -> bytes:
    b = bytes()
    serializer = get_uint_serializer(int_bytes_length)
    for i in l:
        b += serializer(int(i))
    return b

def deser_uint_list(bytes_list: list, int_bytes_length: int) -> list:
    l = list()
    for i in range(0, len(bytes_list), int_bytes_length):
        l.append(deser_uint(bytes_list[i:i + int_bytes_length]))
    return l


def get_float_packing_format(float_bytes_length: int) -> str:
    format = "d" # C double 8 bytes = 64 bits (python default)
    if float_bytes_length == 4: # C float 4 bytes = 32 bits
        format = "f"
    return format

def get_float_serializer(float_bytes_length: int): # Returns a function
    format = get_float_packing_format(float_bytes_length)
    return lambda f: struct.pack(format, f)

def deser_float(float_bytes: bytes, float_bytes_length: int) -> float:
    format = get_float_packing_format(float_bytes_length)
    return struct.unpack(format, float_bytes)[0]


def ser_float_list(l: list, float_bytes_length: int) -> bytes:
    b = bytes()
    serializer = get_float_serializer(float_bytes_length)
    for i in l:
        b += serializer(float(i))
    return b

def deser_float_list(bytes_list: list, float_bytes_length: int) -> list:
    l = list()
    for i in range(0, len(bytes_list), float_bytes_length):
        l.append(deser_float(bytes_list[i:i + float_bytes_length], float_bytes_length))
    return l


def get_ctrd_msg_serializer(str_bytes_length: int,
                            float_bytes_length: int): # Returns a function
    return lambda obj: ser_ctrd_msg(obj, str_bytes_length, float_bytes_length)

def ser_ctrd_msg(centroid_msg: CentroidMessage,
                 str_bytes_length: int,
                 float_bytes_length: int) -> bytes:
    ser_founder = ser_string(centroid_msg.get_founder(), str_bytes_length)
    ser_centroid = ser_float_list(centroid_msg.get_centroid(),
                                  float_bytes_length)
    return ser_founder + ser_centroid

def get_ctrd_msg_deserializer(str_bytes_length: int,
                              float_bytes_length: int): # Returns a function
    return lambda obj: deser_ctrd_msg(obj, str_bytes_length, float_bytes_length)

def deser_ctrd_msg(ser_centroid_msg: bytes,
                   str_bytes_length: int,
                   float_bytes_length: int) -> CentroidMessage:
    ser_founder = ser_centroid_msg[:str_bytes_length]
    founder = deser_string(ser_founder)
    ser_centroid = ser_centroid_msg[str_bytes_length:]
    x, y = deser_float_list(ser_centroid, float_bytes_length)
    return CentroidMessage(x, y, founder)


def get_world_pos_msg_serializer(str_bytes_length: int): # Returns a function
    return lambda obj: ser_world_pos_msg(obj, str_bytes_length)

def ser_world_pos_msg(world_pos_msg: WorldPosition,
                      str_bytes_length: int) -> bytes:
    ser_sender = ser_string(world_pos_msg.sender, str_bytes_length)
    ser_world_pos = ser_ros2_msg(world_pos_msg.world_position)
    return ser_sender + ser_world_pos

def get_world_pos_msg_deserializer(str_bytes_length: int): # Returns a function
    return lambda obj: deser_world_pos_msg(obj, str_bytes_length)

def deser_world_pos_msg(world_pos_msg: bytes,
                        str_bytes_length: int) -> WorldPosition:
    ser_sender = world_pos_msg[:str_bytes_length]
    sender = deser_string(ser_sender)
    ser_world_pos = world_pos_msg[str_bytes_length:]
    world_pos = get_ros2_deserializer(PoseStamped)(ser_world_pos)
    return WorldPosition(world_pos, sender)

# To get the asynchronous functions for the inputs:
def get_input_func(name, input):
    #This doesn't work using lambda function because it's not asynchronous
    async def func():
        return (name, await input.recv())
    return func
