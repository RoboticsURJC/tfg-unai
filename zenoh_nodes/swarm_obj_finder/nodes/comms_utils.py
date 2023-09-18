from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

import message_utils



def ser_string(s: str, s_length: int, fill_char=' ') -> bytes:
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


def get_int_serializer(int_bystes_length: int): # Returns a function
    return lambda i: i.to_bytes(int_bystes_length, 'big')

def deser_int(int_bytes: bytes) -> int:
    return int.from_bytes(int_bytes, 'big')


def ser_int_list(l: list, int_bytes_length) -> bytes:
    b = bytes()
    serializer = get_int_serializer(int_bytes_length)
    for i in l:
        b += serializer(int(i))
    return b

def deser_int_list(bytes_list: list, int_bytes_length) -> list:
    l = list()
    for i in range(0, len(bytes_list), int_bytes_length):
        l.append(deser_int(bytes_list[i:i+int_bytes_length]))
    return l


def get_ctrd_msg_serializer(str_bytes_length: int,
                            int_bytes_length: int): # Returns a function
    return lambda obj: ser_ctrd_msg(obj, str_bytes_length, int_bytes_length)

def ser_ctrd_msg(centroid_msg: message_utils.CentroidMessage,
                 str_bytes_length: int,
                 int_bytes_length: int) -> bytes:
    ser_msg = ser_string(centroid_msg.get_founder(), str_bytes_length)
    ser_msg += ser_int_list(centroid_msg.get_centroid(), int_bytes_length)
    return ser_msg



# To get the asynchronous functions for the inputs:
def get_input_func( name, input):
    async def func():
        return (name, await input.recv())
    return func