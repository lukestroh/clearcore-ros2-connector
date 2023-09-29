#!/usr/bin/env python3
import socket
from asyncio import IncompleteReadError # only import the exception class

class SocketStreamReader():
    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.recv_buffer = bytearray()

    def _recv_into(self, view: memoryview) -> int:
        bytes_read = min(len(view), len(self.recv_buffer))
        view[:bytes_read] = self.recv_buffer[:bytes_read]
        self.recv_buffer = self.recv_buffer[bytes_read:]
        if bytes_read == len(view):
            return bytes_read
        bytes_read += self.sock.recv_into(view[bytes_read:])
        return bytes_read

    def read(self, num_bytes: int = -1) -> bytes:
        if num_bytes < 1:
            raise ValueError("'num_bytes' must be greater than 1.")
        return self._read_bytes(num_bytes)
    
    def _read_bytes(self, num_bytes: int) -> bytes:
        buf = bytearray(num_bytes)
        pos = 0
        while pos < num_bytes:
            n = self._recv_into(memoryview(buf)[:pos])
            if n == 0:
                raise IncompleteReadError(bytes(buf[:pos]), num_bytes)
            pos += n
        return bytes(buf)
    
    def read_until(self, separator: bytes = b'\n', chunk_size: int = 4096) -> bytes:
        if len(separator) != 1:
            raise ValueError("Only separators of length 1 are supported")
        
        chunk = bytearray(chunk_size)
        start = 0
        buf = bytearray(len(self.recv_buffer))
        bytes_read = self._recv_into(memoryview(buf))
        assert bytes_read == len(buf)

        while True:
            idx = buf.find(separator, start)
            if idx != -1:
                break

            start = len(self.recv_buffer)
            bytes_read = self._recv_into(memoryview(chunk))
            buf += memoryview(chunk)[:bytes_read]

        result = bytes(buf[: idx + 1])
        self.recv_buffer = b''.join((memoryview(buf)[idx + 1 :], self.recv_buffer))

        return result
    
    def readline(self) -> bytes:
        return self.read_until(b'\n')