#!/usr/bin/env python3
# tests/fake_rtkrcv.py — stand-in for rtkrcv: serve llh lines on RTKRCV_SOL_PORT.
import os
import socket
import threading
import time

LINE = ("2026/08/27 04:15:55.400   44.501234567   90.287654321   617.1234"
        "   1  38   0.0110   0.0123   0.0322  -0.0001   0.0002   0.0003"
        "   0.80   25.0\r\n").encode()


def serve(conn):
    try:
        while True:
            conn.sendall(LINE)
            time.sleep(0.2)
    except OSError:
        pass


def main():
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", int(os.environ["RTKRCV_SOL_PORT"])))
    srv.listen()
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=serve, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
