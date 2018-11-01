import signal
import time


def end_program(a, b):
    print("ya ya")

signal.signal(signal.SIGINT, end_program)
print("Calling sleep")
time.sleep(10)
