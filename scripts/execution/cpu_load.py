#!/usr/bin/env python3
import multiprocessing as mp
import os
import signal
import sys
import time

def worker():
    x = 0
    while True:
        x = (x * 1664525 + 1013904223) & 0xFFFFFFFF

def main():
    workers = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    procs = []
    for _ in range(workers):
        p = mp.Process(target=worker)
        p.daemon = True
        p.start()
        procs.append(p)
    print(f"CPU stress started: {len(procs)} workers (PIDs: {[p.pid for p in procs]})", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        for p in procs:
            if p.is_alive():
                p.terminate()

if __name__ == "__main__":
    main()
