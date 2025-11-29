#!/usr/bin/env python3
"""
Simple plotter for joystick button events logged by util/joystick.py
Usage:
  python scripts/plot_buttons.py [--file path/to/button_events.csv]

The script draws a raster-like plot: x=time, y=button index (0..N-1), points where state=1 are marked.
"""
import argparse
import os
import csv
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime


def load_events(path):
    events = []
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                ts = float(r['ts'])
                btn = int(r['button'])
                state = int(r['state'])
                events.append((ts, btn, state))
            except Exception:
                continue
    return events


def plot_events(events, window=None):
    if not events:
        print('No events to plot')
        return
    # determine number of buttons
    max_btn = max(e[1] for e in events)
    nbtn = max_btn + 1
    # convert to per-button lists of times when state==1
    btn_times = [[] for _ in range(nbtn)]
    for ts, btn, state in events:
        if state:
            btn_times[btn].append(datetime.fromtimestamp(ts))
    # plot
    fig, ax = plt.subplots(figsize=(10, max(4, nbtn*0.4)))
    for i in range(nbtn):
        ys = [i] * len(btn_times[i])
        ax.plot(btn_times[i], ys, '|', markersize=10)
    ax.set_yticks(range(nbtn))
    ax.set_yticklabels([str(i) for i in range(nbtn)])
    ax.set_xlabel('time')
    ax.set_ylabel('button index')
    ax.set_title('Joystick button press events')
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    fig.autofmt_xdate()
    plt.tight_layout()
    plt.show()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--file', '-f', default=os.path.join('src','logs','button_events.csv'))
    args = p.parse_args()
    events = load_events(args.file)
    plot_events(events)

if __name__ == '__main__':
    main()
