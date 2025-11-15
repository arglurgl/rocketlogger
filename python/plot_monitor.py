import argparse
import os
import time
import csv
import matplotlib.pyplot as plt

def read_csv_pairs(path):
    times = []
    heights = []
    try:
        with open(path, 'r') as fh:
            rdr = csv.reader(fh)
            for row in rdr:
                if not row: 
                    continue
                # join in case line was split oddly, then split by comma
                if len(row) == 1:
                    parts = row[0].split(',')
                else:
                    parts = row
                try:
                    t = float(parts[0].strip())
                    h = float(parts[1].strip())
                except Exception:
                    continue
                times.append(t)
                heights.append(h)
    except Exception:
        pass
    return times, heights

def plot_data(ax, times, heights, fname):
    ax.clear()
    ax.plot(times, heights, '-o', markersize=4, linewidth=1)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Height (m)')
    ax.set_title(f'Flight log: {os.path.basename(fname)}')
    ax.grid(True)
    if times:
        ax.set_xlim(min(times), max(times))
    plt.tight_layout()

def main():
    p = argparse.ArgumentParser(description="Monitor a CSV and plot time,height on change")
    p.add_argument("file", help="CSV file to monitor")
    p.add_argument("--interval", type=float, default=0.5, help="Poll interval in seconds (default 0.5s)")
    args = p.parse_args()

    fname = args.file
    if not os.path.exists(fname):
        print("Waiting for file to appear:", fname)

    plt.ion()
    fig, ax = plt.subplots(figsize=(8,4))

    last_mtime = None
    try:
        while True:
            if os.path.exists(fname):
                mtime = os.path.getmtime(fname)
                if last_mtime is None or mtime != last_mtime:
                    times, heights = read_csv_pairs(fname)
                    plot_data(ax, times, heights, fname)
                    plt.draw()
                    last_mtime = mtime
            else:
                # file missing, clear plot
                ax.clear()
                ax.set_title(f'Waiting for file: {os.path.basename(fname)}')
                plt.pause(0.01)
            # keep GUI responsive
            plt.pause(0.01)
            time.sleep(max(0.01, args.interval - 0.01))
    except KeyboardInterrupt:
        print("Exiting monitor.")
    finally:
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
