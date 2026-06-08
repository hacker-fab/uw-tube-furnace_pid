"""Non-blocking keyboard input via a background thread.

``readchar`` reads one keypress at a time (handling raw-terminal setup and
cross-platform quirks for us). We run it in a daemon thread that pushes keys
onto a queue, so the control loop can poll ``get()`` without ever blocking.
"""

import queue
import threading

import readchar


class KeyboardListener:
    def __init__(self):
        self._q: "queue.Queue[str]" = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()
        return self

    def _run(self):
        while not self._stop.is_set():
            try:
                key = readchar.readkey()
            except Exception:
                # Terminal closed / interrupted — stop quietly.
                break
            self._q.put(key)

    def get(self):
        """Return the next pending key, or None if nothing is queued."""
        try:
            return self._q.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        self._stop.set()
