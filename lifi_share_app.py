"""
Li-Fi File & Text Share
-----------------------
Run this app on BOTH computers. Each computer has an Arduino with
lifi_trx.ino uploaded:
    LED transmitter on D9, photodiode receiver (OPA320 + LM393) on D2.

1. Plug in the Arduino and pick its port, choose the SAME light speed on
   both computers, then press "Connect".
2. Send files or text in either direction. Everything travels as light.

Reliability: data is split into small packets with a CRC; the other side
acknowledges each packet and the sender retries if it is lost or damaged.

Requires Python 3.9+ and pyserial (pip install pyserial).
"""

import os
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk

try:
    import serial
    import serial.tools.list_ports as list_ports
except Exception:  # pyserial missing
    serial = None
    list_ports = None

DEFAULT_SAVE = os.path.join(os.path.expanduser("~"), "Shared_Received")

# Light speed presets (bits per second on the LED; each byte = 10 bits)
SPEEDS = {
    "Very slow (10 bit/s)": 10,
    "Slow (50 bit/s)": 50,
    "Medium (100 bit/s)": 100,
    "Fast (250 bit/s)": 250,
    "Faster (500 bit/s)": 500,
    "Max (1000 bit/s)": 1000,
}
DEFAULT_SPEED = "Medium (100 bit/s)"

# Packet layer ---------------------------------------------------------------
# A5 5A | type | seq | len | payload (<= 48) | crc16 (type..payload)
MAGIC = b"\xA5\x5A"
T_START, T_DATA, T_ACK = 1, 2, 3
MAX_PAYLOAD = 48
DATA_CHUNK = 40
MAX_TRIES = 8
OVERHEAD = 7


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_packet(ptype, seq, payload=b""):
    body = bytes([ptype, seq & 0xFF, len(payload)]) + payload
    return MAGIC + body + struct.pack(">H", crc16(body))


def list_serial_ports():
    if list_ports is None:
        return []
    return [f"{p.device} - {p.description}" for p in list_ports.comports()]


def unique_path(folder, name):
    name = os.path.basename(name) or "received_file"
    path = os.path.join(folder, name)
    base, ext = os.path.splitext(name)
    i = 1
    while os.path.exists(path):
        path = os.path.join(folder, f"{base} ({i}){ext}")
        i += 1
    return path


def human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024


def fmt_time(sec):
    sec = int(max(sec, 0))
    h, rem = divmod(sec, 3600)
    m, s = divmod(rem, 60)
    return f"{h}h {m:02d}m {s:02d}s"


# --------------------------------------------------------------------------
# Arduino light link
# --------------------------------------------------------------------------
class LightLink:
    """Serial protocol with the Arduino (115200 baud):
         PC -> Arduino:  'P'            ping            -> 'K'
                         'B' hi lo      set bit rate    -> 'K'
                         'D' len data   queue for LED
         Arduino -> PC:  'R' byte       byte seen by the photodiode
    """

    def __init__(self, on_packet, on_lost):
        self.on_packet = on_packet
        self.on_lost = on_lost
        self.ser = None
        self.running = False
        self.bps = 100
        self.wlock = threading.Lock()
        self.buf = bytearray()
        self.expect_data = False

    @property
    def connected(self):
        return self.ser is not None

    def open(self, port, bps):
        if serial is None:
            raise RuntimeError("pyserial is not installed (pip install pyserial)")
        s = serial.Serial(port, 115200, timeout=0.2)
        time.sleep(2.0)  # Arduino resets when the port opens
        s.reset_input_buffer()
        s.write(b"P")
        deadline = time.time() + 3
        ok = False
        while time.time() < deadline:
            if s.read(1) == b"K":
                ok = True
                break
        if not ok:
            s.close()
            raise RuntimeError("No reply from the Arduino. Is lifi_trx.ino uploaded?")
        self.ser = s
        self.set_rate(bps)
        self.buf.clear()
        self.expect_data = False
        self.running = True
        threading.Thread(target=self._read_loop, daemon=True).start()

    def close(self):
        self.running = False
        s, self.ser = self.ser, None
        if s:
            try:
                s.close()
            except Exception:
                pass

    def set_rate(self, bps):
        self.bps = bps
        if self.ser:
            with self.wlock:
                self.ser.write(b"B" + struct.pack(">H", bps))

    def write_packet(self, pkt):
        with self.wlock:
            self.ser.write(b"D" + bytes([len(pkt)]) + pkt)

    # ---- receiving
    def _read_loop(self):
        last = time.time()
        while self.running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                if self.running:
                    self.running = False
                    self.on_lost()
                return
            now = time.time()
            if data:
                last = now
                for b in data:
                    self._byte(b)
            elif self.buf and now - last > 60 * 10 / self.bps + 3:
                self.buf.clear()  # stale partial packet

    def _byte(self, b):
        if self.expect_data:
            self.expect_data = False
            self.buf.append(b)
            self._scan()
        elif b == ord("R"):
            self.expect_data = True

    def _scan(self):
        buf = self.buf
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                if buf and buf[-1] == MAGIC[0]:
                    del buf[:-1]
                else:
                    buf.clear()
                return
            del buf[:i]
            if len(buf) < 5:
                return
            n = buf[4]
            if n > MAX_PAYLOAD:
                del buf[:1]
                continue
            total = 5 + n + 2
            if len(buf) < total:
                return
            body = bytes(buf[2:5 + n])
            crc = (buf[5 + n] << 8) | buf[6 + n]
            if crc16(body) == crc:
                del buf[:total]
                try:
                    self.on_packet(body[0], body[1], body[3:])
                except Exception:
                    pass
            else:
                del buf[:1]


# --------------------------------------------------------------------------
# application
# --------------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Li-Fi File & Text Share")
        self.geometry("890x780")
        self.minsize(760, 720)

        self.link = LightLink(self._on_packet, lambda: self.ui(self._link_lost))
        self.stop_flag = threading.Event()
        self.send_lock = threading.Lock()
        self.sending = False
        self.ack_seq = -1
        self.ack_evt = threading.Event()
        self.rx = None
        self.rx_expected = 0
        self.ui_q = queue.Queue()
        self.file_path = None
        self.save_dir = DEFAULT_SAVE
        os.makedirs(self.save_dir, exist_ok=True)

        self._build_ui()
        self.after(50, self._pump)
        self.refresh_ports()
        if serial is None:
            self.log_msg("pyserial not found: run  pip install pyserial  first.")

    # ---------------------------------------------------------------- UI
    def _build_ui(self):
        style = ttk.Style(self)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        bg = style.lookup("TFrame", "background") or "#f0f0f0"
        self.configure(bg=bg)

        root = ttk.Frame(self, padding=(10, 6))
        root.pack(fill="both", expand=True)

        # ---- Connection
        conn = ttk.LabelFrame(root, text="Light link (Arduino)", padding=(10, 8))
        conn.pack(fill="x")
        row = ttk.Frame(conn)
        row.pack(fill="x")
        ttk.Label(row, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(row, textvariable=self.port_var, width=34)
        self.port_combo.pack(side="left", padx=(6, 6))
        ttk.Button(row, text="Refresh", command=self.refresh_ports).pack(
            side="left", padx=(0, 6)
        )
        self.btn_connect = ttk.Button(row, text="Connect", command=self.toggle_connect)
        self.btn_connect.pack(side="left")

        row2 = ttk.Frame(conn)
        row2.pack(fill="x", pady=(10, 0))
        ttk.Label(row2, text="Light speed").pack(side="left")
        self.speed_var = tk.StringVar(value=DEFAULT_SPEED)
        sp = ttk.Combobox(
            row2,
            textvariable=self.speed_var,
            values=list(SPEEDS),
            state="readonly",
            width=24,
        )
        sp.pack(side="left", padx=(8, 0))
        sp.bind("<<ComboboxSelected>>", lambda e: self._speed_changed())
        ttk.Label(row2, text="(must be the same on both computers)").pack(
            side="left", padx=(8, 0)
        )

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(conn, textvariable=self.status_var).pack(anchor="w", pady=(8, 0))

        # ---- Transmitter
        tx = ttk.LabelFrame(root, text="Transmitter - This computer", padding=(10, 8))
        tx.pack(fill="x", pady=(10, 0))
        r1 = ttk.Frame(tx)
        r1.pack(fill="x")
        self.file_var = tk.StringVar(value="No file selected")
        ttk.Label(r1, textvariable=self.file_var).pack(side="left")
        self.btn_send_file = ttk.Button(
            r1, text="Send file", command=self.send_file, state="disabled"
        )
        self.btn_send_file.pack(side="right")
        ttk.Button(r1, text="Choose file", command=self.choose_file).pack(
            side="right", padx=(0, 8)
        )

        r2 = ttk.Frame(tx)
        r2.pack(fill="x", pady=(10, 0))
        ttk.Label(r2, text="Text").pack(side="left")
        self.text_var = tk.StringVar()
        ent = ttk.Entry(r2, textvariable=self.text_var)
        ent.pack(side="left", fill="x", expand=True, padx=(8, 8))
        ent.bind("<Return>", lambda e: self.send_text())
        self.btn_send_text = ttk.Button(
            r2, text="Send text", command=self.send_text, state="disabled"
        )
        self.btn_send_text.pack(side="right")

        self.tx_bar = ttk.Progressbar(root, maximum=100)
        self.tx_bar.pack(fill="x", pady=(10, 0), ipady=5)
        self.tx_var = tk.StringVar(value="TX: idle")
        ttk.Label(root, textvariable=self.tx_var).pack(anchor="w", pady=(4, 0))

        # ---- Receiver
        rx = ttk.LabelFrame(root, text="Receiver - Other computer", padding=(10, 8))
        rx.pack(fill="x", pady=(6, 0))
        self.rx_bar = ttk.Progressbar(rx, maximum=100)
        self.rx_bar.pack(fill="x", ipady=5)
        self.rx_var = tk.StringVar(value="RX: idle")
        ttk.Label(rx, textvariable=self.rx_var).pack(anchor="w", pady=(6, 0))
        self.save_var = tk.StringVar(value=f"Save folder: {self.save_dir}")
        ttk.Label(rx, textvariable=self.save_var).pack(anchor="w", pady=(2, 0))
        ttk.Button(
            rx, text="Choose receive folder", command=self.choose_folder
        ).pack(anchor="e")

        note = tk.Label(
            root,
            text="Point each LED at the other computer's photodiode. "
            "Both sides can send and receive at the same time.",
            fg="#a0522d",
            bg=bg,
            anchor="w",
            justify="left",
        )
        note.pack(fill="x", pady=(10, 2))

        # ---- Activity
        act = ttk.LabelFrame(root, text="Activity", padding=(2, 2))
        act.pack(fill="both", expand=True)
        self.log = tk.Text(act, height=8, bg="white", relief="solid", bd=1, wrap="word")
        sb = ttk.Scrollbar(act, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set, state="disabled")
        sb.pack(side="right", fill="y")
        self.log.pack(fill="both", expand=True)

    # ------------------------------------------------------- UI plumbing
    def ui(self, fn, *a):
        """Schedule fn(*a) on the UI thread (safe from any thread)."""
        self.ui_q.put((fn, a))

    def _pump(self):
        try:
            while True:
                fn, a = self.ui_q.get_nowait()
                fn(*a)
        except queue.Empty:
            pass
        self.after(50, self._pump)

    def log_msg(self, msg):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def update_buttons(self):
        c = self.link.connected
        self.btn_connect.configure(text="Disconnect" if c else "Connect")
        can = "normal" if c and not self.sending else "disabled"
        self.btn_send_text.configure(state=can)
        self.btn_send_file.configure(state=can if self.file_path else "disabled")

    def refresh_ports(self):
        ports = list_serial_ports()
        self.port_combo.configure(values=ports)
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    # ------------------------------------------------------ connection
    def _bps(self):
        return SPEEDS.get(self.speed_var.get(), 100)

    def _speed_changed(self):
        if self.link.connected:
            self.link.set_rate(self._bps())
            self.log_msg(f"Light speed set to {self._bps()} bit/s.")

    def toggle_connect(self):
        if self.link.connected:
            self.disconnect()
            return
        port = self.port_var.get().split(" - ")[0].strip()
        if not port:
            self.log_msg("Pick the Arduino port first.")
            return
        self.btn_connect.configure(state="disabled")
        self.status_var.set(f"Connecting to {port} ...")
        bps = self._bps()

        def work():
            try:
                self.link.open(port, bps)
                self.ui(self._connected, port)
            except Exception as e:
                self.ui(self._connect_failed, str(e))

        threading.Thread(target=work, daemon=True).start()

    def _connected(self, port):
        self.stop_flag.clear()
        self.btn_connect.configure(state="normal")
        self.status_var.set(f"Connected on {port} at {self._bps()} bit/s")
        self.log_msg(f"Light link ready on {port}.")
        self.update_buttons()

    def _connect_failed(self, err):
        self.btn_connect.configure(state="normal")
        self.status_var.set("Disconnected")
        self.log_msg(f"Connection failed: {err}")
        self.update_buttons()

    def disconnect(self):
        self.stop_flag.set()
        self.link.close()
        self._rx_close()
        self.status_var.set("Disconnected")
        self.log_msg("Disconnected.")
        self.update_buttons()

    def _link_lost(self):
        if self.link.connected:
            self.log_msg("Arduino connection lost.")
            self.disconnect()

    # --------------------------------------------------------- sending
    def choose_file(self):
        p = filedialog.askopenfilename(title="Choose file to send")
        if p:
            self.file_path = p
            self.file_var.set(f"{os.path.basename(p)}  ({human(os.path.getsize(p))})")
            self.update_buttons()

    def _reliable(self, ptype, seq, payload):
        """Send one packet and wait for its acknowledgement (with retries)."""
        pkt = build_packet(ptype, seq, payload)
        bps = self.link.bps
        # my packet + partner's queue (worst case one packet) + its ack
        wait = (len(pkt) + (MAX_PAYLOAD + OVERHEAD) + OVERHEAD) * 10 / bps * 1.3 + 2
        for attempt in range(MAX_TRIES):
            if self.stop_flag.is_set():
                raise ConnectionError("Stopped")
            self.ack_evt.clear()
            self.ack_seq = seq
            self.link.write_packet(pkt)
            end = time.time() + wait
            while time.time() < end:
                if self.stop_flag.is_set():
                    raise ConnectionError("Stopped")
                if self.ack_evt.wait(0.1):
                    return
            self.ui(self.log_msg, f"No acknowledgement, retrying ({attempt + 1}/{MAX_TRIES})")
        raise TimeoutError("Partner did not answer")

    def _transfer(self, kind, name, total, chunks, label):
        name_b = name.encode("utf-8")[:MAX_PAYLOAD - 5]
        self._reliable(T_START, 0, kind.encode() + struct.pack(">I", total) + name_b)
        seq, sent, t0 = 1, 0, time.time()
        for chunk in chunks:
            self._reliable(T_DATA, seq & 0xFF, chunk)
            seq += 1
            sent += len(chunk)
            pct = sent * 100 / max(total, 1)
            el = time.time() - t0
            eta = f"  - about {fmt_time(el / sent * (total - sent))} left" if sent else ""
            self.ui(self.tx_bar.configure, {"value": pct})
            self.ui(
                self.tx_var.set,
                f"TX: {label}  {human(sent)} / {human(total)}  ({pct:.0f}%){eta}",
            )
        self.ui(self.tx_bar.configure, {"value": 100})

    def _run_send(self, job):
        if self.sending or not self.link.connected:
            return
        self.sending = True
        self.update_buttons()

        def work():
            with self.send_lock:
                try:
                    job()
                except Exception as e:
                    self.ui(self.tx_var.set, "TX: failed")
                    self.ui(self.log_msg, f"Send failed: {e}")
                self.sending = False
                self.ui(self.update_buttons)

        threading.Thread(target=work, daemon=True).start()

    def send_text(self):
        txt = self.text_var.get()
        if not txt:
            return
        data = txt.encode("utf-8")

        def job():
            self.ui(self.tx_var.set, f"TX: sending text ({len(data)} bytes)")
            chunks = (data[i:i + DATA_CHUNK] for i in range(0, len(data), DATA_CHUNK))
            self._transfer("T", "", len(data), chunks, "text")
            self.ui(self.tx_var.set, "TX: idle")
            self.ui(self.log_msg, f"Sent text: {txt}")
            self.ui(self.text_var.set, "")

        self._run_send(job)

    def send_file(self):
        if not self.file_path:
            return
        path = self.file_path
        name, size = os.path.basename(path), os.path.getsize(path)

        def job():
            self.ui(self.log_msg, f"Sending file: {name}")

            def chunks():
                with open(path, "rb") as f:
                    while True:
                        b = f.read(DATA_CHUNK)
                        if not b:
                            break
                        yield b

            self._transfer("F", name, size, chunks(), name)
            self.ui(self.tx_var.set, f"TX: done - {name}")
            self.ui(self.log_msg, f"Sent file: {name}")

        self._run_send(job)

    # ------------------------------------------------------- receiving
    def choose_folder(self):
        d = filedialog.askdirectory(title="Choose receive folder", initialdir=self.save_dir)
        if d:
            self.save_dir = d
            os.makedirs(d, exist_ok=True)
            self.save_var.set(f"Save folder: {d}")

    def _ack(self, seq):
        try:
            self.link.write_packet(build_packet(T_ACK, seq))
        except Exception:
            pass

    def _rx_close(self):
        rx = self.rx
        if rx and rx.get("f"):
            try:
                rx["f"].close()
            except Exception:
                pass
            rx["f"] = None
        self.rx = None

    def _on_packet(self, ptype, seq, payload):
        """Called from the reader thread for every valid packet."""
        if ptype == T_ACK:
            if seq == self.ack_seq:
                self.ack_evt.set()
            return

        if ptype == T_START:
            if len(payload) < 5:
                return
            self._rx_close()
            kind = chr(payload[0])
            size = struct.unpack(">I", payload[1:5])[0]
            name = payload[5:].decode("utf-8", "replace")
            rx = {"kind": kind, "size": size, "got": 0, "buf": bytearray(),
                  "f": None, "path": None, "done": False}
            if kind == "F":
                rx["path"] = unique_path(self.save_dir, name)
                rx["f"] = open(rx["path"], "wb")
                rx["name"] = os.path.basename(rx["path"])
                self.ui(self.log_msg, f"Receiving file: {name} ({human(size)})")
            else:
                rx["name"] = "text"
                self.ui(self.rx_var.set, f"RX: receiving text ({size} bytes)")
            self.rx = rx
            self.rx_expected = (seq + 1) & 0xFF
            self.ui(self.rx_bar.configure, {"value": 0})
            self._ack(seq)
            if size == 0:
                self._rx_finish()
            return

        if ptype == T_DATA:
            rx = self.rx
            if rx is None:
                return
            if seq == (self.rx_expected - 1) & 0xFF:
                self._ack(seq)  # duplicate: our ack was lost
                return
            if seq != self.rx_expected or rx["done"]:
                return
            chunk = payload[: rx["size"] - rx["got"]]
            if rx["kind"] == "F":
                rx["f"].write(chunk)
            else:
                rx["buf"].extend(chunk)
            rx["got"] += len(chunk)
            self.rx_expected = (seq + 1) & 0xFF
            self._ack(seq)
            pct = rx["got"] * 100 / max(rx["size"], 1)
            self.ui(self.rx_bar.configure, {"value": pct})
            self.ui(
                self.rx_var.set,
                f"RX: {rx['name']}  {human(rx['got'])} / {human(rx['size'])}  ({pct:.0f}%)",
            )
            if rx["got"] >= rx["size"]:
                self._rx_finish()

    def _rx_finish(self):
        rx = self.rx
        rx["done"] = True
        if rx["kind"] == "F":
            rx["f"].close()
            rx["f"] = None
            self.ui(self.rx_var.set, f"RX: done - {rx['name']}")
            self.ui(self.log_msg, f"Saved: {rx['path']}")
        else:
            text = bytes(rx["buf"]).decode("utf-8", "replace")
            self.ui(self.rx_var.set, "RX: idle")
            self.ui(self.log_msg, f"Received text: {text}")


def main():
    app = App()

    def close():
        app.stop_flag.set()
        app.link.close()
        app.destroy()

    app.protocol("WM_DELETE_WINDOW", close)
    app.mainloop()


if __name__ == "__main__":
    main()
