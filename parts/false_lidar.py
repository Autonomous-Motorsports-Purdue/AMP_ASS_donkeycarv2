import dpkt
import socket
import threading

TEST_DATA_FILE = 'test_pcap.pcap'
HOST = '127.0.0.1'
PORT = 2368

class False_Lidar():

    def __init__(self):
        self.f = open(TEST_DATA_FILE, 'rb')
        try:
            reader = dpkt.pcapng.Reader(self.f)
        except Exception:
            self.f.seek(0)
            reader = dpkt.pcap.Reader(self.f)

        # Pre-extract all UDP payloads from the pcap
        self.packets = []
        for ts, buf in reader:
            try:
                eth = dpkt.ethernet.Ethernet(buf)
                if not isinstance(eth.data, dpkt.ip.IP):
                    continue
                ip = eth.data
                if not isinstance(ip.data, dpkt.udp.UDP):
                    continue
                udp = ip.data
                self.packets.append(bytearray(udp.data))
            except Exception:
                continue

        print(f"False_Lidar: loaded {len(self.packets)} UDP packets from {TEST_DATA_FILE}")

        self.index = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = False
        self.thread = None

    def _send_loop(self):
        while self.running:
            if len(self.packets) == 0:
                return

            payload = self.packets[self.index]
            self.index = (self.index + 1) % len(self.packets)
            self.sock.sendto(payload, (HOST, PORT))

    def run(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._send_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join()
            self.thread = None

    def __del__(self):
        self.stop()
        self.sock.close()
        self.f.close()
