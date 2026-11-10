#!/usr/bin/env python3
# ==============================================================================
# Robot Command Python API Client
# Memungkinkan Dashboard GUI, Web Backend, atau Voice Pipeline mengirim perintah
# dan menerima feedback streaming secara asinkron dari robot.
# ==============================================================================

import socket
import json
import threading
import time
from typing import Dict, Any, Optional, Callable, List

class RobotCommandClient:
    """
    Client API untuk berkomunikasi dengan robot_command node via TCP Socket.
    Kompatibel dengan kontrol dari Dashboard GUI, Web Apps, dan NLU Voice Pipeline.
    """
    
    def __init__(self, host: str = "127.0.0.1", port: int = 5000, timeout: float = 3.0):
        self.host = host
        self.port = int(port)
        self.timeout = timeout
        
        # Feedback listener
        self._listener_socket: Optional[socket.socket] = None
        self._listener_thread: Optional[threading.Thread] = None
        self._listener_running = False
        self._feedback_callbacks: List[Callable[[Dict[str, Any]], None]] = []

    def ping(self, timeout: float = 1.5) -> bool:
        """Memeriksa apakah server robot_command aktif dan dapat dihubungi."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(timeout)
                s.connect((self.host, self.port))
            return True
        except Exception:
            return False

    def send_command(self, cmd_dict: Dict[str, Any], timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Kirim structured command JSON ke robot dan kembalikan response dictionary.
        """
        t = timeout or self.timeout
        payload = (json.dumps(cmd_dict, ensure_ascii=False) + "\n").encode('utf-8')
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(t)
                s.connect((self.host, self.port))
                s.sendall(payload)
                
                # Terima response
                data = s.recv(4096)
                if data:
                    line = data.decode('utf-8').strip().split('\n')[0]
                    return json.loads(line)
                return {"success": False, "error": "Empty response from server"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    # ==========================================================================
    # API FUNCTIONS UNTUK DASHBOARD & VOICE COMMAND
    # ==========================================================================
    def navigate(self, location: str) -> Dict[str, Any]:
        """
        Memerintahkan robot bergerak ke nama ruangan / waypoint tertentu.
        Contoh: client.navigate("ruang_203") atau client.navigate("Lobi")
        """
        cmd = {
            "intent": "NAVIGATE",
            "slots": {"location": str(location)},
            "status": "VALID",
            "source": "api"
        }
        return self.send_command(cmd)

    def navigate_coordinates(self, x: float, y: float, yaw: float = 0.0) -> Dict[str, Any]:
        """
        Memerintahkan robot bergerak ke koordinat peta spesifik (x, y, yaw).
        """
        cmd = {
            "intent": "NAVIGATE",
            "slots": {"x": float(x), "y": float(y), "yaw": float(yaw)},
            "status": "VALID",
            "source": "api"
        }
        return self.send_command(cmd)

    def stop(self, emergency: bool = False) -> Dict[str, Any]:
        """
        Menghentikan pergerakan robot seketika.
        """
        cmd = {
            "intent": "EMERGENCY" if emergency else "STOP",
            "status": "VALID",
            "source": "api"
        }
        return self.send_command(cmd)

    def cancel_goal(self) -> Dict[str, Any]:
        """
        Membatalkan target navigasi yang sedang aktif.
        """
        cmd = {
            "intent": "CANCEL",
            "status": "VALID",
            "source": "api"
        }
        return self.send_command(cmd)

    def return_home(self) -> Dict[str, Any]:
        """
        Memerintahkan robot kembali ke posisi awal / stasiun Home.
        """
        cmd = {
            "intent": "GO_BACK",
            "slots": {"location": "home"},
            "status": "VALID",
            "source": "api"
        }
        return self.send_command(cmd)

    def get_status(self) -> Dict[str, Any]:
        """
        Mengambil status telemetri robot (state, koordinat, rintangan, sisa jarak).
        """
        cmd = {"intent": "GET_STATUS", "source": "api"}
        return self.send_command(cmd)

    def get_waypoints(self) -> Dict[str, Any]:
        """
        Mengambil daftar seluruh ruangan / titik waypoint yang terdaftar.
        """
        cmd = {"intent": "GET_WAYPOINTS", "source": "api"}
        return self.send_command(cmd)

    # ==========================================================================
    # STREAMING FEEDBACK LISTENER (Sudah sampai, Tabrakan, Replanning)
    # ==========================================================================
    def register_feedback_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """Menambahkan fungsi callback untuk menerima event feedback real-time."""
        if callback not in self._feedback_callbacks:
            self._feedback_callbacks.append(callback)

    def start_feedback_listener(self, on_feedback: Optional[Callable[[Dict[str, Any]], None]] = None):
        """
        Membuka koneksi socket persisten di background untuk mendengarkan
        semua feedback event (GOAL_REACHED, OBSTACLE_WARNING, REPLANNING_PATH).
        """
        if on_feedback:
            self.register_feedback_callback(on_feedback)
            
        if self._listener_running:
            return

        self._listener_running = True
        self._listener_thread = threading.Thread(target=self._feedback_loop, daemon=True)
        self._listener_thread.start()

    def _feedback_loop(self):
        while self._listener_running:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5.0)
                s.connect((self.host, self.port))
                self._listener_socket = s
                
                # Kirim perintah inisialisasi listener
                s.sendall(b'{"command": "SUBSCRIBE_FEEDBACK"}\n')
                
                buffer = ""
                while self._listener_running:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    buffer += chunk.decode('utf-8')
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            try:
                                event = json.loads(line)
                                for cb in self._feedback_callbacks:
                                    try:
                                        cb(event)
                                    except Exception as cb_err:
                                        pass
                            except json.JSONDecodeError:
                                pass
            except Exception:
                time.sleep(2.0)
            finally:
                if self._listener_socket:
                    try:
                        self._listener_socket.close()
                    except Exception:
                        pass
                    self._listener_socket = None

    def stop_feedback_listener(self):
        """Menutup listener feedback background."""
        self._listener_running = False
        if self._listener_socket:
            try:
                self._listener_socket.close()
            except Exception:
                pass

# ==============================================================================
# CONVENIENCE STANDALONE FUNCTIONS
# ==============================================================================
def send_voice_command(location: str, host: str = "127.0.0.1", port: int = 5000) -> Dict[str, Any]:
    """Helper fungsi satu baris untuk perintah navigasi suara."""
    client = RobotCommandClient(host=host, port=port)
    return client.navigate(location)

def emergency_stop(host: str = "127.0.0.1", port: int = 5000) -> Dict[str, Any]:
    """Helper fungsi satu baris untuk emergency stop."""
    client = RobotCommandClient(host=host, port=port)
    return client.stop(emergency=True)
