import time
import serial
import collections
import matplotlib.pyplot as plt

# ========= A MODIFIER =========
PORT = "/dev/ttyACM0"      # ton port Arduino
BAUD = 9600        # doit matcher Serial.begin
# ==============================

ZOOM_WINDOW_S = 20.0
MAX_HISTORY_S = 2500.0

t0 = time.time()
times = collections.deque()
tcc = collections.deque()
tc = collections.deque()
t1 = collections.deque()
pid=  collections.deque()

ser = serial.Serial(PORT, BAUD, timeout=1)

plt.ion()
fig = plt.figure(figsize=(10, 10))
plt.show()

ax_temp_full = fig.add_subplot(2, 2, 1)
ax_temp_zoom = fig.add_subplot(2, 2, 2)
ax_pid_full = fig.add_subplot(2, 2, 3)
ax_pid_zoom = fig.add_subplot(2, 2, 4)

line_tcc_full, = ax_temp_full.plot([], [], label="Tcc", c='black', linewidth=1)
line_tc_full, = ax_temp_full.plot([], [], label="Tc", c='green', linewidth=2)
line_t1_full, = ax_temp_full.plot([], [], label="NTC1", c='orange', linewidth=2)
ax_temp_full.set_title("Global (historique)")
ax_temp_full.set_xlabel("Temps (s)")
ax_temp_full.set_ylabel("Température (°C)")
ax_temp_full.legend()
ax_temp_full.grid(True)

line_tcc_zoom, = ax_temp_zoom.plot([], [], label="Tcc", c='black', linewidth=1)
line_tc_zoom, = ax_temp_zoom.plot([], [], label="Tc", c='green',linewidth=2)
line_t1_zoom, = ax_temp_zoom.plot([], [], label="NTC1", c='orange',linewidth=2)
ax_temp_zoom.set_title("Zoom (20 dernières secondes)")
ax_temp_zoom.set_xlabel("Temps (s)")
ax_temp_zoom.set_ylabel("Température (°C)")
ax_temp_zoom.legend()
ax_temp_zoom.grid(True)

line_pid_full, = ax_pid_full.plot([], [], label="PID", c='blue',linewidth=2)
ax_pid_full.set_title("Global (historique pid)")
ax_pid_full.set_xlabel("Temps (s)")
ax_pid_full.set_ylabel("Power")
ax_pid_full.legend()
ax_pid_full.grid(True)

line_pid_zoom, = ax_pid_zoom.plot([], [], label="PID", c='blue',linewidth=2)
ax_pid_zoom.set_title("Zoom (20 dernières secondes)")
ax_pid_zoom.set_xlabel("Temps (s)")
ax_pid_zoom.set_ylabel("Power")
ax_pid_zoom.legend()
ax_pid_zoom.grid(True)


running = True

def on_key(event):
    global running
    if event.key == 'q':
        running = False

fig.canvas.mpl_connect('key_press_event', on_key)

def prune():
    while times and (times[-1] - times[0]) > MAX_HISTORY_S:
        times.popleft(); tc.popleft(); t1.popleft(); pid.popleft();

def parse_line(raw: str):
    raw = raw.strip()
    print(raw)
    if not raw:
        return None

    # Format 1 : "Tc:33.20 T1:32.90" (espaces) ou tab
    if "Tcc:" in raw and "Tc:" in raw and "T1:" in raw and "sortiePID:" in raw:
        raw = raw.replace("\t", " ")
        parts = raw.split()
        #print("parts:", parts, "len", len(parts))
        if len(parts) < 4:
            return None
        try:    
            Tcc = float(parts[0].split(":", 1)[1])
            Tc = float(parts[1].split(":", 1)[1])
            T1 = float(parts[2].split(":", 1)[1])
            PID = float(parts[3].split(":", 1)[1])
            return Tcc,Tc, T1, PID
        except Exception:
            return None    
        
    # Sinon : ligne texte (ex: "=== PID Ready ===") -> on ignore
    return None

try:
    while running:
        raw = ser.readline().decode(errors="ignore")
        parsed = parse_line(raw)        
        if parsed is None:
            continue

        Tcc, Tc, T1 , sortiePID = parsed

        t = time.time() - t0
        times.append(t); tcc.append(Tcc); tc.append(Tc); t1.append(T1); pid.append(sortiePID)
        prune()

        x = list(times); y_tcc = list(tcc); y_tc = list(tc); y_t1 = list(t1); y_pid=list(pid)

        # TEMPERATURE FULL
        line_tcc_full.set_data(x, y_tcc)
        line_tc_full.set_data(x, y_tc)
        line_t1_full.set_data(x, y_t1)        
        
        if len(x) > 1:
            ax_temp_full.set_xlim(x[0], x[-1])

        allf = y_tcc + y_t1
        ymin, ymax = min(allf), max(allf)
        pad = max(0.2, (ymax - ymin) * 0.1)
        ax_temp_full.set_ylim(ymin - pad, ymax + pad)

        # PID FULL
        line_pid_full.set_data(x, y_pid)
        ymin=-200
        ymax=200
        pad=20
        ax_pid_full.set_ylim(ymin - pad, ymax + pad)

        if len(x) > 1:
            ax_pid_full.set_xlim(x[0], x[-1])

        # ZOOM TEMPERATURE
        tmin = max(0, x[-1] - ZOOM_WINDOW_S)
        idx0 = 0        
        for i in range(len(x) - 1, -1, -1):
            if x[i] < tmin:
                idx0 = i
                break

        xz = x[idx0:]; tccz = y_tcc[idx0:]; tcz = y_tc[idx0:]; t1z = y_t1[idx0:]; pidz = y_pid[idx0:]; 
        line_tcc_zoom.set_data(xz, tccz)
        line_tc_zoom.set_data(xz, tcz)
        line_t1_zoom.set_data(xz, t1z)
        
        if len(xz) > 1:
            ax_temp_zoom.set_xlim(tmin, x[-1])

        allz = tcz + t1z
        ymin, ymax = min(allz), max(allz)
        pad = max(0.05, (ymax - ymin) * 0.3)
        ax_temp_zoom.set_ylim(ymin - pad, ymax + pad)

        # ZOOM PID
        line_pid_zoom.set_data(xz, pidz)
        if len(xz) > 1:
            ax_pid_zoom.set_xlim(tmin, x[-1])        

        ymin, ymax = min(pidz), max(pidz)
        pad=20
        ax_pid_zoom.set_ylim(ymin - pad, ymax + pad)


        fig.canvas.draw()
        fig.canvas.flush_events()

except Exception as e:
    print("Erreur :", e)

finally:
    ser.close()
    plt.ioff()
    plt.close(fig)
    print("Arrêt OK — port COM libéré")
