

import sys, time, math, os, signal, statistics as stats
from datetime import time
from collections import deque, defaultdict

from pymavlink import mavutil
from jinja2 import Template

# =========================
# CONFIGURAÇÕES
# =========================
MAV_ENDPOINT = os.environ.get("MAV_ENDPOINT", "udp:127.0.0.1:14550")
#MAV_ENDPOINT = udp:127.0.0.1:14550 --> padrão para qgroundcontrol roda no mesmo PC
#MAV_ENDPOINT = udp:192.168.1.50:14550 → se usar Wi-Fi/telemetria.
RUN_SECONDS  = int(os.environ.get("RUN_SECONDS", "120"))  # tempo de coleta antes de gerar o relatório
ROLL_WINDOW_S = 15  # janela em segundos para estatísticas
RC_NEUTRAL = 1500
RC_TOL = 20

# Limiares (ajuste conforme plataforma)
ATT_NEUTRAL_RAD = 0.05        # ~2.8°
ATT_DRIFT_RAD   = 0.15        # ~8.6°
HDOP_BAD        = 2.5
BARO_GPS_ALT_D  = 5.0         # m
VIBE_ALERT      = 30.0        # m/s^2 (px4 costuma escalar; use como indicativo)
PWM_SAT_MIN     = 1100
PWM_SAT_MAX     = 1900
PWM_SAT_HOLD_MS = 800
ROLL_RATE_BAD   = 0.5         # rad/s como proxy p/ SP ruim (quando não há ATTITUDE_TARGET completo)

# =========================
# UTIL
# =========================
def clamp(x, lo, hi): return max(lo, min(hi, x))

def ts():
    return datetime.utcnow().strftime("%Y-%m-%d %H:%M:%SZ")

def rad2deg(r): return r * 180.0 / math.pi

# =========================
# COLETA
# =========================
class Rolling:
    """ janelas deslizantes por campo """
    def __init__(self, seconds):
        self.seconds = seconds
        self.buf = deque()  # (t, dict_campos)

    def add(self, t, d):
        self.buf.append((t, d))
        self._trim(t)

    def _trim(self, tnow):
        while self.buf and (tnow - self.buf[0][0]) > self.seconds:
            self.buf.popleft()

    def last(self, key, default=None):
        for t, d in reversed(self.buf):
            if key in d: return d[key]
        return default

    def series(self, key):
        return [d.get(key) for _, d in self.buf if key in d]

    def stats(self, key):
        xs = self.series(key)
        if not xs: return None
        return dict(
            n=len(xs),
            mean=sum(xs)/len(xs),
            min=min(xs), max=max(xs),
            stdev=(stats.pstdev(xs) if len(xs) > 1 else 0.0)
        )

class Collector:
    def __init__(self, roll_secs=ROLL_WINDOW_S):
        self.roll = Rolling(roll_secs)
        self.motor_sat_ms = 0
        self.motor_sat_flag = False
        self.last_ms = int(time.time()*1000)

        # caches “atuais”
        self.att = {}         # roll, pitch, yaw
        self.sp  = {}         # roll_sp, pitch_sp (proxy)
        self.rc  = {}         # chan1..4
        self.gps = {}         # fix_type, hdop, alt
        self.baro = {}        # alt proxy
        self.vibe = {}        # vibration metrics
        self.sys  = {}        # bateria/erros
        self.pwm  = []        # servo outputs

    def feed(self, msg):
        now = time.time()
        mtype = msg.get_type()

        if mtype == "ATTITUDE":
            self.att = dict(
                roll=float(msg.roll),
                pitch=float(msg.pitch),
                yaw=float(msg.yaw)
            )
            self.roll.add(now, {"att_roll": self.att["roll"], "att_pitch": self.att["pitch"]})

        elif mtype in ("ATTITUDE_TARGET", "SET_ATTITUDE_TARGET"):
            # alguns firmwares não publicam Euler; pega body rates como proxy
            self.sp = dict(
                roll_sp=float(getattr(msg, "body_roll_rate", 0.0)),
                pitch_sp=float(getattr(msg, "body_pitch_rate", 0.0)),
                yaw_sp=float(getattr(msg, "body_yaw_rate", 0.0)),
            )
            self.roll.add(now, {"sp_roll": self.sp["roll_sp"], "sp_pitch": self.sp["pitch_sp"]})

        elif mtype == "RC_CHANNELS":
            self.rc = dict(
                roll=int(getattr(msg, "chan1_raw", RC_NEUTRAL)),
                pitch=int(getattr(msg, "chan2_raw", RC_NEUTRAL)),
                throttle=int(getattr(msg, "chan3_raw", 1000)),
                yaw=int(getattr(msg, "chan4_raw", RC_NEUTRAL)),
            )
            self.roll.add(now, {"rc_roll": self.rc["roll"], "rc_pitch": self.rc["pitch"]})

        elif mtype == "GPS_RAW_INT":
            self.gps = dict(
                fix_type=int(msg.fix_type),
                hdop=float(msg.eph)/100.0 if msg.eph else 99.0,
                alt=float(msg.alt)/1000.0 if msg.alt else None
            )
            self.roll.add(now, {"gps_hdop": self.gps["hdop"]})

        elif mtype == "SCALED_PRESSURE":
            # sem conversão precisa — usamos como proxy de estabilidade do baro (pode adaptar)
            # muitos preferem BARO_ALT/ALTITUDE msgs; depende do firmware.
            self.baro = dict(press_abs=float(msg.press_abs))
            self.roll.add(now, {"baro_press": self.baro["press_abs"]})

        elif mtype == "VIBRATION":
            # PX4 publica vibration_x/y/z (m/s^2 RMS aprox)
            self.vibe = dict(vx=float(msg.vibration_x), vy=float(msg.vibration_y), vz=float(msg.vibration_z))
            self.roll.add(now, {"vibe_vx": self.vibe["vx"], "vibe_vy": self.vibe["vy"], "vibe_vz": self.vibe["vz"]})

        elif mtype in ("SERVO_OUTPUT_RAW", "RC_OUT"):
            vals = []
            for k in ("servo1_raw","servo2_raw","servo3_raw","servo4_raw",
                      "servo5_raw","servo6_raw","servo7_raw","servo8_raw"):
                if hasattr(msg, k) and getattr(msg,k):
                    vals.append(int(getattr(msg,k)))
            self.pwm = vals
            self.roll.add(now, {"pwm_avg": sum(vals)/len(vals) if vals else None})
            self._update_motor_sat()

        elif mtype == "SYS_STATUS":
            self.sys = dict(
                vbat=(float(msg.voltage_battery)/1000.0 if msg.voltage_battery else None),
                load= float(msg.load)/10.0 if msg.load else None,
                errors= int(msg.errors_count1 or 0) + int(msg.errors_count2 or 0) \
                        + int(msg.errors_count3 or 0) + int(msg.errors_count4 or 0)
            )

    def _update_motor_sat(self):
        now_ms = int(time.time()*1000)
        dt = now_ms - self.last_ms
        self.last_ms = now_ms
        if self.pwm:
            if any(v >= PWM_SAT_MAX or v <= PWM_SAT_MIN for v in self.pwm):
                self.motor_sat_ms += dt
            else:
                self.motor_sat_ms = 0
            self.motor_sat_flag = self.motor_sat_ms > PWM_SAT_HOLD_MS

# =========================
# DIAGNÓSTICO
# =========================
class Diagnoser:
    def __init__(self, coll: Collector):
        self.c = coll
        self.issues = []     # mensagens simples
        self.findings = []   # (titulo, explicação técnica, evidências)
        self.causes  = []    # hipóteses combinadas

    def rc_neutral(self):
        rc = self.c.rc
        if not rc: return None
        return abs(rc["roll"]-RC_NEUTRAL) < RC_TOL and abs(rc["pitch"]-RC_NEUTRAL) < RC_TOL

    def run(self):
        # --- RADIO ---
        if self.c.rc:
            if abs(self.c.rc["roll"]-RC_NEUTRAL) > RC_TOL or abs(self.c.rc["pitch"]-RC_NEUTRAL) > RC_TOL:
                self.issues.append("Rádio fora do neutro (trim)")
                self.findings.append((
                    "Trim do rádio possivelmente incorreto",
                    "Canais de roll/pitch em repouso não estão próximos a 1500 µs.",
                    f"RC roll={self.c.rc['roll']}, pitch={self.c.rc['pitch']}, tolerância ±{RC_TOL}."
                ))

        # --- GPS ---
        if self.c.gps:
            if self.c.gps["fix_type"] < 3:
                self.issues.append("GPS sem fix 3D")
                self.findings.append((
                    "GPS sem fix 3D",
                    "O fix type reportado pelo GPS é menor que 3 (sem solução 3D).",
                    f"fix_type={self.c.gps['fix_type']}"
                ))
            elif self.c.gps["hdop"] > HDOP_BAD:
                self.issues.append("GPS impreciso (HDOP alto)")
                self.findings.append((
                    "Precisão GPS insuficiente",
                    "HDOP acima do recomendado reduz a confiabilidade de modos baseados em posição.",
                    f"HDOP={self.c.gps['hdop']:.2f} (limite {HDOP_BAD})"
                ))

        # --- VIBRAÇÃO ---
        if self.c.vibe:
            vmax = max(abs(self.c.vibe.get("vx",0)), abs(self.c.vibe.get("vy",0)), abs(self.c.vibe.get("vz",0)))
            if vmax > VIBE_ALERT:
                self.issues.append("Vibração elevada")
                self.findings.append((
                    "Vibração acima do ideal",
                    "Vibração alta contamina a IMU, afeta D-term e pode causar deriva/instabilidade.",
                    f"VIBE max (último) ≈ {vmax:.1f} m/s² (limite indicativo {VIBE_ALERT})"
                ))

        # --- HOVER/ATT vs SP ---
        att = self.c.att
        sp  = self.c.sp
        if att:
            # sem ATTITUDE_TARGET, usamos proxy: se rates SP ~0 (ou faltando), ainda avaliamos atitude
            sp_roll  = sp.get("roll_sp", 0.0) if sp else 0.0
            sp_pitch = sp.get("pitch_sp", 0.0) if sp else 0.0

            neutral_cmd = self.rc_neutral()
            if neutral_cmd and abs(sp_roll) < ATT_NEUTRAL_RAD and abs(sp_pitch) < ATT_NEUTRAL_RAD:
                if abs(att["pitch"]) > ATT_DRIFT_RAD or abs(att["roll"]) > ATT_DRIFT_RAD:
                    self.issues.append("Hover inclinado com comandos neutros")
                    self.findings.append((
                        "Hover inclinado com setpoint neutro",
                        "Comandos neutros e setpoint ~0, mas atitude apresenta inclinação persistente.",
                        f"att.roll={rad2deg(att['roll']):.1f}°, att.pitch={rad2deg(att['pitch']):.1f}°; "
                        f"sp_roll≈{sp_roll:.2f}, sp_pitch≈{sp_pitch:.2f}"
                    ))

        # --- SATURAÇÃO MOTOR ---
        if self.c.motor_sat_flag:
            self.issues.append("Motores em saturação prolongada")
            self.findings.append((
                "Saída de motor saturada",
                "Comandos de motor ficaram próximos ao limite por período prolongado, indicando tuning inadequado, CG deslocado ou empuxo insuficiente.",
                f"PWM sat > {PWM_SAT_HOLD_MS} ms; últimos PWM={self.c.pwm}"
            ))

        # --- DIVERGÊNCIA BARO/GPS ---
        if self.c.gps and self.c.gps.get("alt") is not None and self.c.baro and "press_abs" in self.c.baro:
            # proxy simples: variação baro não acompanhando GPS (heurístico)
            # como não convertimos pressão em altitude, avaliamos estabilidade/consistência na janela
            # Para um check direto de altitude, prefira mensagens ALTITUDE ou calcular baro_alt.
            pass  # mantemos para logs; divergência real depende da msg ALTITUDE

        # =========================
        # COMBINAÇÕES (CAUSA PROVÁVEL)
        # =========================
        txt = " ".join(self.issues)

        if "Hover inclinado com comandos neutros" in self.issues:
            if "Rádio fora do neutro (trim)" in self.issues:
                self.causes.append((
                    "Causa provável: Trim do rádio incorreto (não é apenas PID).",
                    "Como os canais de RC não estão centrados e o hover aparece inclinado com setpoint neutro, a integradora corrige na direção errada. Ajuste subtrim/enderece calibração do rádio antes de mexer em PID."
                ))
            elif "Motores em saturação prolongada" in self.issues:
                self.causes.append((
                    "Causa provável: Centro de gravidade deslocado / empuxo insuficiente.",
                    "Hover inclinado com saturação de motor indica torque/empuxo desequilibrados (bateria/câmera muito à frente/trás) ou limite de potência. Reposicione a bateria/carga ou aumente empuxo (hélices/ESC)."
                ))
            elif "Vibração elevada" in self.issues:
                self.causes.append((
                    "Causa provável: Vibração na IMU afetando controle (D-term).",
                    "Vibração alta pode induzir leituras erradas de atitude e levar o PID a 'lutar' para manter hover. Balanceie hélices, isole a FC, ajuste filtros (LPF/notch)."
                ))
            else:
                self.causes.append((
                    "Causa provável: I-term (integral) acumulando / IMU com offset.",
                    "Comandos neutros e setpoint ~0 mas atitude inclinada persistente é assinatura de I-term desbalanceado ou de offset residual na IMU (nível/acc). Refaça 'Level' e limite IMAX do I-term; avalie reduzir I em pitch/roll."
                ))

        if "GPS impreciso (HDOP alto)" in txt or "GPS sem fix 3D" in txt:
            self.causes.append((
                "Impacto no PosHold/Mission",
                "Precisão/lock do GPS insuficientes degradam modos baseados em posição (Loiter/Mission/RTL). Melhore antena, coloque longe de ESCs, verifique céu aberto e grounding."
            ))

        if "Rádio fora do neutro (trim)" in self.issues and not self.causes:
            self.causes.append((
                "Causa provável: Rádio mal calibrado.",
                "Canais não centrados (≠1500 µs). Execute calibração no QGC e aplique deadband 3–5 µs."
            ))

        return dict(
            issues=self.issues,
            findings=self.findings,
            causes=self.causes
        )

# =========================
# RELATÓRIO
# =========================
REPORT_TMPL = """
    <!doctype html>
    <html lang="pt-br">
    <head>
    <meta charset="utf-8">
    <title>Relatório de Diagnóstico – {{ ts }}</title>
    <style>
    body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Arial,sans-serif;margin:32px;color:#111}
    h1{margin:0 0 8px} h2{margin-top:28px}
    .code{background:#f5f5f7;border:1px solid #eee;padding:8px;border-radius:6px;font-family:ui-monospace,Consolas,monospace;font-size:13px;white-space:pre-wrap}
    .bad{color:#b00020} .warn{color:#a15c00} .ok{color:#0b7}
    .card{border:1px solid #e5e7eb;border-radius:10px;padding:12px;margin:10px 0;background:#fff}
    small{color:#555}
    ul{margin:6px 0 6px 20px}
    </style>
    </head>
    <body>
    <h1>Relatório de Diagnóstico</h1>
    <small>Gerado em {{ ts }} | Janela analisada: {{ window }} s | Endpoint: {{ endpoint }}</small>

    <h2>Sumário</h2>
    <div class="card">
    <ul>
        {% if diag.causes %}
        {% for t,exp in diag.causes %}
        <li><b>{{ t }}</b> — {{ exp }}</li>
        {% endfor %}
        {% else %}
        <li class="ok">Nenhuma causa provável crítica detectada com os dados coletados.</li>
        {% endif %}
    </ul>
    </div>

    <h2>Diagnósticos & Evidências</h2>
    {% if diag.findings %}
    {% for title,exp,ev in diag.findings %}
    <div class="card">
        <b>{{ title }}</b><br>
        <small>{{ exp }}</small>
        <div class="code">{{ ev }}</div>
    </div>
    {% endfor %}
    {% else %}
    <div class="card ok">Sem achados relevantes na janela de análise.</div>
    {% endif %}

    <h2>Métricas (últimos {{ window }} s)</h2>
    <div class="card">
    <pre class="code">
    ATT (roll/pitch deg): mean=({{ m.att_roll_mean|default("na") }}, {{ m.att_pitch_mean|default("na") }})
    ATT stdev:            ({{ m.att_roll_std|default("na") }}, {{ m.att_pitch_std|default("na") }})
    RC roll/pitch median: ({{ m.rc_roll_med|default("na") }}, {{ m.rc_pitch_med|default("na") }})  target≈1500
    GPS HDOP mean:        {{ m.gps_hdop_mean|default("na") }}
    VIBE max last:        {{ m.vibe_max_last|default("na") }}
    PWM avg mean:         {{ m.pwm_avg_mean|default("na") }}
    </pre>
    </div>

    <h2>Thresholds usados</h2>
    <div class="card">
    <pre class="code">
    RC tolerância neutro: ±{{ RC_TOL }} µs (alvo 1500)
    Setpoint neutro:      &lt; {{ ATT_NEUTRAL_RAD }} rad (~{{ (ATT_NEUTRAL_RAD*57.2958)|round(1) }}°)
    Drift atitude:        &gt; {{ ATT_DRIFT_RAD }} rad (~{{ (ATT_DRIFT_RAD*57.2958)|round(1) }}°)
    GPS HDOP ruim:        &gt; {{ HDOP_BAD }}
    Baro vs GPS alt:      &gt; {{ BARO_GPS_ALT_D }} m (quando disponível)
    Vibração alerta:      &gt; {{ VIBE_ALERT }} m/s²
    Saturação PWM:        &lt;={{ PWM_SAT_MIN }} ou &gt;={{ PWM_SAT_MAX }} por &gt; {{ PWM_SAT_HOLD_MS }} ms
    </pre>
    </div>

    <h2>Recomendações</h2>
    <div class="card">
    <ul>
        <li>Confirme <b>calibração do rádio</b> (centros 1500 µs) e aplique <i>deadband</i> 3–5 µs.</li>
        <li>Refaça <b>Acc/Gyro Cal</b> e <b>Set Level</b>; faça <b>Compass Cal</b> longe de metais.</li>
        <li>Verifique <b>CG</b> (bateria/carga centralizadas). Se hover inclina com motor saturando, reposicione carga.</li>
        <li>Balanceie <b>hélices</b>, verifique <b>montagem antivibração</b> e ajuste <b>filtros</b> (LPF/notch).</li>
        <li>Em GPS: garanta <b>céu aberto</b>, antena longe de ESCs, e cabos limpos de EMI.</li>
        <li>Se persistir: reduza <b>I-term</b> ou limite <b>IMAX</b> em pitch/roll; depois refine P/D ou use autotune guiado.</li>
    </ul>
    </div>

    </body>
    </html>
"""

def render_report(endpoint, window_s, diag, metrics):
    tmpl = Template(REPORT_TMPL)
    html = tmpl.render(
        ts=ts(),
        endpoint=endpoint,
        window=window_s,
        diag=diag,
        m=metrics,
        RC_TOL=RC_TOL,
        ATT_NEUTRAL_RAD=ATT_NEUTRAL_RAD,
        ATT_DRIFT_RAD=ATT_DRIFT_RAD,
        HDOP_BAD=HDOP_BAD,
        BARO_GPS_ALT_D=BARO_GPS_ALT_D,
        VIBE_ALERT=VIBE_ALERT,
        PWM_SAT_MIN=PWM_SAT_MIN,
        PWM_SAT_MAX=PWM_SAT_MAX,
        PWM_SAT_HOLD_MS=PWM_SAT_HOLD_MS,
    )
    fn = f"report_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}.html"
    with open(fn, "w", encoding="utf-8") as f:
        f.write(html)
    return fn

def build_metrics(coll: Collector):
    r = coll.roll
    def getstat(name, field):
        s = r.stats(field)
        return s[name] if s else None

    vibe_last = None
    if coll.vibe:
        vibe_last = max(abs(coll.vibe.get("vx",0)), abs(coll.vibe.get("vy",0)), abs(coll.vibe.get("vz",0)))

    return dict(
        att_roll_mean = (rad2deg(getstat("mean","att_roll")) if getstat("mean","att_roll") is not None else None),
        att_pitch_mean= (rad2deg(getstat("mean","att_pitch")) if getstat("mean","att_pitch") is not None else None),
        att_roll_std  = (rad2deg(getstat("stdev","att_roll")) if getstat("stdev","att_roll") is not None else None),
        att_pitch_std = (rad2deg(getstat("stdev","att_pitch")) if getstat("stdev","att_pitch") is not None else None),
        rc_roll_med   = (stats.median(r.series("rc_roll")) if r.series("rc_roll") else None),
        rc_pitch_med  = (stats.median(r.series("rc_pitch")) if r.series("rc_pitch") else None),
        gps_hdop_mean = (getstat("mean","gps_hdop")),
        pwm_avg_mean  = (getstat("mean","pwm_avg")),
        vibe_max_last = vibe_last
    )

# =========================
# MAIN
# =========================
def main():
    print(f"[{ts()}] Conectando MAVLink em {MAV_ENDPOINT}…")
    master = mavutil.mavlink_connection(MAV_ENDPOINT)
    master.wait_heartbeat()
    print(f"[{ts()}] ✅ Heartbeat recebido. Coletando por {RUN_SECONDS}s…")

    coll = Collector()
    end_time = time.time() + RUN_SECONDS

    # permitir Ctrl+C para terminar cedo
    def handler(sig, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, handler)

    try:
        while time.time() < end_time:
            msg = master.recv_match(blocking=True, timeout=1)
            if not msg: 
                continue
            coll.feed(msg)
    except KeyboardInterrupt:
        print(f"\n[{ts()}] Interrompido pelo usuário. Gerando relatório com dados coletados até agora…")

    # Diagnóstico + relatório
    diag = Diagnoser(coll).run()
    metrics = build_metrics(coll)
    out = render_report(MAV_ENDPOINT, ROLL_WINDOW_S, diag, metrics)
    print(f"[{ts()}] 📄 Relatório salvo: {out}")

if __name__ == "__main__":
    main()
