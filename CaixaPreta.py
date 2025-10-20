#!/usr/bin/env python3
import os, sys, time
from pymavlink import mavutil
from datetime import datetime
from statistics import mean, stdev

# ╔════════════════════════════════════════════════════════════╗
# ║                    MENU DE CONFIGURAÇÃO                    ║
# ╚════════════════════════════════════════════════════════════╝
def user_menu():
    print("=== Drone Diagnostic V2.5 ===\n")

    print("1) Método de conexão:")
    print("   [1] Telemetria (UDP via rádio HT0X)")
    print("   [2] USB direto na PX4 (/dev/ttyACM0)")
    conn = input("Escolha (1 ou 2): ").strip()
    mav_endpoint = "udp:127.0.0.1:14550" if conn == "1" else "/dev/ttyACM0"

    roll_window_s = int(input("2) Screen Width (janela de análise, em segundos): ") or "10")
    run_seconds = int(input("3) Tempo de coleta (segundos): ") or "120")
    rc_neutral = int(input("4) Valor de RC_NEUTRAL (default 1500): ") or "1500")

    return mav_endpoint, roll_window_s, run_seconds, rc_neutral


# ╔════════════════════════════════════════════════════════════╗
# ║              CONEXÃO MAVLINK E COLETA DE DADOS             ║
# ╚════════════════════════════════════════════════════════════╝
def conectar_mavlink(endpoint):
    print(f"\n🔌 Conectando ao MAVLink em {endpoint} ...")
    if endpoint.startswith("udp"):
        master = mavutil.mavlink_connection(endpoint)
    else:
        master = mavutil.mavlink_connection(endpoint, baud=57600)

    print("Esperando heartbeat...")
    master.wait_heartbeat()
    print(f"✅ Conectado (SysID={master.target_system}, CompID={master.target_component})")
    return master


def coletar_dados(master, run_seconds, rc_neutral):
    start = time.time()
    dados = {
        "att": [], "rc": [], "gps": [], "vib": []
    }

    print(f"\n⏳ Coletando dados por {run_seconds}s (pressione Ctrl+C para parar)...\n")

    try:
        while time.time() - start < run_seconds:
            msg = master.recv_match(blocking=True, timeout=1)
            if not msg:
                continue

            tipo = msg.get_type()
            if tipo == "ATTITUDE":
                dados["att"].append((msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw))
            elif tipo == "RC_CHANNELS":
                dados["rc"].append((msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw))
            elif tipo == "GPS_RAW_INT":
                dados["gps"].append((msg.satellites_visible, msg.hdop/100.0))
            elif tipo == "VIBRATION":
                dados["vib"].append((msg.vibration_x, msg.vibration_y, msg.vibration_z))

            # monitoramento ao vivo
            if tipo in ["ATTITUDE", "RC_CHANNELS", "GPS_RAW_INT"]:
                os.system('clear')
                print("=== MONITOR DE VOO (Tempo real) ===")
                if dados["att"]:
                    r, p, y = dados["att"][-1][1:]
                    print(f"🧭 Attitude → Roll: {r:.2f} | Pitch: {p:.2f} | Yaw: {y:.2f}")
                if dados["rc"]:
                    ch = dados["rc"][-1]
                    print(f"🎮 RC → CH1:{ch[0]} CH2:{ch[1]} CH3:{ch[2]} CH4:{ch[3]}")
                if dados["gps"]:
                    sats, hdop = dados["gps"][-1]
                    print(f"📡 GPS → Satélites:{sats} | HDOP:{hdop:.2f}")
                if dados["vib"]:
                    vx, vy, vz = dados["vib"][-1]
                    print(f"💢 Vibração RMS → X:{vx:.2f} Y:{vy:.2f} Z:{vz:.2f}")
                print(f"\nTempo restante: {int(run_seconds - (time.time()-start))}s")
    except KeyboardInterrupt:
        print("\n⛔ Coleta interrompida manualmente.")

    return dados


# ╔════════════════════════════════════════════════════════════╗
# ║                     ANÁLISE DE DIAGNÓSTICO                 ║
# ╚════════════════════════════════════════════════════════════╝
def diagnosticar(dados, rc_neutral):
    problemas = []

    # 1. GPS
    if not dados["gps"]:
        problemas.append("❌ Nenhum dado GPS recebido (módulo desconectado ou falha).")
    else:
        sats = [g[0] for g in dados["gps"]]
        hdops = [g[1] for g in dados["gps"]]
        if mean(sats) < 6:
            problemas.append(f"⚠️ Sinal GPS fraco: média {mean(sats):.1f} satélites.")
        if mean(hdops) > 2.0:
            problemas.append(f"⚠️ HDOP alto ({mean(hdops):.2f}) — baixa precisão de posição.")

    # 2. Rádio (Trim / Calibração)
    if dados["rc"]:
        ch1_vals = [rc[0] for rc in dados["rc"]]
        desv = abs(mean(ch1_vals) - rc_neutral)
        if desv > 70:
            problemas.append(f"⚠️ Trim do rádio desajustado ({desv:.1f} µs de diferença).")
    else:
        problemas.append("❌ Nenhum dado RC_CHANNELS recebido (verifique receptor).")

    # 3. Vibração / IMU
    if dados["vib"]:
        vx = [v[0] for v in dados["vib"]]
        rms_v = (mean(vx) if vx else 0)
        if rms_v > 30:
            problemas.append("⚠️ Vibração excessiva detectada — possível montagem solta ou hélice desbalanceada.")

    # 4. Attitude (PID / CG)
    if dados["att"]:
        rolls = [a[1] for a in dados["att"]]
        pitchs = [a[2] for a in dados["att"]]
        roll_drift = abs(mean(rolls))
        pitch_drift = abs(mean(pitchs))
        if roll_drift > 0.2 or pitch_drift > 0.2:
            problemas.append(f"⚠️ Deriva de atitude (Roll={roll_drift:.2f}, Pitch={pitch_drift:.2f}) — possível CG deslocado.")
    else:
        problemas.append("❌ Nenhum dado ATTITUDE — IMU pode estar mal calibrada.")

    # 5. Avaliação geral
    if not problemas:
        problemas.append("✅ Todos os sistemas parecem operacionais.")

    return problemas


# ╔════════════════════════════════════════════════════════════╗
# ║                       RELATÓRIO FINAL                      ║
# ╚════════════════════════════════════════════════════════════╝
def salvar_relatorio(problemas):
    nome = f"relatorio_drone_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    with open(nome, "w") as f:
        f.write("=== RELATÓRIO DE DIAGNÓSTICO DE DRONE V2.5 ===\n\n")
        for p in problemas:
            f.write(p + "\n")
        f.write("\nLegenda:\n⚠️ = Atenção | ❌ = Falha crítica | ✅ = OK\n")
    print(f"\n📂 Relatório salvo em: {nome}")


# ╔════════════════════════════════════════════════════════════╗
# ║                           MAIN                             ║
# ╚════════════════════════════════════════════════════════════╝
if __name__ == "__main__":
    mav_endpoint, roll_window_s, run_seconds, rc_neutral = user_menu()
    master = conectar_mavlink(mav_endpoint)
    dados = coletar_dados(master, run_seconds, rc_neutral)
    problemas = diagnosticar(dados, rc_neutral)
    salvar_relatorio(problemas)
#!/usr/bin/env python3
import os, sys, time
from pymavlink import mavutil
from datetime import datetime
from statistics import mean, stdev

# ╔════════════════════════════════════════════════════════════╗
# ║                    MENU DE CONFIGURAÇÃO                    ║
# ╚════════════════════════════════════════════════════════════╝
def user_menu():
    print("=== Drone Diagnostic V2.5 ===\n")

    print("1) Método de conexão:")
    print("   [1] Telemetria (UDP via rádio HT0X)")
    print("   [2] USB direto na PX4 (/dev/ttyACM0)")
    conn = input("Escolha (1 ou 2): ").strip()
    mav_endpoint = "udp:127.0.0.1:14550" if conn == "1" else "/dev/ttyACM0"

    roll_window_s = int(input("2) Screen Width (janela de análise, em segundos): ") or "10")
    run_seconds = int(input("3) Tempo de coleta (segundos): ") or "120")
    rc_neutral = int(input("4) Valor de RC_NEUTRAL (default 1500): ") or "1500")

    return mav_endpoint, roll_window_s, run_seconds, rc_neutral


# ╔════════════════════════════════════════════════════════════╗
# ║              CONEXÃO MAVLINK E COLETA DE DADOS             ║
# ╚════════════════════════════════════════════════════════════╝
def conectar_mavlink(endpoint):
    print(f"\n🔌 Conectando ao MAVLink em {endpoint} ...")
    if endpoint.startswith("udp"):
        master = mavutil.mavlink_connection(endpoint)
    else:
        master = mavutil.mavlink_connection(endpoint, baud=57600)

    print("Esperando heartbeat...")
    master.wait_heartbeat()
    print(f"✅ Conectado (SysID={master.target_system}, CompID={master.target_component})")
    return master


def coletar_dados(master, run_seconds, rc_neutral):
    start = time.time()
    dados = {
        "att": [], "rc": [], "gps": [], "vib": []
    }

    print(f"\n⏳ Coletando dados por {run_seconds}s (pressione Ctrl+C para parar)...\n")

    try:
        while time.time() - start < run_seconds:
            msg = master.recv_match(blocking=True, timeout=1)
            if not msg:
                continue

            tipo = msg.get_type()
            if tipo == "ATTITUDE":
                dados["att"].append((msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw))
            elif tipo == "RC_CHANNELS":
                dados["rc"].append((msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw))
            elif tipo == "GPS_RAW_INT":
                dados["gps"].append((msg.satellites_visible, msg.hdop/100.0))
            elif tipo == "VIBRATION":
                dados["vib"].append((msg.vibration_x, msg.vibration_y, msg.vibration_z))

            # monitoramento ao vivo
            if tipo in ["ATTITUDE", "RC_CHANNELS", "GPS_RAW_INT"]:
                os.system('clear')
                print("=== MONITOR DE VOO (Tempo real) ===")
                if dados["att"]:
                    r, p, y = dados["att"][-1][1:]
                    print(f"🧭 Attitude → Roll: {r:.2f} | Pitch: {p:.2f} | Yaw: {y:.2f}")
                if dados["rc"]:
                    ch = dados["rc"][-1]
                    print(f"🎮 RC → CH1:{ch[0]} CH2:{ch[1]} CH3:{ch[2]} CH4:{ch[3]}")
                if dados["gps"]:
                    sats, hdop = dados["gps"][-1]
                    print(f"📡 GPS → Satélites:{sats} | HDOP:{hdop:.2f}")
                if dados["vib"]:
                    vx, vy, vz = dados["vib"][-1]
                    print(f"💢 Vibração RMS → X:{vx:.2f} Y:{vy:.2f} Z:{vz:.2f}")
                print(f"\nTempo restante: {int(run_seconds - (time.time()-start))}s")
    except KeyboardInterrupt:
        print("\n⛔ Coleta interrompida manualmente.")

    return dados


# ╔════════════════════════════════════════════════════════════╗
# ║                     ANÁLISE DE DIAGNÓSTICO                 ║
# ╚════════════════════════════════════════════════════════════╝
def diagnosticar(dados, rc_neutral):
    problemas = []

    # 1. GPS
    if not dados["gps"]:
        problemas.append("❌ Nenhum dado GPS recebido (módulo desconectado ou falha).")
    else:
        sats = [g[0] for g in dados["gps"]]
        hdops = [g[1] for g in dados["gps"]]
        if mean(sats) < 6:
            problemas.append(f"⚠️ Sinal GPS fraco: média {mean(sats):.1f} satélites.")
        if mean(hdops) > 2.0:
            problemas.append(f"⚠️ HDOP alto ({mean(hdops):.2f}) — baixa precisão de posição.")

    # 2. Rádio (Trim / Calibração)
    if dados["rc"]:
        ch1_vals = [rc[0] for rc in dados["rc"]]
        desv = abs(mean(ch1_vals) - rc_neutral)
        if desv > 70:
            problemas.append(f"⚠️ Trim do rádio desajustado ({desv:.1f} µs de diferença).")
    else:
        problemas.append("❌ Nenhum dado RC_CHANNELS recebido (verifique receptor).")

    # 3. Vibração / IMU
    if dados["vib"]:
        vx = [v[0] for v in dados["vib"]]
        rms_v = (mean(vx) if vx else 0)
        if rms_v > 30:
            problemas.append("⚠️ Vibração excessiva detectada — possível montagem solta ou hélice desbalanceada.")

    # 4. Attitude (PID / CG)
    if dados["att"]:
        rolls = [a[1] for a in dados["att"]]
        pitchs = [a[2] for a in dados["att"]]
        roll_drift = abs(mean(rolls))
        pitch_drift = abs(mean(pitchs))
        if roll_drift > 0.2 or pitch_drift > 0.2:
            problemas.append(f"⚠️ Deriva de atitude (Roll={roll_drift:.2f}, Pitch={pitch_drift:.2f}) — possível CG deslocado.")
    else:
        problemas.append("❌ Nenhum dado ATTITUDE — IMU pode estar mal calibrada.")

    # 5. Avaliação geral
    if not problemas:
        problemas.append("✅ Todos os sistemas parecem operacionais.")

    return problemas


# ╔════════════════════════════════════════════════════════════╗
# ║                       RELATÓRIO FINAL                      ║
# ╚════════════════════════════════════════════════════════════╝
def salvar_relatorio(problemas):
    nome = f"relatorio_drone_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    with open(nome, "w") as f:
        f.write("=== RELATÓRIO DE DIAGNÓSTICO DE DRONE V2.5 ===\n\n")
        for p in problemas:
            f.write(p + "\n")
        f.write("\nLegenda:\n⚠️ = Atenção | ❌ = Falha crítica | ✅ = OK\n")
    print(f"\n📂 Relatório salvo em: {nome}")


# ╔════════════════════════════════════════════════════════════╗
# ║                           MAIN                             ║
# ╚════════════════════════════════════════════════════════════╝
if __name__ == "__main__":
    mav_endpoint, roll_window_s, run_seconds, rc_neutral = user_menu()
    master = conectar_mavlink(mav_endpoint)
    dados = coletar_dados(master, run_seconds, rc_neutral)
    problemas = diagnosticar(dados, rc_neutral)
    salvar_relatorio(problemas)
