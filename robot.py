#!/usr/bin/env python3
# Proyecto ECO-SENSE 1.0
# Script principal de control del robot y captura de datos.

import os
import sys
import time
import serial  # Para leer el USB
import smtplib  # Para mandar el correo
import ssl
from email.message import EmailMessage
import csv  # Para guardar el historial
import datetime
import pandas as pd  # <-- Para procesar datos
import numpy as np  # <-- Para cálculos numéricos
import random  # <-- Para simular la Corriente

# Importar la API de xArm
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI

# --- 2. CONFIGURACIÓN DEL PROYECTO ---
IP_ROBOT = '192.168.1.153'
VELOCIDAD = 500
PAUSA_MEDICION = 10

PUERTO_SERIAL_ESP32 = "COM3"
BAUD_RATE = 115200
NOMBRE_ARCHIVO_CSV = 'historial_mediciones_robot.csv'
VOLTAJE_LINEA = 220

LIMITE_TEMP_ALTA = 60.0
LIMITE_CORRIENTE_SIM = 2.0
LIMITE_TEMP_BAJA = 10.0

MAIL_USUARIO = "juan07higuera@gmail.com"
MAIL_PASS_APP = ""
MAIL_DESTINO = "juan07higuera@gmail.com"

# Coordenadas de los 4 lotes de medición
CICLO_DE_MOVIMIENTO = [
    {"label": "Punto 1", "coords_acercamiento": (269, -279, 250, -180, 0, -90),
     "coords_medicion": (269, -279, 132, -180, 0, -90)},
    {"label": "Punto 2", "coords_acercamiento": (269, -130, 250, -180, 0, -90),
     "coords_medicion": (274, -130, 132, -180, 0, -90)},
    {"label": "Punto 3", "coords_acercamiento": (269, 60, 250, -180, 0, -90),
     "coords_medicion": (269, 60, 132, -180, 0, -90)},
    {"label": "Punto 4", "coords_acercamiento": (269, 277, 250, -180, 0, -90),
     "coords_medicion": (269, 277, 132, -180, 0, -90)},
]

# --- 3. PLANTILLA DE CORREO HTML ---
HTML_EMAIL_TEMPLATE = """
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <title>{asunto}</title>
</head>
<body style="font-family: Arial, 'Helvetica Neue', Helvetica, sans-serif; margin: 0; padding: 0; background-color: #f4f4f4;">
    <table width="100%" border="0" cellspacing="0" cellpadding="0">
        <tr>
            <td align="center" style="padding: 20px 0;">
                <table width="600" border="0" cellspacing="0" cellpadding="0" style="background-color: #ffffff; border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1);">
                    <tr>
                        <td align="center" style="padding: 20px; background-color: {color_alerta}; color: #ffffff; border-top-left-radius: 8px; border-top-right-radius: 8px;">
                            <h1 style="margin: 0; font-size: 24px;">ALERTA DE SISTEMA</h1>
                        </td>
                    </tr>
                    <tr>
                        <td style="padding: 30px 40px;">
                            <h2 style="color: #333; font-size: 20px;">Atención Equipo de Mantenimiento</h2>
                            <p style="color: #555; font-size: 16px; line-height: 1.6;">
                                Se ha detectado una condición de alerta en el sistema <strong>ECO-SENSE 1.0</strong>.
                            </p>
                            <p style="color: #555; font-size: 16px; line-height: 1.6;">
                                El robot ha detenido su ciclo automáticamente y requiere revisión inmediata.
                            </p>
                            <h3 style="color: #333; margin-top: 25px; border-bottom: 2px solid #f4f4f4; padding-bottom: 5px;">Detalles de la Alerta</h3>
                            <table width="100%" border="0" cellspacing="0" cellpadding="0" style="border: 1px solid #eee;">
                                <tr style="background-color: #f9f9f9;">
                                    <td style="padding: 12px; border-bottom: 1px solid #eee; font-size: 16px; color: #333; width: 40%;"><strong>Motivo de Alerta:</strong></td>
                                    <td style="padding: 12px; border-bottom: 1px solid #eee; font-size: 16px; color: {color_alerta};"><strong>{titulo_html}</strong></td>
                                </tr>
                                <tr>
                                    <td style="padding: 12px; border-bottom: 1px solid #eee; font-size: 16px; color: #333;"><strong>Fecha y Hora:</strong></td>
                                    <td style="padding: 12px; border-bottom: 1px solid #eee; font-size: 16px; color: #555;">{ahora}</td>
                                </tr>
                                <tr>
                                    <td style="padding: 12px; border-bottom: 1px solid #eee; font-size: 16px; color: #333;"><strong>Temperatura Detectada:</strong></td>
                                    <td style="padding: 12px; border-bottom: 1px solid #eee; font-size: 16px; color: #555;">{temp_str}</td>
                                </tr>
                                <tr>
                                    <td style="padding: 12px; font-size: 16px; color: #333;"><strong>Corriente (Simulada):</strong></td>
                                    <td style="padding: 12px; font-size: 16px; color: #555;">{corr_str}</td>
                                </tr>
                            </table>
                        </td>
                    </tr>
                    <tr>
                        <td align="center" style="padding: 20px; background-color: #f9f9f9; color: #999; border-bottom-left-radius: 8px; border-bottom-right-radius: 8px; font-size: 12px;">
                            Este es un mensaje automático del Sistema ECO-SENSE 1.0. Por favor, no responder.
                        </td>
                    </tr>
                </table>
            </td>
        </tr>
    </table>
</body>
</html>
"""


# --- 4. CLASE PRINCIPAL DEL ROBOT ---

class EcoSenseRobot:
    def __init__(self):
        """Inicializa las variables del robot."""
        self.ip_robot = IP_ROBOT
        self.mail_usuario = MAIL_USUARIO
        self.mail_pass_app = MAIL_PASS_APP
        self.mail_destino = MAIL_DESTINO
        
        self.arm = None
        self.contador_lote_global = 1
        self.lista_lecturas_global = []
        self.tiempo_inicio_script = time.time()

    def _leer_sensor_usb(self):
        """Lee los datos del ESP32 por el puerto USB."""
        temp, hum, volt = 0.0, 0.0, 0.0
        corr_sim = 0.0
        status_flag = 0  # 0=OK, 1=ALERTA_ALTA, 2=ALERTA_BAJA

        try:
            # Se usa un timeout de 3s para esperar al ESP32 (que tiene delay de 2s)
            ser = serial.Serial(PUERTO_SERIAL_ESP32, BAUD_RATE, timeout=3)
            time.sleep(0.1)
            ser.flushInput()
            linea = ser.readline().decode('utf-8').strip()
            ser.close()

            # Filtro para ignorar líneas vacías o de 'startup' del ESP32
            if not linea:
                print("[SERIAL] Línea vacía (Timeout). Ignorando.")
                return 0, 0, 0, 0, 0
            if "ErrorT_H" in linea:
                print("[SERIAL] No se recibieron datos (¡ErrorT_H!).")
                return 0, 0, 0, 0, 0
            if not (linea.startswith("OK:") or linea.startswith("ALERTA_ALTA:") or linea.startswith("ALERTA_BAJA:")):
                print(f"[SERIAL] Ignorando línea de startup: {linea}")
                return 0, 0, 0, 0, 0

            partes = linea.split(":")
            status_str = partes[0]
            datos = partes[1]
            datos_sensores = datos.split(",")

            if len(datos_sensores) < 3:
                print(f"[SERIAL] ¡ERROR! Línea mal formada: {linea}")
                return 0, 0, 0, 0, 0

            temp = float(datos_sensores[0].split("=")[1])
            hum = float(datos_sensores[1].split("=")[1])
            volt = float(datos_sensores[2].split("=")[1])

            # Decisión de Diseño: Se simula la corriente ya que el sensor ACS712
            # arrojó V=0.00. Esto permite validar la lógica de kWh y alertas.
            corr_sim = round(1.8 + random.uniform(-0.3, 0.4), 2)

            # Revisa si la alerta vino del ESP32 o de la simulación
            if status_str == "ALERTA_ALTA":
                status_flag = 1
            elif status_str == "ALERTA_BAJA":
                status_flag = 2

            if corr_sim >= LIMITE_CORRIENTE_SIM:
                print(f"[ALERTA SIM] Corriente simulada alta: {corr_sim:.2f}A")
                status_flag = 1

            return temp, hum, volt, corr_sim, status_flag

        except Exception as e:
            print(f"[SERIAL] ¡ERROR! No pude leer/procesar el puerto USB: {e}")
            return 0, 0, 0, 0, 0

    def _mover(self, coords):
        """Envía el comando de movimiento al robot."""
        self.arm.set_position(
            x=coords[0], y=coords[1], z=coords[2],
            roll=coords[3], pitch=coords[4], yaw=coords[5],
            speed=VELOCIDAD, wait=True
        )

    def _preparar_csv(self):
        """Crea el archivo CSV con las cabeceras."""
        try:
            with open(NOMBRE_ARCHIVO_CSV, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(
                    ['timestamp', 'lote', 'temperatura', 'humedad', 'voltaje_real', 'corriente_sim', 'piezas_contadas'])
            print(f"Archivo '{NOMBRE_ARCHIVO_CSV}' listo.")
            return True
        except Exception as e:
            print(f"¡ERROR! No pude crear el CSV: {e}")
            return False

    def _guardar_en_csv(self, datos_fila):
        """Añade una nueva fila de datos al CSV."""
        try:
            with open(NOMBRE_ARCHIVO_CSV, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(datos_fila)
        except Exception as e:
            print(f"¡ERROR! No pude guardar en CSV: {e}")

    def _enviar_alerta_email(self, temp, corr, status_flag):
        """Formatea y envía el correo de alerta HTML."""

        ahora = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        temp_str = f"{temp:.1f}°C"
        corr_str = f"{corr:.2f} A"

        if status_flag == 1:
            motivo_texto = "ALERTA ALTA (T>60 o I_sim>2)"
            asunto = f"ALERTA CRÍTICA: {motivo_texto}"
            titulo_html = "ALERTA DE ALTA TEMPERATURA / CORRIENTE"
            color_alerta = "#D9534F"  # Rojo
        else:
            motivo_texto = f"ALERTA BAJA (T<{LIMITE_TEMP_BAJA})"
            asunto = f"ALERTA CRÍTICA: {motivo_texto}"
            titulo_html = "ALERTA DE BAJA TEMPERATURA"
            color_alerta = "#5BC0DE"  # Azul

        print(f"\n¡¡¡ ENVIANDO ALERTA POR CORREO ({motivo_texto}) !!!")

        texto_plano = f"""
        ATENCIÓN EQUIPO DE MANTENIMIENTO
        Se ha detectado una condición de alerta en el sistema ECO-SENSE 1.0.
        El robot ha detenido su ciclo automáticamente y requiere revisión.
        DETALLES DE LA ALERTA:
        - Motivo: {titulo_html} - Fecha y Hora: {ahora}
        - Temperatura Detectada: {temp_str}
        - Corriente (Simulada): {corr_str}
        """

        html_body = HTML_EMAIL_TEMPLATE.format(
            asunto=asunto,
            color_alerta=color_alerta,
            titulo_html=titulo_html,
            ahora=ahora,
            temp_str=temp_str,
            corr_str=corr_str
        )

        msg = EmailMessage()
        msg['Subject'] = asunto
        msg['From'] = self.mail_usuario
        msg['To'] = self.mail_destino
        msg.set_content(texto_plano)
        msg.add_alternative(html_body, subtype='html')

        try:
            context = ssl.create_default_context()
            with smtplib.SMTP_SSL("smtp.gmail.com", 465, context=context) as smtp:
                smtp.login(self.mail_usuario, self.mail_pass_app)
                smtp.send_message(msg)
            print("¡Correo de alerta enviado exitosamente!")
        except Exception as e:
            print(f"¡¡¡ ERROR !!! No se pudo enviar el correo de alerta: {e}")

    def _analizar_datos_acumulados(self):
        """Usa Pandas para calcular y mostrar los promedios acumulados."""

        if not self.lista_lecturas_global:
            print("No hay datos para analizar.")
            return

        print("\n--- Procesando datos ACUMULADOS (TOTALES) con Pandas y NumPy ---")
        df_acumulado = pd.DataFrame(self.lista_lecturas_global)

        print("\n--- INDICADOR DE PROMEDIO (ACUMULADOS TOTALES) ---")
        promedios_por_posicion = df_acumulado.groupby('posicion_label')[
            ['temperatura', 'humedad', 'voltaje_real', 'corriente_sim', 'piezas_contadas']
        ].mean()
        print(promedios_por_posicion.to_string())

        # Cálculo de kWh
        promedio_corr_general = df_acumulado['corriente_sim'].mean()
        potencia_promedio_watts = VOLTAJE_LINEA * promedio_corr_general
        tiempo_total_segundos = time.time() - self.tiempo_inicio_script
        energia_kwh = (potencia_promedio_watts * tiempo_total_segundos) / 3600000.0
        total_piezas_acumuladas = df_acumulado['piezas_contadas'].sum()

        print(f"\n   Consumo Energético (Total Acumulado): {energia_kwh:.6f} kWh")
        print(f"   Total Piezas (Total Acumulado): {total_piezas_acumuladas} pzs")
        print("-" * 40)

    def connect(self):
        """Conecta al robot, lo habilita y prepara el CSV."""
        print(f"--- Conectando al robot en {self.ip_robot} ---")
        self.arm = XArmAPI(self.ip_robot)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0) # Corregido (antes 'state=0')
        self.arm.reset(wait=True)
        print("--- Robot conectado e inicializado ---")

        if not self._preparar_csv():
            self.arm.disconnect()
            print("¡ERROR! No se pudo crear el CSV. Abortando.")
            sys.exit()

    def disconnect(self):
        """Resetea y desconecta el robot de forma segura."""
        print("--- RUTINA FINALIZADA ---")
        print("Volviendo a posición inicial (reset).")
        self.arm.reset(wait=True)
        print("Desconectando del robot.")
        self.arm.disconnect()

    def run_main_loop(self):
        """El bucle principal que ejecuta el patrón de 4 puntos."""
        while True:
            print(f"\n--- INICIANDO PATRÓN DE 4 PUNTOS ---")
            hubo_alerta = False

            for paso in CICLO_DE_MOVIMIENTO:

                label_para_csv = f"Lote {self.contador_lote_global}"

                print(f"\n[{label_para_csv}] Moviendo a posición de acercamiento...")
                self._mover(paso["coords_acercamiento"])
                print(f"[{label_para_csv}] Acercando sensor para medir...")
                self._mover(paso["coords_medicion"])
                print(f"[{label_para_csv}] MIDiendo... (esperando {PAUSA_MEDICION} segundos)")
                time.sleep(PAUSA_MEDICION)

                temp, hum, volt, corr, status_flag = self._leer_sensor_usb()
                piezas = 1 if (temp > 0 or hum > 0) else 0

                if piezas > 0:
                    print(
                        f"   ... T°={temp}°C, H={hum}%, V={volt:.2f}V, I(sim)={corr:.2f}A, P(cont)={piezas} (Status={status_flag})")
                    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                    self.lista_lecturas_global.append({
                        'posicion_label': label_para_csv,
                        'temperatura': temp, 'humedad': hum,
                        'voltaje_real': volt, 'corriente_sim': corr,
                        'piezas_contadas': piezas
                    })

                    self._guardar_en_csv([timestamp, label_para_csv, temp, hum, volt, corr, piezas])
                    self._analizar_datos_acumulados()

                else:
                    print(f"[{label_para_csv}] Lectura fallida. No se guardan datos para este lote.")

                if status_flag != 0:
                    self._enviar_alerta_email(temp, corr, status_flag)
                    hubo_alerta = True
                    break

                print(f"[{label_para_csv}] Medición completada. Retirando sensor.")
                self._mover(paso["coords_acercamiento"])

                # El contador avanza, falle o no la lectura
                self.contador_lote_global += 1

            if hubo_alerta:
                print("--- DETENIENDO LA RUTINA (Alerta procesada) ---")
                break  # Rompe el 'while True:'
            else:
                print("\n--- Patrón de 4 puntos terminado. Repitiendo... ---")


# --- 5. PUNTO DE ENTRADA DEL SCRIPT ---
def main():
    """Función principal para inicializar y ejecutar el robot."""
    print("--- INICIANDO SISTEMA ECO-SENSE 1.0 ---")
    robot = EcoSenseRobot()

    try:
        robot.connect()
        robot.run_main_loop()

    except KeyboardInterrupt:
        print("\nProceso interrumpido por el usuario (Ctrl+C).")
    except Exception as e:
        print(f"\n¡¡¡ ERROR FATAL NO CONTROLADO !!!: {e}")
    finally:
        if robot.arm:
            robot.disconnect()
        print("--- Sistema ECO-SENSE 1.0 desconectado. ---")


if __name__ == "__main__":
    main()
