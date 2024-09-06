from abb_robot_client.egm import EGM
import abb_motion_program_exec as abb
import time
import numpy as np
import copy
import socket
import json
import pygame
import sys


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    return [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]


def apply_deadzone(value, threshold):
    if abs(value) < threshold:
        return 0
    return value


def egm_pose_target(gain=1):
    # config de limites de correccion
    mm = abb.egm_minmax(-1e-3, 1e-3)

    # config de los marcos de referencia
    corr_frame = abb.pose([0, 0, 0], [1, 0, 0, 0])
    corr_fr_type = abb.egmframetype.EGM_FRAME_WOBJ
    sense_frame = abb.pose([0, 0, 0], [1, 0, 0, 0])
    sense_fr_type = abb.egmframetype.EGM_FRAME_WOBJ
    egm_offset = abb.pose([0, 0, 0], [1, 0, 0, 0])

    # config EGM para orientación en el espacio de trabajo
    egm_config = abb.EGMPoseTargetConfig(corr_frame, corr_fr_type, sense_frame, sense_fr_type,
                                         mm, mm, mm, mm, mm, mm, 1000, 1000
                                         )
    # establece mov definido
    r1 = abb.robtarget([400, 100, 600], [0.7071068, 0., 0.7071068, 0.], abb.confdata(0, 0, 0, 1), [0] * 6)
    # r3 = abb.robtarget([500, 50, 600], [0.7071068, 0., 0.7071068, 0.], abb.confdata(0, 0, 0, 1), [0] * 6)

    # creación del programa de mov
    mp = abb.MotionProgram(egm_config=egm_config)
    mp.MoveJ(r1, abb.v1000, abb.fine)
    # mp.MoveJ(r3, abb.v1000, abb.fine)
    mp.EGMRunPose(10, 0.05, 0.05, egm_offset)

    # envio de datos al robot
    client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")
    lognum = client.execute_motion_program(mp, wait=False)

    # conexion con mando
    xinput_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    xinput_socket.connect(('localhost', 65433))
    buffer = ""

    # recepción y envio de correcciones en bucle
    t1 = time.perf_counter()

    r2 = copy.copy(r1)
    # r4 = copy.copy(r3)

    egm = EGM()
    t2 = t1
    while True:
        t2 = time.perf_counter()
        res, feedback = egm.receive_from_robot(timeout=0.05)
        if res:
            data = xinput_socket.recv(4096).decode('utf-8')
            buffer += data
            if '\n' in buffer:
                message, buffer = buffer.split('\n', 1)
                state = json.loads(message)

                if state['buttons'][0] == 1:  # Botón X para disminuir ganancia
                    gain = max(0.5, gain - 0.5)
                    print(f"Gain disminuido a: {gain}")
                if state['buttons'][1] == 1:  # Botón Y para aumentar ganancia
                    gain += 0.5
                    print(f"Gain aumentado a: {gain}")

                x = apply_deadzone(state['x'], 0.5)
                y = apply_deadzone(state['y'], 0.5)
                z = apply_deadzone(state['z'], 0.5)

                r2.trans[0] += x * gain
                r2.trans[1] += y * gain
                r2.trans[2] += z * gain
                '''
                r2.rot[0] += state['r_thumb_x'] * gain  # Q1
                r2.rot[1] += state['r_thumb_y'] * gain  # Q2

                # Usar los botones PAD_LEFT y PAD_RIGHT para ajustar Q3
                if state['buttons'] & 0x0004:  # PAD_LEFT
                    r2.rot[2] -= gain * 0.01  # Q3 disminuye
                if state['buttons'] & 0x0008:  # PAD_RIGHT
                    r2.rot[2] += gain * 0.01  # Q3 aumenta

                r2.rot[0] = clamp(r2.rot[0], -1, 1)
                r2.rot[1] = clamp(r2.rot[1], -1, 1)
                r2.rot[2] = clamp(r2.rot[2], -1, 1)
                r2.rot[3] = clamp(r2.rot[3], -1, 1)

                r2.rot = normalize_quaternion(r2.rot)
                '''
                # Actualización de la orientación del robot
                q1 = clamp(r2.rot[0] + state['roll'] * 0.01, -0.7071068, 0.7071068)  # Q1
                q2 = clamp(r2.rot[1] + state['pitch'] * 0.01, -0.7071068, 0.7071068)  # Q2
                q3 = clamp(r2.rot[2] + state['yaw'] * 0.01, -0.7071068,
                           0.7071068)  # Q3

                # Recalcular el cuarto componente del cuaternión (Q4) para mantenerlo unitario
                #q4 = np.sqrt(1.0 - (q1 ** 2 + q2 ** 2 + q3 ** 2))

                # Asegurar que Q1^2 + Q2^2 + Q3^2 <= 1 para evitar valores inválidos
                if q1 ** 2 + q2 ** 2 + q3 ** 2 <= 1.0:
                    q4 = np.sqrt(1.0 - (q1 ** 2 + q2 ** 2 + q3 ** 2))
                else:
                    q4 = r2.rot[3]  # Mantener el valor anterior de Q4 si los otros valores son inválidos

                r2.rot[0] = q1
                r2.rot[1] = q2
                r2.rot[2] = q3
                r2.rot[3] = q4

                if state['buttons'][0] == 1 and state['buttons'][1] == 1:
                    print("Ambos botones presionados, saliendo...")
                    break

            egm.send_to_robot_cart(r2.trans, r2.rot)
            # r4.trans[1] = (t2 - t1) * 100.
            # egm.send_to_robot_cart(r4.trans, r4.rot)

    # detención de EGM y graficar datos botenidos
    client.stop_egm()

    while client.is_motion_program_running():
        time.sleep(0.05)

    log_results = client.read_motion_program_result_log(lognum)

    # log_results.data is a numpy array
    import matplotlib.pyplot as plt
    fig, ax1 = plt.subplots()
    lns1 = ax1.plot(log_results.data[:, 0], log_results.data[:, 2:])
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Joint angle (deg)")
    ax2 = ax1.twinx()
    lns2 = ax2.plot(log_results.data[:, 0], log_results.data[:, 1], '-k')
    ax2.set_ylabel("Command number")
    ax2.set_yticks(range(-1, int(max(log_results.data[:, 1])) + 1))
    ax1.legend(lns1 + lns2, log_results.column_headers[2:] + ["cmdnum"])
    ax1.set_title("Joint motion")
    plt.show()
    print("Operación terminada")


if __name__ == "__main__":
    egm_pose_target()