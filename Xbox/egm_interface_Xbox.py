from abb_robot_client.egm import EGM
import abb_motion_program_exec as abb
import time
import numpy as np
import copy
import socket
import json
import pygame
import sys

v_global = 0
previous_state_x = False
previous_state_y = False


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    return [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]


def mix_target():
    global v_global

    while True:
        if v_global == 0:
            egm_pose_target()
        elif v_global ==1:
            egm_joint_target()
        elif v_global == 2:
            print("Finalización del programa por completo")
            break
        else:
            print("Valor no manejado, finalizando programa")
            break



def egm_joint_target(gain = 0.5):
    global v_global, previous_state_x, previous_state_y
    mm = abb.egm_minmax(-1e-3, 1e-3)

    egm_config = abb.EGMJointTargetConfig(
        mm, mm, mm, mm, mm, mm, 1000, 1000
    )

    joints = abb.jointtarget([0, 0, 0, 0, 0, 0], [0] * 6)

    mp = abb.MotionProgram(egm_config=egm_config)
    mp.MoveAbsJ(joints, abb.v5000, abb.fine)
    mp.EGMRunJoint(10, 0.05, 0.05)

    client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")
    lognum = client.execute_motion_program(mp, wait=False)

    xinput_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    xinput_socket.connect(('localhost', 5001))
    buffer = ""

    t1 = time.perf_counter()

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

                # Detectar el evento de pulsación del botón X (Disminuir gain)
                current_state_x = state['buttons'] & 0x0040
                if current_state_x and not previous_state_x:
                    gain = max(0.5, gain - 0.5)
                    print(f"Gain disminuido a: {gain}")
                previous_state_x = current_state_x

                # Detectar el evento de pulsación del botón Y (Aumentar gain)
                current_state_y = state['buttons'] & 0x0080
                if current_state_y and not previous_state_y:
                    gain += 0.5
                    print(f"Gain aumentado a: {gain}")
                previous_state_y = current_state_y

                # Actualización de las articulaciones según el mando
                joints.robax[0] += state['l_thumb_x'] * gain  # Articulación 1
                joints.robax[1] += state['l_thumb_y'] * gain  # Articulación 2
                joints.robax[2] += state['r_thumb_y'] * gain  # Articulación 3
                joints.robax[3] += state['r_thumb_x'] * gain  # Articulación 4

                if state['buttons'] & 0x0001:  # PAD_UP
                    joints.robax[4] += gain
                if state['buttons'] & 0x0002:  # PAD_DOWN
                    joints.robax[4] -= gain

                if state['buttons'] & 0x0100:  # LB
                    joints.robax[5] -= gain
                if state['buttons'] & 0x0200:  # RB
                    joints.robax[5] += gain

                # Aplicar restricciones a las articulaciones
                joints.robax[0] = clamp(joints.robax[0], -170, 170)  # Articulación 1
                joints.robax[1] = clamp(joints.robax[1], -70, 90)  # Articulación 2
                joints.robax[2] = clamp(joints.robax[2], -60, 40)  # Articulación 3
                joints.robax[3] = clamp(joints.robax[3], -90, 90)  # Articulación 4
                joints.robax[4] = clamp(joints.robax[4], -80, 90)  # Articulación 5
                joints.robax[5] = clamp(joints.robax[5], -180, 180)  # Articulación 6

                # Enviar los valores de las articulaciones al robot
                egm.send_to_robot(joints.robax)

                if state['buttons'] & 0x0010:  # Botón START para salir
                    v_global = 3
                    break

                if state['buttons'] & 0x0020:
                    v_global = 0
                    break

            # egm.send_to_robot(np.ones((6,)) * np.sin(t2 - t1) * 5)

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


def egm_pose_target(gain=10):
    global v_global, previous_state_x, previous_state_y
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
    xinput_socket.connect(('localhost', 5001))
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

                # Detectar el evento de pulsación del botón X (Disminuir gain)
                current_state_x = state['buttons'] & 0x0040
                if current_state_x and not previous_state_x:
                    gain = max(0.5, gain - 0.5)
                    print(f"Gain disminuido a: {gain}")
                previous_state_x = current_state_x

                # Detectar el evento de pulsación del botón Y (Aumentar gain)
                current_state_y = state['buttons'] & 0x0080
                if current_state_y and not previous_state_y:
                    gain += 0.5
                    print(f"Gain aumentado a: {gain}")
                previous_state_y = current_state_y

                # actualización del robot
                r2.trans[0] += state['l_thumb_x'] * gain
                r2.trans[1] += state['l_thumb_y'] * gain
                r2.trans[2] += (state['right_trigger'] - state['left_trigger']) * gain
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
                q1 = clamp(r2.rot[0] + state['r_thumb_x'] * 0.01, -0.7071068, 0.7071068)  # Q1
                q2 = clamp(r2.rot[1] + state['r_thumb_y'] * 0.01, -0.7071068, 0.7071068)  # Q2
                q3 = clamp(r2.rot[2] + (state['buttons'] & 0x0004 - state['buttons'] & 0x0008) * 0.01, -0.7071068,
                            0.7071068)  # Q3
                if state['buttons'] & 0x0004:  # PAD_LEFT
                    q3 = clamp(r2.rot[2] - 0.01 * gain, -0.7071068, 0.7071068)  # Disminuye Q3
                if state['buttons'] & 0x0008:  # PAD_RIGHT
                    q3 = clamp(r2.rot[2] + 0.01 * gain, -0.7071068, 0.7071068)  # Aumenta Q3

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

                if state['buttons'] & 0x0010:
                    v_global = 3
                    break

                if state['buttons'] & 0x0020:
                    v_global = 1
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
    mix_target()

