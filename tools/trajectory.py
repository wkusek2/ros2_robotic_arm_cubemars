#!/usr/bin/env python3
"""
Trapezoidalny profil predkosci dla jednego silnika.
Timer ROS2 zamiast time.sleep() — brak jittera.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

KP = 50.0
KD = 10.0
TAU_FF = 1.0
DT = 1.0 / 50  # 400 Hz

MOTOR_ID = 1
POS_A    = 0.0
POS_B    = 180.0
MAX_VEL  = 80.0
ACCEL    = 40.0


def trapezoidal(pos_start, pos_end, max_vel, accel, dt):
    dist = pos_end - pos_start
    direction = 1.0 if dist >= 0 else -1.0
    dist_abs = abs(dist)

    t_accel = max_vel / accel
    d_accel = 0.5 * accel * t_accel ** 2

    if 2 * d_accel >= dist_abs:
        t_accel = math.sqrt(dist_abs / accel)
        max_vel_real = accel * t_accel
        t_const = 0.0
    else:
        max_vel_real = max_vel
        t_const = (dist_abs - 2 * d_accel) / max_vel

    t_total = 2 * t_accel + t_const
    t = 0.0

    while t <= t_total:
        if t < t_accel:
            vel = accel * t
            pos = 0.5 * accel * t ** 2
        elif t < t_accel + t_const:
            vel = max_vel_real
            pos = d_accel + max_vel_real * (t - t_accel)
        else:
            t_brake = t - t_accel - t_const
            vel = max_vel_real - accel * t_brake
            pos = d_accel + max_vel_real * t_const + max_vel_real * t_brake - 0.5 * accel * t_brake ** 2

        yield pos_start + direction * pos, direction * vel
        t += dt

    yield pos_end, 0.0


def main():
    rclpy.init()
    node = Node("trajectory_node")
    pub = node.create_publisher(Float64MultiArray, "/arm6dof/arm/mit_hw_cmd", 10)

    print(f"Silnik {MOTOR_ID}: oscylacja {POS_A} <-> {POS_B} deg  Ctrl+C aby zatrzymac")

    direction = [1]
    gen = [trapezoidal(POS_A, POS_B, MAX_VEL, ACCEL, DT)]

    def tick():
        try:
            pos, vel = next(gen[0])
        except StopIteration:
            direction[0] *= -1
            pos_s = POS_A if direction[0] == 1 else POS_B
            pos_e = POS_B if direction[0] == 1 else POS_A
            gen[0] = trapezoidal(pos_s, pos_e, MAX_VEL, ACCEL, DT)
            pos, vel = next(gen[0])

        msg = Float64MultiArray()
        msg.data = [float(MOTOR_ID), math.radians(pos), math.radians(vel), KP, KD, TAU_FF]
        pub.publish(msg)
        msg.data = [float(3), math.radians(pos), math.radians(vel), KP, KD, TAU_FF]
        pub.publish(msg)

    node.create_timer(DT, tick)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
