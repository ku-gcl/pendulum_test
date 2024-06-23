import pigpio
import time

class RotaryEncoder:
    def __init__(self, gpio_a, gpio_b, pulses_per_revolution, threshold_on=1, threshold_off=0):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            exit()

        self.gpio_a = gpio_a
        self.gpio_b = gpio_b
        self.pulses_per_revolution = pulses_per_revolution
        self.threshold_on = threshold_on
        self.threshold_off = threshold_off
        self.state_a = 0
        self.state_b = 0
        self.edge_a = 0
        self.edge_b = 0
        self.position = 0

        self.pi.set_mode(gpio_a, pigpio.INPUT)
        self.pi.set_mode(gpio_b, pigpio.INPUT)

        self.pi.set_pull_up_down(gpio_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpio_b, pigpio.PUD_UP)

    def update(self):
        val_a = self.pi.read(self.gpio_a)
        val_b = self.pi.read(self.gpio_b)

        self.edge_a = 0
        if (val_a > self.threshold_on) and (self.state_a == 0):
            self.state_a = 1
            self.edge_a = 1  # Rising edge
        elif (val_a < self.threshold_off) and (self.state_a == 1):
            self.state_a = 0
            self.edge_a = -1  # Falling edge

        self.edge_b = 0
        if (val_b > self.threshold_on) and (self.state_b == 0):
            self.state_b = 1
            self.edge_b = 1  # Rising edge
        elif (val_b < self.threshold_off) and (self.state_b == 1):
            self.state_b = 0
            self.edge_b = -1  # Falling edge

        if self.edge_a == 1:  # A rising
            if self.state_b:
                self.position -= 1
            else:
                self.position += 1
        elif self.edge_a == -1:  # A falling
            if self.state_b:
                self.position += 1
            else:
                self.position -= 1

        if self.edge_b == 1:  # B rising
            if self.state_a:
                self.position += 1
            else:
                self.position -= 1
        elif self.edge_b == -1:  # B falling
            if self.state_a:
                self.position -= 1
            else:
                self.position += 1

    def get_position(self):
        return self.position

    def get_angle(self):
        return self.position * 360 / (4 * self.pulses_per_revolution)

    def reset(self):
        self.position = 0

    def stop(self):
        self.pi.stop()

if __name__ == "__main__":
    PULSES_PER_REVOLUTION = 100  # EC202A100aの1回転あたりのパルス数
    encoder = RotaryEncoder(24, 23, PULSES_PER_REVOLUTION)
    
    try:
        while True:
            encoder.update()
            position = encoder.get_position()
            angle = encoder.get_angle()
            print(f"Position: {position}, Angle: {angle:.2f} degrees")
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        encoder.stop()







# import pigpio
# import time

# class RotaryEncoder:
#     def __init__(self, gpio_a, gpio_b, pulses_per_revolution):
#         self.pi = pigpio.pi()
#         if not self.pi.connected:
#             exit()

#         self.gpio_a = gpio_a
#         self.gpio_b = gpio_b
#         self.position = 0
#         self.pulses_per_revolution = pulses_per_revolution

#         self.last_state = (self.pi.read(gpio_a), self.pi.read(gpio_b))

#         self.pi.set_mode(gpio_a, pigpio.INPUT)
#         self.pi.set_mode(gpio_b, pigpio.INPUT)

#         self.pi.set_pull_up_down(gpio_a, pigpio.PUD_UP)
#         self.pi.set_pull_up_down(gpio_b, pigpio.PUD_UP)

#     def update(self):
#         current_state = (self.pi.read(self.gpio_a), self.pi.read(self.gpio_b))
#         if current_state != self.last_state:
#             if self.last_state == (0, 0):
#                 if current_state == (0, 1):
#                     self.position += 1
#                 elif current_state == (1, 0):
#                     self.position -= 1
#             elif self.last_state == (0, 1):
#                 if current_state == (1, 1):
#                     self.position += 1
#                 elif current_state == (0, 0):
#                     self.position -= 1
#             elif self.last_state == (1, 1):
#                 if current_state == (1, 0):
#                     self.position += 1
#                 elif current_state == (0, 1):
#                     self.position -= 1
#             elif self.last_state == (1, 0):
#                 if current_state == (0, 0):
#                     self.position += 1
#                 elif current_state == (1, 1):
#                     self.position -= 1
#             self.last_state = current_state

#     def get_position(self):
#         return self.position

#     def get_angle(self):
#         # return (self.position % self.pulses_per_revolution) * (360 / self.pulses_per_revolution)
#         return self.position * 360 / (4 * self.pulses_per_revolution)

#     def reset(self):
#         self.position = 0

#     def stop(self):
#         self.pi.stop()

# if __name__ == "__main__":
#     PULSES_PER_REVOLUTION = 100  # EC202A100aの1回転あたりのパルス数
#     encoder = RotaryEncoder(24, 23, PULSES_PER_REVOLUTION)
    
#     try:
#         while True:
#             encoder.update()
#             position = encoder.get_position()
#             angle = encoder.get_angle()
#             print(f"Position: {position}, Angle: {angle:.2f} degrees")
#             time.sleep(0.001)
#     except KeyboardInterrupt:
#         print("Stopping...")
#     finally:
#         encoder.stop()
