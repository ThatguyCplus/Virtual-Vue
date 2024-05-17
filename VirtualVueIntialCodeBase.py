import time
import board
import busio
import pwmio
from adafruit_motor import servo
from adafruit_bno055 import BNO055_I2C

# Initialize I2C for BNO055 sensor
i2c = busio.I2C(scl=board.GP21, sda=board.GP20)
bno = BNO055_I2C(i2c)

# Setup UART for SBUS communication
uart = busio.UART(board.GP0, board.GP1, baudrate=100000, stop=2)

# Initialize PWM outputs
pwm_servos = [pwmio.PWMOut(pin, frequency=333) for pin in (board.GP2, board.GP4, board.GP3)]
servos = [servo.Servo(pwm, min_pulse=500, max_pulse=2500) for pwm in pwm_servos]

# Pre-compute SBUS frame start byte and end bytes
SBUS_START = bytearray([0x0F])
SBUS_END = bytearray([0x00, 0x00])

def pwm_to_sbus_value(pwm_value):
    """ Map PWM values (0 to 180 degrees) to SBUS range (0 to 2047). """
    return int((pwm_value / 180) * 2047)

def create_sbus_frame(angles):
    """ Construct an SBUS frame from servo angles. """
    channel_data = [pwm_to_sbus_value(angle) for angle in angles] + [0] * (16 - len(angles))
    
    # Pack 11 bits per channel into 8-bit bytes
    packed_data = bytearray()
    bit_packed = 0
    bits_filled = 0
    for value in channel_data:
        bit_packed |= (value << bits_filled)
        bits_filled += 11
        
        while bits_filled >= 8:
            packed_data.append(bit_packed & 0xFF)
            bit_packed >>= 8
            bits_filled -= 8

    # Add any remaining bits
    if bits_filled > 0:
        packed_data.append(bit_packed)

    return packed_data

def send_sbus_frame(angles):
    """ Send the SBUS frame over UART. """
    frame = SBUS_START + create_sbus_frame(angles) + SBUS_END
    uart.write(frame)

def quaternion_to_servo(quat):
    """ Map quaternion components to servo angles. """
    w, x, y, z = quat
    norm = (w**2 + x**2 + y**2 + z**2)**0.5
    if norm == 0:
        print("Invalid quaternion received.")
        return None
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    
    # Calculate servo angles (adjust as needed based on your setup)
    yaw_angle = (x + 1) * 90
    pitch_angle = (y + 1) * 90
    roll_angle = (z + 1) * 90
    return [yaw_angle, pitch_angle, roll_angle]

# User-defined servo angle for distance >= 100 cm after 5 seconds
safe_servo_angles = [90, 90, 90]

# Main control loop
while True:
    # Read quaternion data and control servos
    if bno.quaternion:
        quat = bno.quaternion
        servo_angles = quaternion_to_servo(quat)
        if servo_angles:
            for servo, angle in zip(servos, servo_angles):
                servo.angle = max(0, min(180, angle))
            send_sbus_frame(servo_angles)

    time.sleep(0.014)  # SBUS frame rate 
