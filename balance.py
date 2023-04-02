from pybricks.geometry import Axis
from pybricks.hubs import InventorHub
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.tools import StopWatch, wait

# Initialize motors.
left = Motor(Port.C, Direction.COUNTERCLOCKWISE)
right = Motor(Port.D)
left.reset_angle(0)
right.reset_angle(0)

# Initialize hub and color sensor.
# You can also use the TechnicHub and/or the ColorDistanceSensor
# instead.
hub = InventorHub()
sensor = ColorSensor(Port.A)

# Initialize position buffer for speed calculation.
process_loop = 5
window = int(300 / process_loop)
buf = [0] * window
idx = 0
angle = 0
DRIVE_SPEED = 300
PAUSE = 5000

# Timer to generate reference position.
watch = StopWatch()

while True:
    # Get angular rate and estimated angle.
    rate = hub.imu.angular_velocity(Axis.Y)
    angle += rate * process_loop / 1000

    # Get motor position.
    position = (left.angle() + right.angle()) / 2

    # Calculate motor speed.
    speed = (position - buf[idx]) / (window * process_loop) * 1000
    buf[idx] = position
    idx = (idx + 1) % window

    # Calculate reference position, which just grows linearly with
    # time.
    reference = -max(watch.time() - PAUSE, 0) / 1000 * DRIVE_SPEED

    # Calculate duty cycle.
    diff = position - reference
    duty = 0.018 * rate + 19 * angle + 0.45 * diff + 0.16 * speed

    # Account for battery level and type.
    duty *= 7200 / hub.battery.voltage()

    # Calculate steering.
    reflection = sensor.reflection()
    steering = (reflection - 28) * 0.6

    # Apply duty cycle for balancing and steering.
    left.dc(duty + steering)
    right.dc(duty - steering)

    # Wait some time.
    wait(process_loop)