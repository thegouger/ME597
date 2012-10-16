import roslib; roslib.load_manifest('clearpath_horizon')
import rospy

from clearpath_horizon.msg import *



class DataReceiver:
    def __init__(self, horizon):
        self.horizon = horizon
        self.publishers = {}
        horizon.add_handler(self.receive)
        for topic, frequency in rospy.get_param('~data', {}).items():
            self.subscribe(topic, frequency)


    @staticmethod
    def _receive_data_system_status(payload, timestamp):
        msg = SystemStatus()
        msg.uptime = payload.uptime
        msg.voltages = payload.voltages
        msg.currents = payload.currents
        msg.temperatures = payload.temperatures
        return msg

    def _subscribe_data_system_status(self):
        pub = rospy.Publisher('data/system_status', SystemStatus, latch=True)
        return pub, self.horizon.request_system_status



    @staticmethod
    def _receive_data_power_status(payload, timestamp):
        msg = PowerStatus()
        charges = payload.charges
        capacities = payload.capacities
        descriptions = payload.descriptions
        for i in range(len(charges)):
            msg_s = PowerSource(charge=charges[i], 
                                capacity=capacities[i])
            msg_s.present, msg_s.in_use, msg_s.description = descriptions[i]
            msg.sources.append(msg_s)
        return msg

    def _subscribe_data_power_status(self):
        pub = rospy.Publisher('data/power_status', PowerStatus, latch=True)
        return pub, self.horizon.request_power_status
 

    @staticmethod
    def _receive_data_processor_status(payload, timestamp):
        msg = ProcessorStatus()
        msg.errors = payload.errors
        return msg

    def _subscribe_data_processor_status(self):
        pub = rospy.Publisher('data/processor_status', ProcessorStatus, latch=True)
        return pub, self.horizon.request_processor_status


    @staticmethod
    def _receive_data_differential_speed(payload, timestamp):
        msg = DifferentialSpeed()
        msg.left_speed = payload.left_speed
        msg.right_speed = payload.right_speed
        msg.left_accel = payload.left_accel
        msg.right_accel = payload.right_accel
        return msg

    def _subscribe_data_differential_speed(self):
        pub = rospy.Publisher('data/differential_speed', DifferentialSpeed, latch=True)
        return pub, self.horizon.request_differential_speed


    @staticmethod
    def _receive_data_differential_control(payload, timestamp):
        msg = DifferentialControl()
        msg.left_p = payload.left_proportion
        msg.left_i = payload.left_integral
        msg.left_d = payload.left_derivative
        msg.left_ff = payload.left_feed_forward
        msg.left_sc = payload.left_stiction
        msg.left_il = payload.left_limit
        msg.right_p = payload.right_proportion
        msg.right_i = payload.right_integral
        msg.right_d = payload.right_derivative
        msg.right_ff = payload.right_feed_forward
        msg.right_sc = payload.right_stiction
        msg.right_il = payload.right_limit
	msg.left_p = payload.left_proportion
	msg.left_i = payload.left_integral
	msg.left_d = payload.left_derivative
	msg.left_ff = payload.left_feed_forward
	msg.left_sc = payload.left_stiction
	msg.left_il = payload.left_limit
        msg.right_p = payload.right_proportion
        msg.right_i = payload.right_integral
        msg.right_d = payload.right_derivative
        msg.right_ff = payload.right_feed_forward
        msg.right_sc = payload.right_stiction
        msg.right_il = payload.right_limit

        return msg

    def _subscribe_data_differential_control(self):
        pub = rospy.Publisher('data/differential_control', DifferentialControl, latch=True)
        return pub, self.horizon.request_differential_control


    @staticmethod
    def _receive_data_differential_output(payload, timestamp):
        msg = DifferentialOutput()
        msg.left = payload.left
        msg.right = payload.right
        return msg

    def _subscribe_data_differential_output(self):
        pub = rospy.Publisher('data/differential_output', DifferentialOutput, latch=True)
        return pub, self.horizon.request_differential_output


    # Ackermann setpoints
    @staticmethod
    def _receive_data_ackermann_output(payload, timestamp):
        msg = AckermannOutput()
        msg.steering = payload.steering
        msg.throttle = payload.throttle
        msg.brake = payload.brake
        return msg

    def _subscribe_data_ackermann_output(self):
        pub = rospy.Publisher('data/ackermann_output', AckermannOutput, latch=True)
        return pub, self.horizon.request_ackermann_output


    @staticmethod
    def _receive_data_velocity(payload, timestamp):
        msg = VelocitySetpt()
        msg.trans_vel = payload.trans
        msg.rot_vel = payload.rot
        msg.trans_accel = payload.accel
        return msg

    def _subscribe_data_velocity(self):
        pub = rospy.Publisher('data/velocity', VelocitySetpt, latch=True)
        return pub, self.horizon.request_velocity

    @staticmethod
    def _receive_data_platform_orientation(payload, timestamp):
        msg = Orientation()
        msg.roll = payload.get_roll()
        msg.pitch = payload.get_pitch()
        msg.yaw = payload.get_yaw()
        return msg

    def _subscribe_data_platform_orientation(self):
        pub = rospy.Publisher('data/platform_orientation', Orientation, latch=True)
        return pub, self.horizon.request_platform_orientation

    @staticmethod
    def _receive_data_platform_rotation(payload, timestamp):
        msg = RotateRate()
        msg.roll = payload.rotational_roll
        msg.pitch = payload.rotational_pitch
        msg.yaw = payload.rotational_yaw
        return msg

    def _subscribe_data_platform_rotation(self):
        pub = rospy.Publisher('data/platform_rotation', RotateRate, latch=True)
        return pub, self.horizon.request_platform_rotation


    @staticmethod
    def _receive_data_encoders(payload, timestamp):
        msg = Encoders()
        travels = payload.get_travel()
        speeds = payload.get_speed()
        for i in range(len(travels)):
            msg.encoders.append(Encoder(travel=travels[i],
                                        speed=speeds[i]))
        return msg

    def _subscribe_data_encoders(self):
        pub = rospy.Publisher('data/encoders', Encoders, latch=True)
        return pub, self.horizon.request_encoders

    @staticmethod
    def _receive_data_processor_status(payload, timestamp):
        msg = ProcessorStatus()
        msg.errors = payload.errors
        return msg

    def _subscribe_data_processor_status(self):
        pub = rospy.Publisher('data/processor_status', ProcessorStatus, latch=True)
        return pub, self.horizon.request_processor_status

    @staticmethod
    def _receive_data_raw_encoders(payload, timestamp):
        msg = RawEncoders()
        msg.ticks = payload.get_ticks()
        return msg

    def _subscribe_data_raw_encoders(self):
        pub = rospy.Publisher('data/raw_encoders', RawEncoders, latch=True)
        return pub, self.horizon.request_raw_encoders


    @staticmethod
    def _receive_data_safety_status(payload, timestamp):
        msg = SafetyStatus()
        msg.flags = payload.flags
        msg.estop = payload.flags & 0x8;
        return msg

    def _subscribe_data_safety_status(self):
        pub = rospy.Publisher('data/safety_status', SafetyStatus, latch=True)
        return pub, self.horizon.request_safety_status


    @staticmethod
    def _receive_data_distance(payload, timestamp):
        msg = Distance()
        msg.distances = payload.distances
        return msg

    def _subscribe_data_distance(self):
        pub = rospy.Publisher('data/distance', Distance, latch=True)
        return pub, self.horizon.request_distance



    def subscribe(self, name, frequency):
        name = 'data_' + name
        self.publishers[name], do_subscribe = getattr(self, '_subscribe_' + name)()
        do_subscribe(frequency)
        
    def receive(self, name, payload, timestamp):
        msg = getattr(self, '_receive_' + name)(payload, timestamp)
        msg.header.stamp = rospy.Time.now()
        name in self.publishers and self.publishers[name].publish(msg)

