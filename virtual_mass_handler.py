#!/usr/bin/python3

from std_srvs.srv import SetBool
import tf2_ros
from geometry_msgs.msg import TransformStamped, WrenchStamped, TwistStamped, Twist
from visualization_msgs.msg import Marker
import rospy
import numpy as np
import copy
from horizon.rhc import taskInterface
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Path

from force_joystick import ForceJoystick
from joy_commands import JoyForce
from force_filtering import ButterworthFilter, ButterworthWrenches

class VirtualMassHandler:
    def __init__(self, kin_dyn, initial_solution, ti: taskInterface, wrench_topic, wrench_filtering_bool, input_mode='joystick'):

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.predicted_path_lstm = Path()

        # self.MPC_gain =  np.linspace(0, 1, 51)
        # self.LSTM_gain = np.linspace(1, 0, 51)

        self.MPC_gain = np.ones(51)
        self.LSTM_gain = np.zeros(51)

        self.kin_dyn = kin_dyn

        self.dt = ti.prb.getDt()
        self.ns = ti.prb.getNNodes()

        self.__base_yaw_control_flag = True

        # expose this outside
        self.m_virtual = np.array([80, 80, 0.4*80]) # 80 80 80 slow but good
        self.k_virtual = np.array([0, 0, 0])
        self.d_virtual = np.array([70, 70, 0.4*70]) # 70 70 70 slow but good

        # critical damping
        # 2 * np.sqrt(k_virtual[0] * m_virtual[0]
        # 2 * np.sqrt(k_virtual[1] * m_virtual[1]
        self.solution = initial_solution

        self.force_sensed = None
        self.wrench_topic = wrench_topic

        self.ee_name = 'wander_ft_sensor_link'
        self.__base_link = 'wander_base_link'

        self.virtual_mass_controller = self.__init_virtual_mass_controller()
        self.sys_dim = self.virtual_mass_controller.getDimension()

        # ee task
        self.ee_task_name = 'ee_force'
        self.ee_task = ti.getTask(self.ee_task_name)
        # print("solution", self.solution, "sys_dim", self.sys_dim, "ee_task", self.ee_task)

        # ## required for omnisteering
        # # floating base task
        # self.posture_cart_name = "posture_base"
        # self.posture_cart_task = ti.getTask(self.posture_cart_name)

        # kin fun of end effector
        self.ee_fk_pose_fun = kin_dyn.fk(self.ee_name)
        self.ee_fk_vel_fun = kin_dyn.frameVelocity(self.ee_name, ti.model.kd_frame)

        # get pose and linear+angular velocity
        self.ee_initial_pose = self.ee_fk_pose_fun(q=self.solution['q'][:, 0])
        self.ee_initial_vel = self.ee_fk_vel_fun(q=self.solution['q'][:, 0], qdot=self.solution['v'][:, 0])

        # get position and linear velocity
        self.ee_initial_pos = copy.copy(self.ee_initial_pose['ee_pos'][:self.sys_dim].full())
        self.ee_initial_vel_lin = copy.copy(self.ee_initial_vel['ee_vel_linear'][:self.sys_dim].full())
        self.ee_initial_vel_ang = copy.copy(self.ee_initial_vel['ee_vel_angular'][:self.sys_dim].full())

        if self.__base_yaw_control_flag:

            ## base task
            self.base_force_name = 'base_force'
            self.base_force_task = ti.getTask(self.base_force_name)

            # kin_dyn functions of base
            self.base_fk_pose_fun = kin_dyn.fk(self.__base_link)
            self.base_fk_vel_fun = kin_dyn.frameVelocity(self.__base_link, ti.model.kd_frame)

            # get yaw angle of base
            self.base_initial_rot = self.base_fk_pose_fun(q=self.solution['q'][:, 0])['ee_rot'] # matrix
            self.base_initial_yaw = Rotation.from_matrix(self.base_initial_rot).as_euler("xyz")[2] # yaw angle

            # get yaw velocity of base
            self.base_initial_yaw_vel = self.base_fk_vel_fun(q=self.solution['q'][:, 0], qdot=self.solution['v'][:, 0])['ee_vel_angular'].full()[2]

            # virtual mass controller initialized with x_ee-y_ee, yaw_base
            self.ee_xy_base_yaw = np.array([[self.ee_initial_pos[0, 0], self.ee_initial_pos[1, 0], self.base_initial_yaw]]).T
            self.ee_xy_base_yaw_vel = np.array([[self.ee_initial_vel_lin[0, 0], self.ee_initial_vel_lin[1, 0], self.base_initial_yaw_vel[0]]]).T

            # set initial pose
            self.virtual_mass_controller.setPositionReference(self.ee_xy_base_yaw)

            # set initial state
            self.ee_integrated = np.vstack([self.ee_xy_base_yaw, self.ee_xy_base_yaw_vel])

            # get reference of base task force
            self.base_ref = self.base_force_task.getValues()

        else:
            # set initial pose
            self.virtual_mass_controller.setPositionReference(self.ee_initial_pos)

            # set initial state
            self.ee_integrated = np.vstack([self.ee_initial_pos, self.ee_initial_vel_lin])

        # get reference of ee task force
        self.wrench_filtering_bool = wrench_filtering_bool
        # if (self.wrench_filtering_bool) : self.filtered_wrenches_publisher = rospy.Publisher('/filtered_wrenches', WrenchStamped, queue_size=10)
        self.filtered_wrenches_publisher = rospy.Publisher('/filtered_wrenches', WrenchStamped, queue_size=10)
        dt = 132 / 6601
        sampling_frequency = 1/dt
        cutoff_frequency = 1
        order = 2
        btype = 'low'
        analog = False
        
        self.wrench_filter = ButterworthWrenches(sampling_frequency, cutoff_frequency, order, btype, analog)
        self.ee_wrench = np.zeros(6)
        self.ee_ref = self.ee_task.getValues()
        self.ee_ref[3:7, :] = np.array([[0, 0, 0, 1]]).T

        self.ee_homing_posture = copy.copy(self.solution['q'][15:22, :])
        # ee z task
        # self.ee_z_task = ti.getTask('ee_z_force')

        # ===============================================

        self.input_mode = input_mode  # 'joystick' 'sensor'

        if self.input_mode == 'joystick':
            self.__init_joystick()
        elif self.input_mode == 'sensor':
            self.__init_subscribers()

        # compute initial wrench offset
        wrench_init_rate = rospy.Rate(500)
        for i in range(50):
            self.wrench_offset = self.ee_wrench
            wrench_init_rate.sleep()
            i += 1

        print(f'Wrench offset: {self.wrench_offset}')

        self.__init_publisher()

    def __init_publisher(self):

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.transform_ref = TransformStamped()

        self.marker_pub = rospy.Publisher('force_marker', Marker, queue_size=10)

        self.marker_ref = Marker()
        self.marker_ref.header.frame_id = "world"  # Set your desired frame ID
        self.marker_ref.id = 0
        self.marker_ref.type = Marker.ARROW
        self.marker_ref.action = Marker.ADD

        # Set the scale of the marker
        self.marker_ref.scale.x = 0.2
        self.marker_ref.scale.y = 0.02
        self.marker_ref.scale.z = 0.02

        # Set the color of the marker
        self.marker_ref.color.r = 1.0
        self.marker_ref.color.g = 0.0
        self.marker_ref.color.b = 0.0
        self.marker_ref.color.a = 1.0  # Fully opaque


    def __init_joystick(self):
        self.jc = JoyForce()

    def __init_subscribers(self):
        print('Subscribing to force estimation topic...')
        # rospy.Subscriber('/force_estimation/local', WrenchStamped, self.__wrench_callback)  # /cartesian/force_estimation/ee_E
        rospy.Subscriber(self.wrench_topic, WrenchStamped, self.__wrench_callback)
        rospy.Subscriber('/wander/predicted_trajectory_LSTM', Path, self.__LSTM_path_callback)
        print("done.")

    def __init_virtual_mass_controller(self):

        sys_dim = 3

        vmass_opt = dict(mass=self.m_virtual, damp=self.d_virtual, spring=self.k_virtual)
        return ForceJoystick(dt=self.dt, n_step=self.ns, sys_dim=sys_dim, opt=vmass_opt)

    def __LSTM_path_callback(self, msg):

        self.predicted_path_lstm = msg

    def __wrench_callback(self, msg):

        # self.ee_wrench = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
        #                             msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])

        # if self.wrench_filtering_bool == False : 
        self.ee_wrench = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        # elif self.wrench_filtering_bool == True :
        self.ee_wrench_filtered = self.wrench_filter.update(np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]))
        
        filtered_wrenches_msg = WrenchStamped()

        filtered_wrenches_msg.wrench.force.x = self.ee_wrench_filtered[0]
        filtered_wrenches_msg.wrench.force.y = self.ee_wrench_filtered[1]
        filtered_wrenches_msg.wrench.force.z = self.ee_wrench_filtered[2]
        filtered_wrenches_msg.wrench.torque.x = self.ee_wrench_filtered[3]
        filtered_wrenches_msg.wrench.torque.y = self.ee_wrench_filtered[4] 
        filtered_wrenches_msg.wrench.torque.z = self.ee_wrench_filtered[5] 

        self.filtered_wrenches_publisher.publish(filtered_wrenches_msg)
        

    def __integrate(self, q_current, qdot_current, ee_wrench_sensed, wrench_local_frame=False):

        # get wrench
        force_sensed = copy.copy(ee_wrench_sensed[:3])
        # force_sensed = ee_wrench_sensed[1:4]
        # print('force sensed', force_sensed)

        # get current position of the ee on xy
        ee_pose = self.ee_fk_pose_fun(q=q_current)
        ee_vel = self.ee_fk_vel_fun(q=q_current, qdot=qdot_current)

        ee_pos = ee_pose['ee_pos'][:self.sys_dim].full()
        ee_vel_lin = ee_vel['ee_vel_linear'][:self.sys_dim]

        # controller works in world frame
        if self.input_mode == 'sensor' and wrench_local_frame:
            # rotate in world from local ee
            ee_rot = ee_pose['ee_rot']
            
            force_sensed_rot = (ee_rot @ force_sensed)[:self.sys_dim]
            # print('force sensed rotated', force_sensed_rot)
            
        else:
            # force in world coordinates from joystick
            force_sensed_rot = force_sensed

        # ignore z if follow me is on
        force_sensed_rot[2] = 0.0
        # force_sensed_rot[2] = copy.copy(force_sensed_rot[1])

        # compute virtual mass displacement
        # with integrated state
        # with real state
        if self.__base_yaw_control_flag:

            # get current yaw angle of the base
            base_pose = self.base_fk_pose_fun(q=q_current)
            base_vel = self.base_fk_vel_fun(q=q_current, qdot=qdot_current)

            print('base_pose: ', base_pose)
            print('base vel: ', base_vel)

            base_yaw = Rotation.from_matrix(base_pose['ee_rot']).as_euler("xyz")[2]
            base_yaw_vel = base_vel['ee_vel_angular'].full()[2]

            # cross product between force sensed (in world) and vector rotated as the base_link
            # force_sensed_rot[2] = np.cross(np.array(base_pose['ee_rot']) @ np.array([[1, 0, 0]]).T, force_sensed_rot.reshape((3, 1)), axis=0)[2]

            force_sensed_rot[2] = -ee_wrench_sensed[3] #NB the x-axis of the sensor is oriented opposite direction of the z-axis of the base
                                                        # Temporary solution with minus sign but the analytical transformation should be implemented
            # using xy of ee and yaw of base
            ee_x_base_yaw = np.zeros([3, 1])
            ee_x_base_yaw[0] = ee_pos[0][0]
            ee_x_base_yaw[1] = ee_pos[1][0]
            ee_x_base_yaw[2] = base_yaw

            ee_x_base_yaw_vel = np.zeros([3, 1])
            ee_x_base_yaw_vel[0] = ee_vel_lin[0][0]
            ee_x_base_yaw_vel[1] = ee_vel_lin[1][0]
            ee_x_base_yaw_vel[2] = base_yaw_vel

            # self.virtual_mass_controller.update(np.vstack([ee_x_base_yaw, ee_x_base_yaw_vel]), force_sensed_rot[:self.sys_dim]) # Com'era prima
            self.virtual_mass_controller.update(self.ee_integrated[:, 0], force_sensed_rot[:self.sys_dim]) # Con la modifica che mi hai mandato 

        else:
            # using xyz of ee
            self.virtual_mass_controller.update(np.vstack([ee_pos, ee_vel_lin]), force_sensed_rot[:self.sys_dim])

        # Da considerare come modifica per risolvere il problema del drift
        # if (self.__closed_integrated_state):
        #     self.virtual_mass_controller.update(self.ee_integrated[:, 0], force_sensed_rot[:self.sys_dim])
        # else:
        #     self.virtual_mass_controller.update(np.vstack([ee_pos, ee_vel_lin]), force_sensed[:self.sys_dim])

        self.ee_integrated = self.virtual_mass_controller.getIntegratedState()
        return self.ee_integrated
    
    def twist_transformation(self, twist_frame_A, reference_frame_A, reference_frame_B):

        """
        transformation of a twist vector from reference frame A (starting reference frame) to twist in reference frame B (ending RF)
        """ 
        
        transformed_twist = Twist()
        reference_frame_A_T_reference_frame_B = self.tfBuffer.lookup_transform(reference_frame_A, reference_frame_B, rospy.Time(), rospy.Duration(3.0))
        translation = ([reference_frame_A_T_reference_frame_B.transform.translation.x, reference_frame_A_T_reference_frame_B.transform.translation.y, reference_frame_A_T_reference_frame_B.transform.translation.z])
        rotation = Rotation.from_quat([reference_frame_A_T_reference_frame_B.transform.rotation.x, reference_frame_A_T_reference_frame_B.transform.rotation.y, 
                                reference_frame_A_T_reference_frame_B.transform.rotation.z, reference_frame_A_T_reference_frame_B.transform.rotation.w])
                
        rotation_matrix = Rotation.as_matrix(rotation)
        skew_matrix = np.cross(translation, rotation_matrix)

        transform_matrix = np.zeros((6,6))
        transform_matrix[0:3, 0:3] = transform_matrix[3:, 3:] = rotation_matrix
        transform_matrix[3:, 0:3] = skew_matrix
        vector6d = np.array([twist_frame_A.linear.x, twist_frame_A.linear.y, twist_frame_A.linear.z,
                    twist_frame_A.angular.x, twist_frame_A.angular.y, twist_frame_A.angular.z]).transpose()
        vector6d = np.reshape(vector6d, (6,1))
        vel_transformed = np.dot(transform_matrix,vector6d)
        # vel_transformed = np.matmul(transform_matrix,vector6d)
        transformed_twist.linear.x = vel_transformed[0]
        transformed_twist.linear.y = vel_transformed[1]
        transformed_twist.linear.z = vel_transformed[2]
        transformed_twist.angular.x = vel_transformed[3]
        transformed_twist.angular.y = vel_transformed[4]
        transformed_twist.angular.z = vel_transformed[5]
        
        # self.odometry_prev = odom_msg
        # self.time_prev = time_now
        return transformed_twist

    def publish_tf(self, ref):

        self.transform_ref.header.stamp = rospy.Time.now()
        self.transform_ref.header.frame_id = 'world'
        self.transform_ref.child_frame_id = 'force_ref'
        self.transform_ref.transform.translation.x = ref[0, 0]
        self.transform_ref.transform.translation.y = ref[1, 0]
        self.transform_ref.transform.translation.z = ref[2, 0]
        self.transform_ref.transform.rotation.x = ref[3, 0]
        self.transform_ref.transform.rotation.y = ref[4, 0]
        self.transform_ref.transform.rotation.z = ref[5, 0]
        self.transform_ref.transform.rotation.w = ref[6, 0]

        self.tf_broadcaster.sendTransform(self.transform_ref)

    def publish_marker(self, ref):

        self.marker_ref.header.stamp = rospy.Time.now()
        self.marker_ref.pose.position.x = ref[0, 0]
        self.marker_ref.pose.position.y = ref[1, 0]
        self.marker_ref.pose.position.z = ref[2, 0]
        self.marker_ref.pose.orientation.x = ref[3, 0]
        self.marker_ref.pose.orientation.y = ref[4, 0]
        self.marker_ref.pose.orientation.z = ref[5, 0]
        self.marker_ref.pose.orientation.w = ref[6, 0]

        self.marker_pub.publish(self.marker_ref)

    def getForceSensed(self):

        return self.force_sensed

    def getOutput(self):

        return self.ee_ref

    def run(self, solution):

        self.solution = solution

        # select input mode
        if self.input_mode == 'joystick':
            self.jc.run(self.solution)
            self.force_sensed = self.jc.getForce().T
        elif self.input_mode == 'sensor':
            self.force_sensed = self.ee_wrench # - self.wrench_offset

        else:
            raise Exception('Wrong input mode')


        # print('force sensed: ', self.force_sensed)

        # get reference
        self.__integrate(self.solution['q'][:, 0],
                         self.solution['v'][:, 0],
                         self.force_sensed,
                         wrench_local_frame=True)
 
        self.ee_ref[:2, :] = self.ee_integrated[:2, :] # We set the reference for x,y --> self.ee_ref[:2, :] = (2, num_nodes_horizon)
        
        # --- Here we can add the inference of LSTM predicted path ---

        # ee_ref_LSTM = np.zeros((2, self.ee_ref[:2, :].shape[1]))  # Matrice 2xN

        # for i, pose in enumerate(self.predicted_path_lstm.poses):
        #     ee_ref_LSTM[0, i] = pose.pose.position.x  
        #     ee_ref_LSTM[1, i] = pose.pose.position.y  
        
        # self.ee_ref[0, :] = (self.MPC_gain * self.ee_ref[0, :] + self.LSTM_gain * ee_ref_LSTM[0, :]) / (self.MPC_gain + self.LSTM_gain)
        # self.ee_ref[1, :] = (self.MPC_gain * self.ee_ref[1, :] + self.LSTM_gain * ee_ref_LSTM[1, :]) / (self.MPC_gain + self.LSTM_gain)

        # We set the reference for the yaw
        if self.__base_yaw_control_flag:
            # using xyz of ee
            self.base_ref[3:7, :] = Rotation.from_euler('z', self.ee_integrated[2, :]).as_quat().T
        else:
            # using xy of ee and yaw of base
            self.ee_ref[:self.sys_dim, :] = self.ee_integrated[:self.sys_dim, :]

        self.ee_task.setRef(self.ee_ref)
        if self.__base_yaw_control_flag:
            self.base_force_task.setRef(self.base_ref)
