
/** @file uav.h
 * The member functions of class 'UAV' allow to control, 
 * using an \b off-board computer, one or two multirotors,
 * equipped with the \b PX4 autopilot (https://px4.io/), by
 * leveraging the various features of the Robot Operating
 * System ( \b ROS ) and the \b MAVROS package 
 * (http://wiki.ros.org/mavros).
 * 
 * \author Joao Pinto (joao.s.pinto@tecnico.ulisboa.pt)
 * \date 2021-09-22
 */


#pragma once

#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <quad_ctrl_mavros/utils.h>
#include <quad_ctrl_mavros/AssertReadiness.h>
#include <quad_ctrl_mavros/RequestDeparture.h>
#include <thread>
#include <atomic>

class Uav{

    private:

        // namespace of the vehicle
        std::string _ns;

        // User input threads
        std::thread t[2];
        
        /** Variable used to check whether the vehicle
         * is armed, if GCS is connected to the vehicle... 
         * 
         * Check mavros_msgs/State.msg
         */
        mavros_msgs::State _current_state;

        // Publishers and Subscribers

        ros::Subscriber _state_sub;
        ros::Subscriber _local_pos_sub;
        ros::Subscriber _local_vel_sub;

        ros::Publisher _att_target_pub;
        ros::Publisher _local_pos_pub;

        // Messages 

        mavros_msgs::AttitudeTarget msg;
        geometry_msgs::Vector3 msg_body_rate;
        geometry_msgs::Quaternion msg_quaternion;

        // Services

        ros::ServiceClient _set_mode_client;
        ros::ServiceClient _arming_client;

        ros::ServiceServer _ready_server;
        ros::ServiceClient _ready_client;

        ros::ServiceServer _departure_server;
        ros::ServiceClient _departure_client;

        // State variables

        tf2::Vector3 _pos;
        tf2::Vector3 _vel;
        tf2::Matrix3x3 _R;
        tf2::Quaternion _quaternion;
        tf2::Vector3 _ang_vel;

        // Physical parameters

        float _mass;
        Inertia _J;

        // Safety zone

        static SafetyZone _safe_zone;

        // Controller working frequency

        float _fs;

        // Requests from the user

        std::atomic<bool> request_hold_mode;
        std::atomic<bool> request_land_mode;

        // Multi vehicle variables

        static bool _multi;
        bool _ready;
        bool _sync;
        std::string _other_ns;
        
        // Subscribers' callbacks

        void get_cur_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void get_cur_vel(const geometry_msgs::TwistStamped::ConstPtr& msg);

        void state_cb(const mavros_msgs::State::ConstPtr& msg);

        // Ready Server callback

        bool ready_cb(quad_ctrl_mavros::AssertReadiness::Request &req,
                        quad_ctrl_mavros::AssertReadiness::Response &res);

        // Departure Server callback

        bool departure_cb(quad_ctrl_mavros::RequestDeparture::Request &req,
                            quad_ctrl_mavros::RequestDeparture::Response &res);

        // Launch publishers and subscribers

        void launch_pub_sub(ros::NodeHandle& nh);

        // Failsafe requests handler

        void failsafe();

        /** \brief User input handler.
         * 
         * During execution press:
         * 
         * \b 'l'+ENTER or \b 'L'+ENTER to request the vehicle to land immediately;
         * \b SPACEBAR+ENTER to request the vehicle to hover at the current position;
         */
        void user_input();

    public:

        Uav(ros::NodeHandle& nh, const std::string& ns, const float& fs);

        Uav(ros::NodeHandle& nh, const std::string& ns, 
                            const std::string& other_ns, const float& fs);

        Uav(ros::NodeHandle& nh, const std::string& ns, const float& mass, 
                                                const Inertia& J, const float& fs);

        Uav(ros::NodeHandle& nh, const std::string& ns, const std::string& other_ns, 
                                        const float& mass, const Inertia& J, const float& fs);

        ~Uav();

        /** \brief Arm vehicle and set OFFBOARD MODE.
         * \param throttle A Throttle reference.
         * \param ang_vel An AngularV reference.
         */
        void start_up(Throttle& throttle, AngularV& ang_vel);

        /** \brief Send throttle and angular velocity commands.
         * The vehicle must be in OFFBOARD mode to use this function.
         * Check MAVLink message SET_ATTITUTE_TARGET (#82) and
         * the definition of mavros_msgs/AttitudeTarget.msg for further details.
         * \param throttle Throttle input.
         * \param ang_vel Angular velocity input.
         */
        void send_cmd(const Throttle& throttle, const AngularV& ang_vel);

        /** \brief Send throttle and orientation commands.
         * The vehicle must be in OFFBOARD mode to use this function.
         * Check MAVLink message SET_ATTITUTE_TARGET (#82) and
         * the definition of mavros_msgs/AttitudeTarget.msg for further details.
         * \param throttle Throttle input.
         * \param R Rotation matrix input.
         */
        void send_cmd(const Throttle& throttle, const tf2::Matrix3x3& R);

        /** \brief Command vehicle to reach a position setpoint in a local frame.
         * The vehicle must be in OFFBOARD mode to use this function.
         * The main usages of this function are to command the vehicle to TAKE OFF
         * (AS SOON AS THE VEHICLE REACHES THE SETPOINT THE FUNCTION RETURNS) and
         * to keep the drone HOVERING at some position (FAILSAFE ACTIVATED WHEN THE
         * VEHICLE CROSSES THE LIMITS OF THE SAFE ZONE).
         * \param pos_local desired local position setpoint.
         */
        void go_to_pos(const tf2::Vector3& pos_local);

        /** \brief Verify if no failsafe requests have been issued and if
         * the vehicle has not crossed the boundaries of the safe zone. 
         * The vehicle should be AIRBORNE in order to use this function.
         * \returns 'True' if everthing is fine, 'False' whether one of the
         * conditions stated above is verified. 
         */
        const bool ok();
        
        /** \brief Define a safety zone for the vehicle.
         * This safe zone consists in a set of linear bounds on the local
         * position of the vehicle, in each of the three axes
         * (CHECK the definition of struct 'SafetyZone').
         * \param sz A SafetyZone reference.
         */
        static void set_safety_zone(const SafetyZone& sz);

        /** \brief Call this function to enable the exchange
         * of information between two vehicles flying simultaneously.
         */
        static void set_multi_vehicle_mode();

        /** \brief Check if the vehicle is out of the safety zone.
         */
        const bool isOutSafetyZone();

        /** \brief Check if a position setpoint is out of the safety zone.
         * \param pos position setpoint.
         */
        static const bool isOutSafetyZone(const tf2::Vector3& pos);

        /** \brief Check if the vehicle reached some position setpoint.
         * \param pos A tf2::Vector3 reference (position setpoint)
         */
        bool reached(const tf2::Vector3& pos);

        /** \brief Check if the off-board computer is connected to
         * the flying control unit (FCU).
         */
        const bool connected();

        /** \brief Check if the vehicle is armed.
         */
        const bool armed();

        /** \brief Check if the autopilot on-board the vehicle
         * is in a certain flight mode. The flight modes supported
         * by PX4 are available at http://wiki.ros.org/mavros/CustomModes
         */
        const bool isMode(const char* mode);

        /** \brief Verify if the motors of vehicle are engaged.
         */
        const bool isActive();

        /** \brief Command the vehicle to land.
         */
        void land();

        /** \brief Check if vehicle landed after taking off.
         */
        const bool landed();

        /** \brief Check if vehicle took off.
         */
        const bool took_off();

        /** \brief Call this function to ensure that two
         * vehicles start flying synchronously.
         */
        void vehicle_sync();

        // Access state variables

        const tf2::Vector3& position();

        const tf2::Matrix3x3& orientation();

        const tf2::Vector3& linear_vel();

        const tf2::Vector3& angular_vel();

};