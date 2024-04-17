
/** @file uav.cpp
 * Definition of the class members declared in uav.h
 * 
 * \author Joao Pinto (joao.s.pinto@tecnico.ulisboa.pt)
 * \date 2021-09-22
 */

#include <quad_ctrl_mavros/uav.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

// If flying indoors, it is strongly recommended setting adequate bounds
// for the safety zone.
SafetyZone Uav::_safe_zone = SafetyZone(-1000.0f, 1000.0f, -1000.0f, 1000.0f, 0.0f, 1000.0f);

// Variable that defines whether two vehicles are flying at same time.
bool Uav::_multi = false;



void Uav::get_cur_pose(const geometry_msgs::PoseStamped::ConstPtr& msg){

    utils::fromMsg(msg->pose.position,_pos);
    utils::fromMsg(msg->pose.orientation,_quaternion);
}

void Uav::get_cur_vel(const geometry_msgs::TwistStamped::ConstPtr& msg){

    utils::fromMsg(msg->twist.linear,_vel);
    utils::fromMsg(msg->twist.angular, _ang_vel);
}

void Uav::state_cb(const mavros_msgs::State::ConstPtr& msg){

    _current_state = *msg;
}

bool Uav::departure_cb(quad_ctrl_mavros::RequestDeparture::Request &req,
                            quad_ctrl_mavros::RequestDeparture::Response &res){
    
    _sync = req.depart;
    return true;
}

bool Uav::ready_cb(quad_ctrl_mavros::AssertReadiness::Request &req,
                        quad_ctrl_mavros::AssertReadiness::Response &res){

    res.confirmation = _ready;
    return true;
}


void Uav::vehicle_sync(){

    static ros::Time last_request = ros::Time::now();
    std::string str = _ns + ": Waiting for %s";

    quad_ctrl_mavros::AssertReadiness srv;
    quad_ctrl_mavros::RequestDeparture srv2;

    if ( ok() ) {

        if ( ros::Time::now() - last_request > ros::Duration(2.0) ){

            if ( _ready_client.call(srv) ){

                if( srv.response.confirmation ){

                    srv2.request.depart = true;

                    if ( _departure_client.call(srv2) ) 
                        _sync = true;

                } else {

                    ROS_INFO(str.c_str(), _other_ns.c_str());
                }

               last_request = ros::Time::now();
            }

        }

    }

}

void Uav::send_cmd(const Throttle& throttle, const AngularV& ang_vel){

    utils::toMsg( ang_vel.get_value(), msg_body_rate );

    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    msg.body_rate = msg_body_rate;
    msg.thrust = throttle.get_value_norm();

    msg.header.stamp = ros::Time::now();

    _att_target_pub.publish(msg);
}

void Uav::send_cmd(const Throttle& throttle, const tf2::Matrix3x3& R){

    tf2::Quaternion q;
    R.getRotation(q);

    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE +
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE +
                            mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    utils::toMsg( q, msg_quaternion );

    msg.orientation = msg_quaternion;
    msg.thrust = throttle.get_value_norm();

    msg.header.stamp = ros::Time::now();

    _att_target_pub.publish(msg);

}

void Uav::go_to_pos(const tf2::Vector3& pos_local){

    ros::Rate loop_rate(_fs);

    geometry_msgs::PoseStamped wp;
    utils::toMsg(pos_local, wp.pose);

    std::string strA = _ns + ": Reached [%.3f,%.3f,%.3f]";
    std::string strB = _ns + ": Hovering at [%.3f,%.3f,%.3f]";
    std::string strC = _ns + ": Position Request failed";

    ros::Time last_print = ros::Time::now();

    static int cnt = 0;
    static int cnt2 = 0;

    while( ros::ok() ){

        if ( landed() || request_land_mode )
            break;

        if( isMode("OFFBOARD") && armed() ) {

            if ( took_off() && cnt < 1 ){
                
                std::string str3 = _ns + ": Taking off";
                ROS_INFO("%s",str3.c_str());
                ++cnt;

            }

            if ( reached(pos_local) ){

                if ( ok() ){

                    if ( cnt2 < 1 ) {

                        _ready = true;
                        ROS_INFO(strA.c_str(), position().x(), position().y(), position().z());
                        ++cnt2;
                    }

                    // Wait for the other vehicle!
                    if (_multi) {

                        vehicle_sync();

                        if ( _sync ){
                            std::string str4 = _ns + ": Sychronised!";
                            ROS_INFO("%s",str4.c_str());
                            break;
                        }

                    } else {

                        break;
                    }

                } else {

                    if( ros::Time::now() - last_print > ros::Duration(3.0) ){

                        ROS_INFO(strB.c_str(), position().x(), position().y(), position().z());
                        last_print = ros::Time::now();
                    }
                }
                
            }

        } else {

            if( ros::Time::now() - last_print > ros::Duration(3.0) ){

                ROS_INFO("%s",strC.c_str());
                last_print = ros::Time::now();
            }

        }

        ros::spinOnce();

        wp.header.stamp = ros::Time::now();
        _local_pos_pub.publish(wp);

        loop_rate.sleep();
    }

}

bool Uav::reached(const tf2::Vector3& pos){

    constexpr float DIFF_MAX = 0.07;
    constexpr float DIFF_MAX_Z = 0.03;
    constexpr float DIFF_VEL_MAX = 0.07;

    tf2::Vector3 diff = pos - position();

    bool pos_ok = ( ( std::abs(diff.x()) <= DIFF_MAX ) 
                        && ( std::abs(diff.y()) <= DIFF_MAX ) 
                            && ( std::abs(diff.z()) <= DIFF_MAX_Z ) ); 

    bool vel_ok = ( ( std::abs( linear_vel().x() ) <= DIFF_VEL_MAX )
                    && ( std::abs( linear_vel().y() ) <= DIFF_VEL_MAX )
                        && ( std::abs( linear_vel().z() ) <= DIFF_VEL_MAX ) );

    return ( pos_ok && vel_ok );
}

void Uav::user_input(){

    std::string strA, strB;
    strA = _ns + ": Requested hover command";
    strB = _ns + ": Requested land command";

    while( ros::ok() ){

        if( landed() ) break;

        switch ( getchar() ){

            // Detect if the space bar is pressed
            case 32:
                ROS_INFO("%s", strA.c_str());
                request_hold_mode = true;
                break;

            // Detect if the L key is pressed
            case 76:
                ROS_INFO("%s", strB.c_str());
                request_land_mode = true;
                break;

            // Detect if the L key is pressed
            case 108:
                ROS_INFO("%s", strB.c_str());
                request_land_mode = true;
                break;
            
            default:
                break;
        }

    }

}

void Uav::failsafe(){
    
    ros::Rate loop_rate(_fs);

    std::string strA = _ns + ": Request failed, not active";

    while( ros::ok() ){

        if( landed() )
            break;

        if ( isActive() ){

            if( request_hold_mode ){

                std::string str0 = _ns + ": Setting hover mode...";
                ROS_INFO("%s", str0.c_str());

                go_to_pos( position() );
                request_hold_mode = false;

            } else if( request_land_mode ) {

                land();
                request_land_mode = false;
            }

        } else {
            
            if ( request_hold_mode || request_land_mode ){
                
                ROS_INFO("%s", strA.c_str());
                request_hold_mode = false;
                request_land_mode = false;

            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

}

void Uav::start_up(Throttle& throttle, AngularV& ang_vel){

    // PX4 has a timeout of 500ms between two Offboard commands!
    ros::Rate loop_rate(_fs);

    throttle = 0.0f;
    ang_vel = tf2::Vector3(0.0f,0.0f,0.0f);

    utils::toMsg( ang_vel.get_value(), msg.body_rate );

    // wait for FCU connection
    while( ros::ok() && !connected() ){
        ros::spinOnce();
        loop_rate.sleep();
    }

    // send a few setpoints before starting
    for(int i = 300; ros::ok() && i > 0; --i){

        send_cmd(throttle,ang_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::string str0 = _ns + ": Starting Mission";
    ROS_INFO("%s",str0.c_str());

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while ( ros::ok() && ( !armed() || !isMode("OFFBOARD") ) ){

        // space out the service calls by 5 seconds 
        // to not flood the autopilot with the requests

        if( !isMode("OFFBOARD") &&
                (ros::Time::now() - last_request > ros::Duration(5.0)) ){

            if( _set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent ){
                    
                    std::string str1 = _ns + ": Offboard enabled";
                    ROS_INFO("%s",str1.c_str());

           }

           last_request = ros::Time::now();

        } else {

            if ( !armed() &&
                    (ros::Time::now() - last_request > ros::Duration(5.0)) ){

                if( _arming_client.call(arm_cmd) && arm_cmd.response.success ){

                    std::string str2 = _ns + ": Vehicle armed";
                    ROS_INFO("%s",str2.c_str());

                }

                last_request = ros::Time::now();

            }

        }

        // Necessary in order to set the OFFBOARD mode
        _att_target_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();

    }

}

void Uav::launch_pub_sub(ros::NodeHandle& nh){

    _state_sub = nh.subscribe<mavros_msgs::State>(_ns + "/mavros/state", 1, &Uav::state_cb, this);

    _local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(_ns + "/mavros/local_position/pose", 
                                                                            1, &Uav::get_cur_pose, this);

    _local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(_ns + "/mavros/local_position/velocity_local", 
                                                                                        1, &Uav::get_cur_vel, this);

    _att_target_pub = nh.advertise<mavros_msgs::AttitudeTarget>(_ns + "/mavros/setpoint_raw/attitude", 1);

    _local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(_ns + "/mavros/setpoint_position/local", 1);

    if ( _multi ){
        
        /** Here one can easily set publishers and subscribers that allow to
         * interact with the other vehicle (whose namespace is labelled '_other_ns').
         * 
         * For instance, to obtain pose information from the other vehicle,
         * set up this subscriber (you will need to create a suitable callback function),
         * 
         * sub = nh.subscribe<geometry_msgs::PoseStamped>(_other_ns + 
         *                     "/mavros/local_position/pose", 1, &Uav::get_other_pose, this);
         * 
         * To send position setpoints to other vehicle, set up this publisher,
         * 
         * pub = nh.advertise<geometry_msgs::PoseStamped>(_other_ns + 
         *                                          "/mavros/setpoint_position/local", 1);
         * 
         * 
         * Although the class member functions 'as-is' only allow to control two drones
         * simultaneously, one can extend these functions to control an arbitrary number of
         * vehicles. However, bear in mind that further code optimisation may be required
         * to ensure a reliable and efficient implementation of a given controller.
         */



        /** Set up the necessary servers to ensure both vehicles start tracking
         * some trajectory at the same time.
         */

        _ready_server = nh.advertiseService(_ns + "/ready", &Uav::ready_cb, this);
        _ready_client = nh.serviceClient<quad_ctrl_mavros::AssertReadiness>(_other_ns + "/ready");

        _departure_server = nh.advertiseService(_ns + "/departure", &Uav::departure_cb, this);
        _departure_client = nh.serviceClient<quad_ctrl_mavros::RequestDeparture>(_other_ns + "/departure");

        _ready = false;
        _sync = false;
    }

    _set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(_ns + "/mavros/set_mode");

    _arming_client = nh.serviceClient<mavros_msgs::CommandBool>(_ns + "/mavros/cmd/arming");

    // Threads for receiving and handling failsafe commands

    t[0] = std::thread(&Uav::user_input, this);
    t[1] = std::thread(&Uav::failsafe, this);

    request_hold_mode = false;
    request_land_mode = false;
}

Uav::Uav(ros::NodeHandle& nh, const std::string& ns, 
                        const float& fs) : _ns(ns), _fs(fs) {

    launch_pub_sub(nh);
}

Uav::Uav(ros::NodeHandle& nh, const std::string& ns, 
        const std::string& other_ns,  const float& fs) :
                             _ns(ns), _other_ns(other_ns), _fs(fs) {

    launch_pub_sub(nh);
}

Uav::Uav(ros::NodeHandle& nh, const std::string& ns, 
            const float& mass, const Inertia& J, const float& fs)
                                : _ns(ns), _mass(mass), _J(J), _fs(fs) {
    launch_pub_sub(nh);

}

Uav::Uav(ros::NodeHandle& nh, const std::string& ns, 
            const std::string& other_ns, const float& mass, 
                const Inertia& J, const float& fs): _ns(ns), _other_ns(other_ns), _mass(mass), _J(J), _fs(fs) {

    launch_pub_sub(nh);

}

Uav::~Uav(){
    
    t[0].join();
    t[1].join();
}

void Uav::set_safety_zone(const SafetyZone& sz){

    _safe_zone = sz;
}

void Uav::set_multi_vehicle_mode(){

    _multi = true;
}

const bool Uav::connected(){

    return _current_state.connected;
}

const bool Uav::armed(){

    return _current_state.armed;
}

const bool Uav::isMode(const char* mode){

    return (_current_state.mode == mode);
}

const bool Uav::isActive(){

    return (_current_state.system_status == 4);
}

const bool Uav::landed(){

    return (_current_state.system_status == 3 && !armed() && took_off());
}

void Uav::land(){

    ros::Rate loop_rate(_fs);

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    while( ros::ok() ){

        if( landed() ){
            std::string str7 = _ns + ": Landed";
            ROS_INFO("%s",str7.c_str());
            break;
        }

        if ( !land_set_mode.response.mode_sent ){

            if( _set_mode_client.call(land_set_mode) && 
                    land_set_mode.response.mode_sent ){

                std::string str5 = _ns + ": Landing";
                ROS_INFO("%s",str5.c_str());

            } else {

                std::string str2 = _ns + ": Landing command failed!";
                ROS_INFO("%s", str2.c_str());
            }

        }

        ros::spinOnce();
        loop_rate.sleep();

    }

}

const bool Uav::ok(){

    static bool fine = true;

    // Check if the FCU is within the bounds of the safety zone
    if( fine && isOutSafetyZone() ) {

        /** When the vehicle crosses the bounds of the safety zone
         * activate the hover failsafe. One may alternatively activate
         * the land failsafe by setting request_land_mode = true
         */
        
        std::string str = _ns + ": Activate hover failsafe, the FCU was about to crash";
        ROS_INFO("%s",str.c_str());

        request_hold_mode = true;
    }

    if( fine )
        fine = ( isActive() && !(request_land_mode || request_hold_mode) );

    return fine;
}

const bool Uav::isOutSafetyZone(){

    return ( _pos.y() < _safe_zone.ym || _pos.y() > _safe_zone.yM ||
                    _pos.z() < _safe_zone.zm || _pos.z() > _safe_zone.zM ||
                        _pos.x() < _safe_zone.xm || _pos.x() > _safe_zone.xM );
}

const bool Uav::isOutSafetyZone(const tf2::Vector3& pos){

    return ( pos.y() < _safe_zone.ym || pos.y() > _safe_zone.yM ||
                    pos.z() < _safe_zone.zm || pos.z() > _safe_zone.zM ||
                        pos.x() < _safe_zone.xm || pos.x() > _safe_zone.xM );
}

const bool Uav::took_off(){

    static bool did = false;

    if( !did )
        did = (isActive() && _pos.z() <= 0.1);

    return did;
}

const tf2::Vector3& Uav::position(){
    
    return _pos;
}

const tf2::Matrix3x3& Uav::orientation(){

    _R = tf2::Matrix3x3(_quaternion);
    return _R;
}

const tf2::Vector3& Uav::linear_vel(){
    
    return _vel;
}

const tf2::Vector3& Uav::angular_vel(){
    
    return _ang_vel;
}
