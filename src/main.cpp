
/** @file main.cpp
 * Implement tracking controllers
 * for multirotors, using the functions
 * of this package.
 * 
 * \author Joao Pinto (joao.s.pinto@tecnico.ulisboa.pt)
 * \date 2021-09-22
 */


#include <ros/ros.h>
#include <quad_ctrl_mavros/uav.h>
#include <quad_ctrl_mavros/utils.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <forward_list>


int main(int argc, char** argv){

    std::string other_ns;

    switch (argc) {

        case 6:
            // ROS_INFO("Single-vehicle mode");
            break;

        case 7:
            // ROS_INFO("Multi-vehicle mode");
            Uav::set_multi_vehicle_mode();
            other_ns = argv[4]; // Get namespace of the other vehicle
            break;

        default:
            return INCORRECT_NUMBER_ARGS;
    }

    // Get namespace of the vehicle
    std::string ns = argv[3];

    ////////////////////////////////
    // List of Trajectory instances
    ////////////////////////////////
    std::forward_list<Trajectory> trajectory_list;

    ///////////////////////////////////////
    // Load list with data from a CSV file
    ///////////////////////////////////////
    trajectory_list << argv[1];

    if ( trajectory_list.empty() ){

        ROS_INFO("Trajectory file %s does not exist", argv[1]);
        return UNEXISTENT_FILE;
    }

    ////////////////////////////
    // Trajectory list iterators
    ////////////////////////////
    auto trajectory_it = trajectory_list.begin();
    auto trajectory_prev_it = trajectory_it;

    ros::init(argc, argv, "quad_ctrl_mavros");
    ros::NodeHandle nh;

    float mass;
    float gravity;
    float thr_lim[2];
    float ang_vel_lim[3];
    Inertia J;
    Gains K;
    SafetyZone safe_zone;

    // PX4 has a timeout of 500ms between two Offboard commands!
    float fs = 100.0f;
    ros::Rate loop_rate(fs);

    /////////////////////////////
    // Load params from YAML file
    /////////////////////////////
    utils::load_params(nh, ns, K, mass, 
                            gravity, thr_lim, 
                                ang_vel_lim, J, safe_zone);

    ///////////////////////////////////////
    Uav uav(nh, ns, other_ns, mass, J, fs);
    ///////////////////////////////////////

    Uav::set_safety_zone(safe_zone);

    // It is necessary to provide a 
    // norm_fcn(float&, const float&) 
    // when constructing a Throttle instance
    std::function<void(float&, const float&)> norm_fcn;

    if ( std::atoi(argv[2]) > 0 ) {

        // Intel Aero Normalised Throttle-vs-Thrust Curve
        norm_fcn = [](float& th, const float& vel){
            th = ( tan( (th-10.37)/8.84 ) + 1.478 )/2.955;
        };

    } else {

        // Iris Normalised Thrust-vs-Thrust-vs-Airspeed
        // Curve (for SITL simulations)
        norm_fcn = [](float& th, const float& vel){
            th = ( th/( 1.0 - vel/25.0) - 1.52*9.80665 )/
                        ( 2*34.068*0.561 + 7.1202 ) + 0.561;
        };
    }

    Throttle thr(norm_fcn, thr_lim);

    tf2::Vector3 z_w(0.0f, 0.0f, 1.0f);

    tf2::Vector3 pos_error(0.f,0.f,0.f);
    tf2::Vector3 pos_error_prev(0.f,0.f,0.f);
    tf2::Vector3 vel_error(0.f,0.f,0.f);

    tf2::Vector3 integral(0.f,0.f,0.f);
    tf2::Vector3 increment_integral(0.f,0.f,0.f);

    thr = 0.f;
    float thr_unsat = 0.f;

    tf2::Vector3 F_des;
    tf2::Vector3 zb;
    tf2::Vector3 zb_des;
    tf2::Matrix3x3 R_des;
    AngularV w_des(ang_vel_lim);

    ////////////////////////////////////////////////////////////
    // Call this function before sending commands to the vehicle
    ////////////////////////////////////////////////////////////
    uav.start_up(thr,w_des);

    //////////////////////////////////////////////////////////
    // Send a position waypoint so that the vehicle takes off
    //////////////////////////////////////////////////////////
    uav.go_to_pos(trajectory_it->position);

    while ( ros::ok() )
    {   
        /////////////////////////////////////////////////////////////////////////
        // Request the quadcoper to land when it finishes tracking the trajectory
        /////////////////////////////////////////////////////////////////////////

        if ( trajectory_it == trajectory_list.end() && uav.ok() ){
            
            uav.land();
            break;
        }
        
        /////////////////////////////
        // Vehicle state variables 
        /////////////////////////////

        // pos = uav.position();
        // vel = uav.linear_vel();
        // R = uav.orientation();
        // w = uav.angular_vel();


        // Stop sending commands if the user requests an emergency land
        // or the quadcopter to hover

        if ( uav.ok() ){
            
            ////////////////////////////////////
            /** IMPLEMENT YOUR CONTROLLER HERE!
             * Below, an example of a trajectory
             * tracking controller is presented.
             */
            ////////////////////////////////////

            pos_error = uav.position() - trajectory_it->position;
            vel_error = uav.linear_vel() - trajectory_it->velocity;
            
            if ( abs( thr_unsat - thr.get_value() ) < EPS ){

                increment_integral = 1/(2*fs)*(pos_error_prev + pos_error);

                if ( abs( trajectory_it->position.x() 
                            - trajectory_prev_it->position.x() ) < 0.1 ){

                    if ( abs(pos_error.x()) < K.int_bounds.x() ){
                    
                        integral.setX( integral.x() + increment_integral.x() );
                        pos_error_prev.setX( pos_error.x() );

                    }

                } else {

                    pos_error_prev.setX(0);
                    increment_integral.setX(0);
                    integral.setX(0);
                }

                if ( abs( trajectory_it->position.y() 
                            - trajectory_prev_it->position.y() ) < 0.1 ){

                    if ( abs(pos_error.y()) < K.int_bounds.y() ){

                        integral.setY( integral.y() + increment_integral.y() );
                        pos_error_prev.setY( pos_error.y() );

                    }

                } else {

                    pos_error_prev.setY(0);
                    increment_integral.setY(0);
                    integral.setY(0);
                }

                if ( abs( trajectory_it->position.z() 
                            - trajectory_prev_it->position.z() ) < 0.1 ){

                    if ( abs(pos_error.z()) < K.int_bounds.z() ){

                        integral.setZ( integral.z() + increment_integral.z() );
                        pos_error_prev.setZ( pos_error.z() );
                        
                    }

                } else {

                    pos_error_prev.setZ(0);
                    increment_integral.setZ(0);
                    integral.setZ(0);
                }

            }

            F_des = -K.Kp*pos_error - K.Kv*vel_error - K.Ki*integral +
                        mass*gravity*z_w + mass*trajectory_it->acceleration;

            zb_des = tf2::Vector3(F_des);
            zb_des.normalize();

            zb = R_des.getColumn(2);
            zb.normalize();
            thr_unsat = F_des.dot(zb);
            thr.set_value( thr_unsat, uav.linear_vel().length() );     

            utils::getRd(R_des, trajectory_it->yaw, zb_des);

            // utils::get_wd(w_des, trajectory_it->jerk, uav.angular_vel(), 
            //                          R_des, trajectory_it->dyaw, mass, thr);

            // uav.send_cmd(thr,w_des);
            uav.send_cmd(thr,R_des);

            if ( trajectory_it != trajectory_prev_it )
                ++trajectory_prev_it;

            ++trajectory_it;

        } else {

            // Emergency landing
            if ( uav.landed() )
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ROS_INFO("Press ENTER to quit");
}