
/** @file utils.cpp
 * Definition of the funtions of namespace
 * 'utils' declared in utils.h
 * 
 * \author Joao Pinto (joao.s.pinto@tecnico.ulisboa.pt)
 * \date 2021-09-22
 */

#include <quad_ctrl_mavros/utils.h>

void utils::scale_gain(tf2::Matrix3x3& K, const float& gain){

    K.setIdentity();
    K *= gain;

}

void utils::load_params(ros::NodeHandle& nh, const std::string& ns, Gains& K, 
                                    float& mass, float& gravity, float* thr_lim,
                                            float* ang_vel_lim, Inertia& J, SafetyZone& sz){

    float jxx, jxy, jxz,
            jyx, jyy, jyz,
                jzx, jzy, jzz;

    float kp, kv;
    float kix, kiy, kiz;
    float kix_bound, kiy_bound, kiz_bound;
    float xm, xM, ym, yM, zm, zM;

    // It is strongly recommended that the default parameter
    // values are verified before flying!

    nh.param<float>(ns+"/physical_params/mass", mass, 0.152f);

    nh.param<float>(ns+"/physical_params/gravity", gravity, 9.8f);

    nh.param<float>(ns+"/physical_params/inertia/jxx", jxx, 0.0138f);
    nh.param<float>(ns+"/physical_params/inertia/jxy", jxy, 0.0f);
    nh.param<float>(ns+"/physical_params/inertia/jxz", jxz, 0.0f);
    nh.param<float>(ns+"/physical_params/inertia/jyx", jyx, 0.0f);
    nh.param<float>(ns+"/physical_params/inertia/jyy", jyy, 0.0110f);
    nh.param<float>(ns+"/physical_params/inertia/jyz", jyz, 0.0f);
    nh.param<float>(ns+"/physical_params/inertia/jzx", jzx, 0.0f);
    nh.param<float>(ns+"/physical_params/inertia/jzy", jzy, 0.0f);
    nh.param<float>(ns+"/physical_params/inertia/jzz", jzz, 0.0082f);

    nh.param<float>(ns+"/gains/kp", kp, 4.5f);
    nh.param<float>(ns+"/gains/kv", kv, 4.3f);
    nh.param<float>(ns+"/gains/kix", kix, 0.0f);
    nh.param<float>(ns+"/gains/kiy", kiy, 0.0f);
    nh.param<float>(ns+"/gains/kiz", kiz, 0.0f);

    nh.param<float>(ns+"/gains/integral_bounds/x", kix_bound, 0.0f);
    nh.param<float>(ns+"/gains/integral_bounds/y", kiy_bound, 0.0f);
    nh.param<float>(ns+"/gains/integral_bounds/z", kiz_bound, 0.0f);

    nh.param<float>(ns+"/throttle/throttle_min", thr_lim[0], 1.74f);
    nh.param<float>(ns+"/throttle/throttle_max", thr_lim[1], 19.1f);

    nh.param<float>(ns+"/ang_vel/w_max_x", ang_vel_lim[0], 6.0f);
    nh.param<float>(ns+"/ang_vel/w_max_y", ang_vel_lim[1], 6.0f);
    nh.param<float>(ns+"/ang_vel/w_max_z", ang_vel_lim[2], 6.0f);

    nh.param<float>(ns+"/safety_zone/x_min", xm, -2);
    nh.param<float>(ns+"/safety_zone/x_max", xM, 2);
    nh.param<float>(ns+"/safety_zone/y_min", ym, -1.5);
    nh.param<float>(ns+"/safety_zone/y_max", yM, 1.5);
    nh.param<float>(ns+"/safety_zone/z_min", zm, 0.5);
    nh.param<float>(ns+"/safety_zone/z_max", zM, 2.3);

    scale_gain(K.Kp, kp);
    scale_gain(K.Kv, kv);

    // x-axis integrator only
    K.Ki[0].setX(kix);
    K.int_bounds.setX(kix_bound);

    // y-axis integrator only
    K.Ki[1].setY(kiy);
    K.int_bounds.setY(kiy_bound);

    // z-axis integrator only
    K.Ki[2].setZ(kiz);
    K.int_bounds.setZ(kiz_bound);

    J = Inertia(jxx, jxy, jxz, jyx, jyy, jyz, jzx, jzy, jzz);
    sz = SafetyZone(xm, xM, ym, yM, zm, zM);
}

void utils::getRd(tf2::Matrix3x3& Rd, const float& yaw_des, 
                                        const tf2::Vector3& z_b){

    tf2::Vector3 x_c( cos(yaw_des), sin(yaw_des), 0 );

    tf2::Vector3 y_b = tf2::tf2Cross( z_b, x_c );
    y_b.normalize();

    tf2::Vector3 x_b = tf2::tf2Cross( y_b, z_b );

    Rd = tf2::Matrix3x3(x_b.x(), y_b.x(), z_b.x(),
                        x_b.y(), y_b.y(), z_b.y(),
                        x_b.z(), y_b.z(), z_b.z());

}

void utils::compute_ang_error(tf2::Vector3& error, const tf2::Vector3& j_des,
                                const tf2::Vector3& ang_vel, const tf2::Matrix3x3& R_des, 
                                    const float& dyaw_des, const float& mass, const Throttle& T){

    tf2::Vector3 z_b = R_des.getColumn(2);
    tf2::Vector3 h_w = mass*( j_des - tf2::tf2Dot( z_b, j_des )*z_b )/T.get_value();

    tf2::Vector3 w_des( -tf2::tf2Dot( h_w, R_des.getColumn(1) ), 
                                tf2::tf2Dot( h_w, R_des.getColumn(0) ), 
                                                    dyaw_des * z_b.z() );

    error = ang_vel - w_des;
}

void utils::get_wd(AngularV& w_des, const tf2::Vector3& j_des,
                                const tf2::Vector3& ang_vel, const tf2::Matrix3x3& R_des, 
                                    const float& dyaw_des, const float& mass, const Throttle& T){

    tf2::Vector3 z_b = R_des.getColumn(2);
    tf2::Vector3 h_w = mass*( j_des - tf2::tf2Dot( z_b, j_des )*z_b )/T.get_value();

    w_des = tf2::Vector3( -tf2::tf2Dot( h_w, R_des.getColumn(1) ), 
                                tf2::tf2Dot( h_w, R_des.getColumn(0) ), 
                                                    dyaw_des * z_b.z() );

}

void utils::fill_pose_list_from_file(const char *file_name, 
                            std::forward_list<Trajectory>& trajectory_list){

    std::ifstream file(file_name);
    std::stringstream line_stream;
    std::string f_line;
    std::string token;

    // Check the definition of struct 'Trajectory'
    // to ensure the file is loaded successfully

    float traj_array[15];
    int j = 0;

    while( std::getline(file, f_line, '\n') ){

        line_stream = std::stringstream(f_line);

        j = 0;

        while( std::getline(line_stream, token, ',') ){

            traj_array[j] = std::atof(token.c_str());
            ++j;
        }

        Trajectory elem(traj_array);
        trajectory_list.push_front(elem);

    }

}

void operator<<(std::forward_list<Trajectory>& trajectory_list,
                                                const char *file_name){
            
    utils::fill_pose_list_from_file(file_name,trajectory_list);
}


tf2::Matrix3x3 operator+(const tf2::Matrix3x3& m1, const tf2::Matrix3x3& m2){

	return tf2::Matrix3x3( m1[0].x() + m2[0].x(), m1[0].y() + m2[0].y(), m1[0].z() + m2[0].z(),
	 				       m1[1].x() + m2[1].x(), m1[1].y() + m2[1].y(), m1[1].z() + m2[1].z(),
					       m1[2].x() + m2[2].x(), m1[2].y() + m2[2].y(), m1[2].z() + m2[2].z() );
}

tf2::Matrix3x3 operator-(const tf2::Matrix3x3& m1, const tf2::Matrix3x3& m2){

	return m1 + (-m2);
}

tf2::Matrix3x3 operator-(const tf2::Matrix3x3& m){

    return tf2::Matrix3x3( -m[0].x(), -m[0].y(), -m[0].z(), 
                           -m[1].x(), -m[1].y(), -m[1].z(),
                           -m[2].x(), -m[2].y(), -m[2].z() );
}

tf2::Matrix3x3 operator*(const tf2::Matrix3x3& m, const tf2Scalar& s){

    return tf2::Matrix3x3( s*m[0].x(), s*m[0].y(), s*m[0].z(), 
                           s*m[1].x(), s*m[1].y(), s*m[1].z(),
                           s*m[2].x(), s*m[2].y(), s*m[2].z() );
}

tf2::Matrix3x3 operator*(const tf2Scalar& s, const tf2::Matrix3x3& m){

    return m*s;
}

void operator*=(tf2::Matrix3x3& m, const tf2Scalar& s){

    m[0].setX( s*m[0].x() ); m[0].setY( s*m[0].y() ); m[0].setZ( s*m[0].z() ); 
    m[1].setX( s*m[1].x() ); m[1].setY( s*m[1].y() ); m[1].setZ( s*m[1].z() );
    m[2].setX( s*m[2].x() ); m[2].setY( s*m[2].y() ); m[2].setZ( s*m[2].z() );
}

tf2::Vector3 operator!(const tf2::Matrix3x3& m){

    return tf2::Vector3( m[2].y(), m[0].z(), m[1].x() );
}

std::ostream& operator<<(std::ostream& os, tf2::Matrix3x3& m){

    os << m[0].x() << " " << m[0].y() << " " << m[0].z() << std::endl
       << m[1].x() << " " << m[1].y() << " " << m[1].z() << std::endl
       << m[2].x() << " " << m[2].y() << " " << m[2].z() << std::endl;

    return os;
}

std::ostream& operator<<(std::ostream& os, const tf2::Matrix3x3& m){

    os << m[0].x() << " " << m[0].y() << " " << m[0].z() << std::endl
       << m[1].x() << " " << m[1].y() << " " << m[1].z() << std::endl
       << m[2].x() << " " << m[2].y() << " " << m[2].z() << std::endl;

    return os;
}
