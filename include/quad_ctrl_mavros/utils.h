
/** @file utils.h
 * Collection of structs which allow to
 * bound and normalise control inputs, and to
 * store information of a given trajectory.
 * 
 * Some operators are introduced to manipulate
 * tf2::Matrix3x3 objects.
 * 
 * The namespace 'utils' provides various
 * functions which are useful to perform
 * compuations which are necessary to obtain
 * control inputs, and other functions which
 * allow to load parameters from a YAML file
 * and to load some trajectory from a CSV file,
 * which then can be set as reference trajectory
 * for the vehicle to track.
 * 
 * \author Joao Pinto (joao.s.pinto@tecnico.ulisboa.pt)
 * \date 2021-09-22
 */


#pragma once

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include <functional>
#include <forward_list>
#include <fstream>
#include <string>

#define INCORRECT_NUMBER_ARGS 1
#define UNEXISTENT_FILE 2
#define EPS 1e-9

/** \brief Struct that groups together
 * a set of floats which define the boundaries
 * of a zone where the vehicle can fly safely.
 * For each of the three-axis (x, y, and z), two
 * values ( a minimum, \b m and a maximum, \b M ) 
 * are defined to set an admissible interval in 
 * each of those axes,
 * 
 * \b xm <= x <= \b xM ,
 * \b ym <= y <= \b yM ,
 * \b zm <= z <= \b zM .
 * 
 */
struct SafetyZone{

    float xm, xM, ym, yM, zm, zM;

    SafetyZone() {};

    SafetyZone(const float& _xm, const float& _xM, 
                    const float& _ym, const float& _yM,
                        const float& _zm, const float& _zM) :
                            xm(_xm), xM(_xM), ym(_ym), yM(_yM), zm(_zm), zM(_zM) {};

};

/** \brief Store the inertia tensor and its inverse.
 */
struct Inertia{

    tf2::Matrix3x3 _J;
    tf2::Matrix3x3 _J_inverse;

    Inertia() {};

    Inertia(const float& jxx, const float& jxy, const float& jxz,
                const float& jyx, const float& jyy, const float& jyz,
                    const float& jzx, const float& jzy, const float& jzz){

                        _J = tf2::Matrix3x3(jxx, jxy, jxz,
                                                jyx, jyy, jyz,
                                                    jzx, jzy, jzz);

                        _J_inverse = _J.inverse();

                    };

    Inertia(const tf2::Matrix3x3& m) : _J(m),  
                                    _J_inverse(m.inverse()) {};

    Inertia(const Inertia& In) : _J(In._J), 
                                    _J_inverse(In._J_inverse) {};

    const tf2::Matrix3x3& inverse() const{
        return _J_inverse;
    }

};

/** \brief Pack the matrix gains of 
 * a given controller in a single struct.
 */
struct Gains{

    tf2::Matrix3x3 Kp, Kv;
    tf2::Matrix3x3 Ki = tf2::Matrix3x3(0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f);
    tf2::Matrix3x3 Ki_inverse;

    tf2::Vector3 int_bounds;

    Gains() {};

    Gains(const tf2::Matrix3x3& _Kp, const tf2::Matrix3x3& _Kv)
                                    : Kp(_Kp), Kv(_Kv) {};

    Gains(const tf2::Matrix3x3& _Kp, 
                const tf2::Matrix3x3& _Kv, 
                    const tf2::Matrix3x3& _Ki) : Kp(_Kp), Kv(_Kv), Ki(_Ki) {};

    Gains(const Gains& _K) : Kp(_K.Kp), Kv(_K.Kv), Ki(_K.Ki) {};

    void set_integral_inverse(){

        if ( Ki[0].getX() ) {

            Ki_inverse[0].setX( 1/Ki[0].getX() );
        }

        if ( Ki[1].getY() ) {

            Ki_inverse[1].setY( 1/Ki[1].getY() );
        }

        if ( Ki[2].getZ() ) {

            Ki_inverse[2].setZ( 1/Ki[2].getZ() );
        }

    }

    const tf2::Matrix3x3& get_integral_inverse() const{
        return Ki_inverse;
    }

};

/** \brief Handles thrust bounding and, given a
 * throttle-vs-thrust curve, computes a throttle 
 * input to be provided to the vehicle from 
 * the desired thrust input computed by a given 
 * controller.
 */
struct Throttle{

    float _thr;
    float _thr_norm;
    float _thr_lim[2];
    const std::function<void(float&,const float&)> norm_fcn;

    Throttle() = delete;
    Throttle(float) = delete;

    Throttle(const std::function<void(float&, const float&)>& fcn,
                        float *thr_lim) : norm_fcn(fcn), 
                            _thr_lim {thr_lim[0], thr_lim[1]} {};

    void set_value(const float& thr, const float& vel){
        _thr = thr;
        _thr = ( ( _thr >= _thr_lim[0] && _thr <= _thr_lim[1] )? _thr : clip() );
        _thr_norm = _thr;
        norm_fcn(_thr_norm, vel);
    }

    const float& clip(){

        if(_thr < _thr_lim[0]){
            return _thr_lim[0];
        } 

        if(_thr > _thr_lim[1]){
            return _thr_lim[1];
        }

        return _thr; 
    }

    const float& get_value() const{
        return _thr;
    }

    const float& get_value_norm() const{
        return _thr_norm;
    }

    void operator=(const float& thr){
        set_value(thr,0);
    }

};

/** \brief Handles torque inputs bounding.
 */
struct Torques{

    tf2::Vector3 _torques;
    float _torques_lim[3];

    Torques() = delete;
    Torques(float,float,float) = delete;
    Torques(tf2::Vector3) = delete;

    Torques(const float* lim) 
        : _torques_lim {lim[0], lim[1], lim[2]} {};

    void clip(){

        if ( abs(_torques.x()) > _torques_lim[0] ){
            _torques.setX( _torques_lim[0] * abs(_torques.x()) / _torques.x() );
        }

        if ( abs(_torques.y()) > _torques_lim[1] ){
            _torques.setY( _torques_lim[1] * abs(_torques.y()) / _torques.y() );
        }

        if ( abs(_torques.z()) > _torques_lim[2] ){
            _torques.setZ( _torques_lim[2] * abs(_torques.z()) / _torques.z() );
        }

    }

    const tf2::Vector3& get_value() const{
        return _torques;
    }

    const tf2Scalar& x() const{
        return _torques.x();
    }

    const tf2Scalar& y() const{
        return _torques.y();
    }

    const tf2Scalar& z() const{
        return _torques.z();
    }

    void set_value(const float& tx, const float& ty, const float& tz){

        _torques.setX(tx);
        _torques.setY(ty);
        _torques.setZ(tz);

        clip();
    }

    void set_value(const tf2::Vector3& tau){

        _torques.setX(tau.x());
        _torques.setY(tau.y());
        _torques.setZ(tau.z());

        clip();
    }

    void operator=(const tf2::Vector3& tau){
       set_value(tau);
    }

};

/** \brief Handles angular rate inputs bounding.
 */
struct AngularV{

    tf2::Vector3 _ang_vel;
    float _ang_vel_lim[3];

    AngularV() = delete;
    AngularV(float,float,float) = delete;
    AngularV(tf2::Vector3) = delete;

    AngularV(const float* lim) 
        : _ang_vel_lim {lim[0], lim[1], lim[2]} {};

    void clip(){

        if ( abs(_ang_vel.x()) > _ang_vel_lim[0] ){
            _ang_vel.setX( _ang_vel_lim[0] * abs(_ang_vel.x()) / _ang_vel.x() );
        }

        if ( abs(_ang_vel.y()) > _ang_vel_lim[1] ){
            _ang_vel.setY( _ang_vel_lim[1] * abs(_ang_vel.y()) / _ang_vel.y() );
        }

        if ( abs(_ang_vel.z()) > _ang_vel_lim[2] ){
            _ang_vel.setZ( _ang_vel_lim[2] * abs(_ang_vel.z()) / _ang_vel.z() );
        }

    }

    const tf2::Vector3& get_value() const{
        return _ang_vel;
    }

    const tf2Scalar& x() const{
        return _ang_vel.x();
    }

    const tf2Scalar& y() const{
        return _ang_vel.y();
    }

    const tf2Scalar& z() const{
        return _ang_vel.z();
    }

    void set_value(const float& omx, 
                        const float& omy, const float& omz){

        _ang_vel.setX(omx);
        _ang_vel.setY(omy);
        _ang_vel.setZ(omz);

        clip();
    }

    void set_value(const tf2::Vector3& angv){

        _ang_vel.setX(angv.x());
        _ang_vel.setY(angv.y());
        _ang_vel.setZ(angv.z());

        clip();
    }

    void operator=(const tf2::Vector3& angv){
       set_value(angv);
    }

};

/** \brief Store the information of 
 * a given trajectory in a single struct.
 */
struct Trajectory{

    float time;
    tf2::Vector3 position;
    tf2::Vector3 velocity;
    tf2::Vector3 acceleration;
    tf2::Vector3 jerk;
    float yaw;
    float dyaw;

    Trajectory(const float& _time, const tf2::Vector3& _pos, 
                    const tf2::Vector3& _vel, const tf2::Vector3& _acc, 
                        const tf2::Vector3& _jerk, const float& _yaw, const float& _dyaw)
                            : time(_time), position(_pos), velocity(_vel), acceleration(_acc),
                                jerk(_jerk), yaw(_yaw), dyaw(_dyaw) {};

    Trajectory(const float* ta) : time(ta[0]), 
                                    position( tf2::Vector3(ta[1],ta[2],ta[3]) ),
                                        velocity( tf2::Vector3(ta[4],ta[5],ta[6]) ), 
                                            acceleration( tf2::Vector3(ta[7],ta[8],ta[9]) ),
                                                    jerk( tf2::Vector3(ta[10],ta[11],ta[12]) ),
                                                                    yaw(ta[13]), dyaw(ta[14]) {};
};

/** \brief Sum two Matrix3x3.
 */
tf2::Matrix3x3 operator+(const tf2::Matrix3x3& m1, const tf2::Matrix3x3& m2);

/** \brief Subtract two Matrix3x3.
 */
tf2::Matrix3x3 operator-(const tf2::Matrix3x3& m1, const tf2::Matrix3x3& m2);

/** \brief Return the negative of a matrix.
 */
tf2::Matrix3x3 operator-(const tf2::Matrix3x3& m);


/** \brief Multiply matrix by scalar.
 */
tf2::Matrix3x3 operator*(const tf2::Matrix3x3& m, const tf2Scalar& s);

/** \brief Multiply matrix by scalar.
 */
tf2::Matrix3x3 operator*(const tf2Scalar& s, const tf2::Matrix3x3& m);

/** \brief Multiply matrix by scalar.
 */
void operator*=(tf2::Matrix3x3& m, const tf2Scalar& s);

/** \brief Unskew matrix.
 * Ensure that the matrix passed by reference is skew-symmetric!
 */
tf2::Vector3 operator!(const tf2::Matrix3x3& m);

/** \brief Print matrix to the cout.
 */
std::ostream& operator<<(std::ostream& os, const tf2::Matrix3x3& m);

/** \brief Print matrix to the cout.
 */
std::ostream& operator<<(std::ostream& os, tf2::Matrix3x3& m);

/** \brief Fill a list of 'Trajectory' instances with data
 * contained in a CSV (Comma-Separated values) file.
 * \param file_name Ensure that the CSV file to be opened
 * matches the definition of struct 'Trajectory'. By default,
 * this struct can be thought as an array with \b 15 entries
 * which stores, in the following order, information on:
 * \b time \b instant, \b position, \b linear \b velocity, 
 * \b acceleration, and \b jerk, the \b yaw angle, and 
 * \b its \b first \b time \b derivative. Hence, each line of the file
 * must contain \b 15 values, separated by commas, in the
 * specific order just stated. The lines in the file must be sorted 
 * according to the time instants (first entry), in descending order, 
 * for the reasons explained next.
 * \param trajectory_list Singly-linked list which keeps the
 * various 'Trajectory' instances, created from each line of
 * the CSV file. New 'Trajectory' instances are added to
 * the beginning of the list.
 */
void operator<<(std::forward_list<Trajectory>& trajectory_list,
                                                const char *file_name);

namespace utils{

    /** \brief Convert a Point message to its equivalent tf2 representation.
     * This function is a specialization of the fromMsg template defined in tf2/convert.h.
     * 
     * This function is in tf2_geometry_msgs.h (Author: Wim Meeussen)
     * Copyright (c) 2008, Willow Garage, Inc.
     * 
     * \param in A Point message type.
     * \param out The Vector3 converted to a tf2 type.
     */
    inline
    void fromMsg(const geometry_msgs::Point& in, tf2::Vector3& out)
    {
        out = tf2::Vector3(in.x, in.y, in.z);
    }

    /** \brief Convert a Vector3 message to its equivalent tf2 representation.
     * This function is a specialization of the fromMsg template defined in tf2/convert.h.
     * 
     * This function is in tf2_geometry_msgs.h (Author: Wim Meeussen)
     * Copyright (c) 2008, Willow Garage, Inc.
     * 
     * \param in A Vector3 message type.
     * \param out The Vector3 converted to a tf2 type.
     */
    inline
    void fromMsg(const geometry_msgs::Vector3& in, tf2::Vector3& out)
    {
        out = tf2::Vector3(in.x, in.y, in.z);
    }

    /** \brief Convert a Quaternion message to its equivalent tf2 representation.
     * 
     * \param in A Quaternion message type.
     * \param out The Quaternion converted to a tf2 type.
     */
    inline
    void fromMsg(const geometry_msgs::Quaternion& in, tf2::Quaternion& out)
    {
        out = tf2::Quaternion(in.x, in.y, in.z, in.w);
    }

    /** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
     * 
     * \param in A tf2 Vector3 object.
     * \param out A geometry_msgs::Vector3 message.
     */
    inline
    void toMsg(const tf2::Vector3& in, geometry_msgs::Vector3& out)
    {
        out.x = in.getX();
        out.y = in.getY();
        out.z = in.getZ();
    }

    /** \brief Convert a tf2 Quaternion type to its equivalent geometry_msgs representation.
     * 
     * \param in A tf2 Quaternion object.
     * \param out A geometry_msgs::Quaternion message.
     */
    inline
    void toMsg(const tf2::Quaternion& in, geometry_msgs::Quaternion& out)
    {
        out.x = in.getX();
        out.y = in.getY();
        out.z = in.getZ();
        out.w = in.getW();
    }

    /** \brief Convert a tf2 Vector3 type representing 
     * a positon vector to its equivalent geometry_msgs representation.
     * 
     * \param in A tf2 Vector3 object.
     * \param out A geometry_msgs::Vector3 message.
     */
    inline
    void toMsg(const tf2::Vector3& in, geometry_msgs::Pose& out)
    {
        out.position.x = in.getX();
        out.position.y = in.getY();
        out.position.z = in.getZ();
    }

    /** \brief Sets the provided matrix as the
     * identity matrix and scales it by a gain.
     */
    void scale_gain(tf2::Matrix3x3& K, const float& gain);

    /** \brief Load parameters from a \b YAML file.
     * Set the gains of the controller, the mass of vehicle in kg,
     * the constant of gravity in m/s^2, the min and max thrust in N
     * (Note that to define the minimum and maximum value of throttle
     * you should check out the parameters MPC_THR_MIN and MPC_THR_MAX),
     * the max torque around each of the three axis in Nm, the inertia tensor
     * in Kg m^2, and the bounds of the safety zone in m.
     * \b Edit the \b files \b in \b folder 
     * \b 'config' \b before \b flying.
     */
    void load_params(ros::NodeHandle& nh, const std::string& ns, 
                            Gains& K, float& mass, float& gravity, float* thr_lim, 
                                            float* tor_lim, Inertia& J, SafetyZone& sz);

    /** \brief From a desired direction of thrust and
     * a value of the yaw angle, compute the corresponding
     * rotation matrix.
     * \param Rd Rotation matrix obtained from the other
     * parameters;
     * \param yaw_des Yaw angle in radians;
     * \param z_b Desired direction of thrust, the
     * z-axis of the body-frame.
     */
    void getRd(tf2::Matrix3x3& Rd, const float& yaw_des, 
                                    const tf2::Vector3& z_b);

    /** \brief Compute the desired angular velocity from    
     * jerk, orientation, the first derivative of the yaw angle,
     * mass, and throttle input reference trajectories.
     * \param w_des Desired angular velocity in rad/s, obtained
     * from the following parameters;
     * \param j_des Desired jerk vector in m/s^3;
     * \param ang_vel Actual Angular velocity of the vehicle;
     * \param R_des Desired rotation matrix;
     * \param dyaw_des Desired value of the first derivative
     * of the yaw angle in rad/s;
     * \param mass Vehicle's mass in kg;
     * \param T Throttle input.
     */
    void get_wd(AngularV& w_des, const tf2::Vector3& j_des,
                        const tf2::Vector3& ang_vel, const tf2::Matrix3x3& R_des, 
                            const float& dyaw_des, const float& mass, const Throttle& T);

    /** \brief Compute the desired angular velocity from
     * jerk, orientation, the first derivative of the yaw angle,
     * mass, and throttle input reference trajectories and obtain 
     * the angular velocity error of the vehicle relative to the 
     * desired value.
     * \param error Angular velocity error vector;
     * \param j_des Desired jerk vector in m/s^3;
     * \param ang_vel Actual Angular velocity of the vehicle;
     * \param R_des Desired rotation matrix;
     * \param dyaw_des Desired value of the first derivative
     * of the yaw angle in rad/s;
     * \param mass Vehicle's mass in kg;
     * \param T Throttle input.
     */
    void compute_ang_error(tf2::Vector3& error, const tf2::Vector3& j_des,
                                const tf2::Vector3& ang_vel, const tf2::Matrix3x3& R_des, 
                                    const float& dyaw_des, const float& mass, const Throttle& T);

    /** \brief Fill a list of 'Trajectory' instances with data
     * contained in a CSV (Comma-Separated values) file.
     * \param file_name Ensure that the CSV file to be opened
     * matches the definition of struct 'Trajectory'. By default,
     * this struct can be thought as an array with \b 15 entries
     * which stores, in the following order, information on:
     * \b time \b instant, \b position, \b linear \b velocity, 
     * \b acceleration, and \b jerk, the \b yaw angle, and 
     * \b its \b first \b time \b derivative. Hence, each line of 
     * the filemust contain \b 15 values, separated by commas, in the
     * specific order just stated. The lines in the file must be sorted 
     * according to the time instants (first entry), in descending order, 
     * for the reasons explained next.
     * \param trajectory_list Singly-linked list which keeps the
     * various 'Trajectory' instances, created from each line of
     * the CSV file. New 'Trajectory' instances are added to
     * the beginning of the list.
     */
    void fill_pose_list_from_file(const char *file_name, 
                                        std::forward_list<Trajectory>& trajectory_list);

}