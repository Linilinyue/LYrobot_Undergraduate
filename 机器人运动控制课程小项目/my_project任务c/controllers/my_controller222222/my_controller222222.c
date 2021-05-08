
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <string.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 32
#define NUM_PISTONS 2
#define pi (3.1415926)
#define PI (3.1415926)

#define ROBOT_INDEX 5

#define COMMUNICATION_CHANNEL 1

WbDeviceTag motors[NUM_PISTONS];
WbDeviceTag position_sensor[NUM_PISTONS];
WbDeviceTag receiver;
WbDeviceTag emitter;
WbDeviceTag receiver2;
WbDeviceTag emitter2;
WbNodeRef root, robot;
WbFieldRef children;
WbDeviceTag ds0, ds1;

WbNodeRef root2, robot2;
WbFieldRef children2;

void find_devices()
{
  int i;
  for (i = 0; i < NUM_PISTONS; i++) {
    char motors_name[NUM_PISTONS][20]={"motor_left","motor_right"};
    motors[i] = wb_robot_get_device(motors_name[i]);
    position_sensor[i] = wb_motor_get_position_sensor(motors[i]);
    wb_position_sensor_enable(position_sensor[i],TIME_STEP);
    
    root=wb_supervisor_node_get_root();
    children=wb_supervisor_node_get_field(root, "children");
    robot = wb_supervisor_field_get_mf_node(children, ROBOT_INDEX);
    
    // wb_motor_enable_torque_feedback(motors[i],TIME_STEP);
    ds0 = wb_robot_get_device("ds0");
    ds1 = wb_robot_get_device("ds1");
    wb_distance_sensor_enable(ds0, TIME_STEP);
    wb_distance_sensor_enable(ds1, TIME_STEP);
  }
  
  // receiver = wb_robot_get_device("receiver111");
  // wb_receiver_enable(receiver, TIME_STEP);
}
double x;
double z;
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  find_devices();


  while (wb_robot_step(TIME_STEP) != -1) {
        const double *centroid = wb_supervisor_node_get_center_of_mass(robot);
        const double *orientation = wb_supervisor_node_get_orientation(robot);
        x=centroid[0];
        z=centroid[2];
        double r=0;
        
        if(x*x+z*z>0.1){
            wb_motor_set_position(motors[0],INFINITY);
            wb_motor_set_position(motors[1],INFINITY);
            const double ds0_value = wb_distance_sensor_get_value(ds0);
            const double ds1_value = wb_distance_sensor_get_value(ds1);
            
            double left_speed, right_speed;
            double SPEED = 7;
            
        
            if (ds1_value > 500) {
    
              if (ds0_value > 200) {
                  left_speed = -SPEED;
                  right_speed = -SPEED / 2;
              } else {
                left_speed = -ds1_value / 100;
                right_speed = (ds0_value / 100) + 1.5;
      
              }
            } else if (ds0_value > 500) {
              left_speed = (ds1_value / 100) + 1.5;
              right_speed = -ds0_value / 100;
      
            } else {
    
                  const double *centroid = wb_supervisor_node_get_center_of_mass(robot);
                  const double *orientation = wb_supervisor_node_get_orientation(robot);
                  x=centroid[0];
                  z=centroid[2];
                  
                  double ori0,ori2,ori4,ori6,ori8;
                  ori0=orientation[0];
                  // ori2=orientation[2];
                  ori4=orientation[4];
                  // ori6=orientation[6];
                  // ori8=orientation[8];
                  
                  
                  
                  double th1 = acos(ori0);
                  double th2;
                  if(x*z<0){
                    th2=atan(-x/z);
                  }
                  else{
                    th2=atan(x/z);
                  }
                  
                  printf("%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",ori4,acos(ori0),th2,x,z,x/z);
                  // while(th1<0){
                      // th1=th1+2*pi;
                  // }
                  // while(th1>2*pi){
                      // th1=th1-2*pi;
                  // }
                  // while(th2<0){
                      // th2=th2+2*pi;
                  // }
                  // while(th2>2*pi){
                      // th2=th2-2*pi;
                  // }
                  // th2=th2-pi/12;
                  // th1=th1+pi/2;
                  
                  if(x>0){
                    if(th1>0.01){
                      double th0 = wb_position_sensor_get_value(position_sensor[0]);
                      double th1 = wb_position_sensor_get_value(position_sensor[1]);
                      wb_motor_set_position(motors[0],th0-0.1);
                      wb_motor_set_position(motors[1],th1+0.1);
                    }
                  }
                  else{
                    if(z>0){
                        if(th1>pi/2+0.01||th1<pi/2-0.01){
                        double th0 = wb_position_sensor_get_value(position_sensor[0]);
                        double th1 = wb_position_sensor_get_value(position_sensor[1]);
                        wb_motor_set_position(motors[0],th0-0.1);
                        wb_motor_set_position(motors[1],th1+0.1);
                      }
                      
                    }

                  }
                  
    
              left_speed = SPEED;
              right_speed = SPEED;
            }
        
            /* set the motor speeds. */
            wb_motor_set_velocity(motors[1], left_speed);
            wb_motor_set_velocity(motors[0], right_speed);
            
          }
          else{
                    wb_motor_set_position(motors[0],INFINITY);
                    wb_motor_set_position(motors[1],INFINITY);
                    wb_motor_set_velocity(motors[0],0);
                    wb_motor_set_velocity(motors[1],0);
          }
  
  
        
        
        
  };


  wb_robot_cleanup();

  return 0;
}
