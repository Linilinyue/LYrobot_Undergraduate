
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




struct message{
  double x;
  double z;
}messa;

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
  
  receiver = wb_robot_get_device("receiver111");
  wb_receiver_enable(receiver, TIME_STEP);
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  double ox=10;
  double oz=10;
  double dist=0;
  double rx=10;
  double rz=10;
  bool flaaag=false;
  if (strncmp(wb_robot_get_name(), "myrobot", 7) == 0){
            find_devices();
            
          
            // wb_motor_set_acceleration(motors[0],-1);
            // wb_motor_set_acceleration(motors[1],-1);
            wb_motor_set_control_pid(motors[0],10,0,0);
            wb_motor_set_control_pid(motors[1],10,0,0);
            
            double th0,th1;
            
          
          
          
            // wb_motor_set_position(motors[0],-15.5/12*pi);
            // wb_motor_set_position(motors[1],15.5/12*pi);
            // double th0 = wb_position_sensor_get_value(position_sensor[0]);
            // double th1 = wb_position_sensor_get_value(position_sensor[1]);
            // th0+=1/0.06;
            // th1+=1/0.06;
            // wb_motor_set_position(motors[0],th0-15.5/12*pi);
            // wb_motor_set_position(motors[1],th1+15.5/12*pi);
          
            // wb_motor_set_position(motors[0],INFINITY);
            // wb_motor_set_position(motors[1],INFINITY);
          
          
          
          
          
            while (wb_robot_step(TIME_STEP) != -1) {
            
              double t = wb_robot_get_time();
          
              double T = 2*pi/0.6;
              
              double v = 25.0/6;
              bool flag = false;
              
          
               if(flaaag==false){
               if(t<1||(2<=t&&t<3)||(4<=t&&t<6)||(7<=t&&t<9)||(10<=t&&t<12)||(13<=t&&t<14)||(15<=t&&t<16)){
                    flag=false;
                    wb_motor_set_position(motors[0],INFINITY);
                    wb_motor_set_position(motors[1],INFINITY);
                    wb_motor_set_velocity(motors[0],v);
                    wb_motor_set_velocity(motors[1],v);
                  }
                  else if((1<=t&&t<2)||(3<=t&&t<4)||(6<=t&&t<7)||(9<=t&&t<10)||(12<=t&&t<13)||(14<=t&&t<15)){
                    if(flag==false){
                      flag=true;
                        th0 = wb_position_sensor_get_value(position_sensor[0]);
                        th1 = wb_position_sensor_get_value(position_sensor[1]);
                        wb_motor_set_position(motors[0],th0-15.5/24*pi);
                        wb_motor_set_position(motors[1],th1+15.5/24*pi);
                    }
                  }
                   else if(t<=16+T){
                      wb_motor_set_velocity(motors[0],-6.55);
                      wb_motor_set_velocity(motors[1],-3.45);
                   }
                   else if(t<=2*T+16+0.4){
                      wb_motor_set_velocity(motors[0],-3.45);
                      wb_motor_set_velocity(motors[1],-6.55);
                      th0 = wb_position_sensor_get_value(position_sensor[0]);
                      th1 = wb_position_sensor_get_value(position_sensor[1]);
                   }
                   else if(t<=2*T+16+1){
                      // wb_motor_set_velocity(motors[0],0);
                      // wb_motor_set_velocity(motors[1],0);
                        double th0 = wb_position_sensor_get_value(position_sensor[0]);
                        double th1 = wb_position_sensor_get_value(position_sensor[1]);
                        wb_motor_set_position(motors[0],th0-15.5/36*pi);
                        wb_motor_set_position(motors[1],th1+15.5/36*pi);
                   }
                   else{
                         
                          // wb_motor_set_position(motors[0],INFINITY);
                          // wb_motor_set_position(motors[1],INFINITY);
                          
                          wb_motor_set_position(motors[0],INFINITY);
                          wb_motor_set_position(motors[1],INFINITY);
                          const double ds0_value = wb_distance_sensor_get_value(ds0);
                          const double ds1_value = wb_distance_sensor_get_value(ds1);
                          
                          double left_speed, right_speed;
                          double SPEED = 7;
                          
                      
                          if (ds1_value > 500) {
                            /*
                             * If both distance sensors are detecting something, this means that
                             * we are facing a wall. In this case we need to move backwards.
                             */
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
                            /*
                             * if nothing was detected we can move forward at maximal speed.
                             */
                                const double *direc = wb_receiver_get_emitter_direction(receiver);
                                if(direc[2]>0.3 || direc[0]>-0.7){
                                  double th0 = wb_position_sensor_get_value(position_sensor[0]);
                                  double th1 = wb_position_sensor_get_value(position_sensor[1]);
                                  wb_motor_set_position(motors[0],th0-0.1);
                                  wb_motor_set_position(motors[1],th1+0.1);
                                }
                            left_speed = SPEED;
                            right_speed = SPEED;
                          }
                      
                          /* set the motor speeds. */
                          wb_motor_set_velocity(motors[1], left_speed);
                          wb_motor_set_velocity(motors[0], right_speed);


                   }
             
              if (wb_receiver_get_queue_length(receiver) > 0) {
                const struct message *messpoint = wb_receiver_get_data(receiver);
                ox = (*messpoint).x;
                oz = (*messpoint).z;
                
                const double *direc = wb_receiver_get_emitter_direction(receiver);
                printf("orientation: \t%lf\t%lf\t%lf\t\t",direc[0],direc[1],direc[2]);
                wb_receiver_next_packet(receiver);
              }
               const double *centroid = wb_supervisor_node_get_center_of_mass(robot);
               double x = centroid[0];
               double z = centroid[2];
               dist = (x-ox)*(x-ox) + (z-oz)*(z-oz);
               dist = sqrt(dist);
               if(flaaag==false){
                 printf("COM:\t%lf\t%lf\t%lf\t\tdistance:\t%lf\n",x,centroid[1],z,dist);
               }
               }
               
               
               if(dist<=0.35){
                    wb_motor_set_position(motors[0],INFINITY);
                    wb_motor_set_position(motors[1],INFINITY);
                    wb_motor_set_velocity(motors[0],0);
                    wb_motor_set_velocity(motors[1],0);
                    flaaag=true;
               }
               
               
               
               
               
               
            if(flaaag==true)
            {
               emitter2 = wb_robot_get_device("emitter2");
               const int channel2 = wb_emitter_get_channel(emitter2);
               if (channel2 != 2) {
                 wb_emitter_set_channel(emitter2, 2);
               }
               
               
               double tempxold=0;
               double tempzold=0;
               const double *center = wb_supervisor_node_get_center_of_mass(robot);
               tempxold = center[0];
               tempzold = center[2];
               
               struct message messa2;
               const double *centroid = wb_supervisor_node_get_center_of_mass(robot);
               messa2.x=centroid[0];
               messa2.z= centroid[2];
               struct message *messpoint2 = &messa2;
               wb_emitter_send(emitter2, messpoint2, sizeof(messa2));
               
               // wb_robot_cleanup();
               
            }
               
         }
         

              
  }
  else if (strncmp(wb_robot_get_name(), "emitter_object", 14) == 0){
        root2=wb_supervisor_node_get_root();
        children2=wb_supervisor_node_get_field(root2, "children");
        robot2 = wb_supervisor_field_get_mf_node(children2, 7);
        

        
        emitter = wb_robot_get_device("emitter");
        const int channel = wb_emitter_get_channel(emitter);
        if (channel != COMMUNICATION_CHANNEL) {
          wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL);
        }
        
        
        receiver2 = wb_robot_get_device("receiver222");
        wb_receiver_enable(receiver2, TIME_STEP);
        
        while (wb_robot_step(TIME_STEP) != -1) {
           const double *centroid = wb_supervisor_node_get_center_of_mass(robot2);
           ox = centroid[0];
           oz = centroid[2];
           // printf("COM:\t%lf\t%lf\t%lf\n",ox,centroid[1],oz);
           messa.x = ox;
           messa.z = oz;
        
          struct message *messpoint = &messa;
          wb_emitter_send(emitter, messpoint, sizeof(messa));
          
          
          
          
          if (wb_receiver_get_queue_length(receiver2) > 0) {
            const struct message *messpoint2 = wb_receiver_get_data(receiver2);
            rx = (*messpoint2).x;
            rz = (*messpoint2).z;
            
            
            if(rx!=10){
              printf("robot_location:\t%lf\t%lf\n",rx,rz);
              WbNodeRef robot_node = wb_supervisor_node_get_from_def("object");
              WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
              const double INITIAL[3] = { rx, 0.07, rz };
              wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL);
              wb_supervisor_node_reset_physics(robot_node);
              messa.x=555;
              struct message *messpoint = &messa;
              wb_emitter_send(emitter, messpoint, sizeof(messa));
              // wb_robot_cleanup();
            }
            
            wb_receiver_next_packet(receiver2);
          }
        };

  }

  

  wb_robot_cleanup();

  return 0;
}
