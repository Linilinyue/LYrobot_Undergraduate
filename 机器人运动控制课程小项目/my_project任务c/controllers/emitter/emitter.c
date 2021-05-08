#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <stdio.h>
#include <string.h>
#define COMMUNICATION_CHANNEL 1


#define TIME_STEP 32


int main(int argc, char **argv) {
  WbDeviceTag emitter;

  wb_robot_init();
  
  emitter = wb_robot_get_device("emitter");

  const int channel = wb_emitter_get_channel(communication);
  if (channel != COMMUNICATION_CHANNEL) {
    wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL);
  }
  
  while (wb_robot_step(TIME_STEP) != -1) {

  };

  wb_robot_cleanup();

  return 0;
}
