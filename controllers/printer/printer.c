/*
 * File:          printer.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define SPEED 0.05


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   WbDeviceTag m1 = wb_robot_get_device("m1");
   WbDeviceTag m2 = wb_robot_get_device("m2");
   WbDeviceTag m3 = wb_robot_get_device("m3");
   WbDeviceTag m4 = wb_robot_get_device("m4");
   WbDeviceTag m5 = wb_robot_get_device("m5");
   WbDeviceTag m6 = wb_robot_get_device("m6");
   WbDeviceTag m7 = wb_robot_get_device("m7");
   WbDeviceTag m8 = wb_robot_get_device("m8");
   
   wb_motor_set_velocity(m1, SPEED);
   wb_motor_set_velocity(m2, SPEED);
   wb_motor_set_velocity(m3, SPEED);
   wb_motor_set_velocity(m4, SPEED);
   wb_motor_set_velocity(m5, SPEED);
   wb_motor_set_velocity(m6, SPEED);
   wb_motor_set_velocity(m7, SPEED);
   wb_motor_set_velocity(m8, SPEED);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     wb_motor_set_position(m1, 0.20);
     wb_motor_set_position(m2, 0.20 * -1 + 0.9);
     wb_motor_set_position(m3, 0.15);
     wb_motor_set_position(m4, 0.20);
     wb_motor_set_position(m5, 0.25);
     wb_motor_set_position(m6, 0.25);
     wb_motor_set_position(m7, 0.25);
     wb_motor_set_position(m8, 0.25 * -1 + 0.9);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
