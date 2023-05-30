#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>
#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10

int main(int argc, char **argv) {

  int i=0;
  double ICX = 0;
  double ICY = 0;
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito=1.0, AceleradorEsquerdo=1.0;
  bool Robot_turn = false;
  bool Mov = true;
  double FCX = 0;
  double FCY = 0;
  int contador = 0;
  
  for(i=0;i<256;i++);
  wb_robot_init();
   
    printf("Default controller of the e-puck robot started...\n");          
    WbDeviceTag MotorEsquerdo, MotorDireito;

  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);

  wb_motor_set_velocity(MotorEsquerdo,0);
  wb_motor_set_velocity(MotorDireito,0);

   WbDeviceTag SensorProx[QtddSensoresProx];
  
   SensorProx[0] = wb_robot_get_device("ps0");
   SensorProx[1] = wb_robot_get_device("ps1");
   SensorProx[2] = wb_robot_get_device("ps2");
   SensorProx[3] = wb_robot_get_device("ps3");
   SensorProx[4] = wb_robot_get_device("ps4");
   SensorProx[5] = wb_robot_get_device("ps5");
   SensorProx[6] = wb_robot_get_device("ps6");
   SensorProx[7] = wb_robot_get_device("ps7");

   wb_distance_sensor_enable(SensorProx[0],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[1],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[2],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[3],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[4],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[5],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[6],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[7],TIME_STEP);

    WbDeviceTag Leds[QtddLeds];
    Leds[0] = wb_robot_get_device("led0");
    wb_led_set(Leds[0],-1);
    Leds[1] = wb_robot_get_device("led1");
    wb_led_set(Leds[1], -1);
    Leds[2] = wb_robot_get_device("led2");
    wb_led_set(Leds[2], -1);
    Leds[3] = wb_robot_get_device("led3");
    wb_led_set(Leds[3], -1);
    Leds[4] = wb_robot_get_device("led4");
    wb_led_set(Leds[4],-1);
    Leds[5] = wb_robot_get_device("led5");
    wb_led_set(Leds[5], -1);
    Leds[6] = wb_robot_get_device("led6");
    wb_led_set(Leds[6], -1);
    Leds[7] = wb_robot_get_device("led7");
    wb_led_set(Leds[7], -1);
    Leds[8] = wb_robot_get_device("led8");
    wb_led_set(Leds[8], -1);
    Leds[9] = wb_robot_get_device("led9");
    wb_led_set(Leds[9], -1);
    
     WbNodeRef WoodenBox = wb_supervisor_node_get_from_def("wooden_box");

  while (wb_robot_step(TIME_STEP) != -1) {

    for(i=0;i<256;i++);
    for(i=0;i<QtddSensoresProx;i++){
       LeituraSensorProx[i]= wb_distance_sensor_get_value(SensorProx[i]);
       if(LeituraSensorProx[0] >= 300 || LeituraSensorProx[1] >= 300 || LeituraSensorProx[6] >= 300 || LeituraSensorProx[7] >= 300){
           Robot_turn = true;
       }
    }

    wb_led_set(Leds[0], wb_led_get(Leds[0])*-1);
   
    if(Robot_turn == true){
      AceleradorDireito = -0.7;
      AceleradorEsquerdo = 1;
      Robot_turn = false;
      }
      
    else {
      AceleradorDireito = 1;
      AceleradorEsquerdo = 0.8;}

   if(Mov == true){
     const double *translation = wb_supervisor_node_get_position(WoodenBox);
     contador = contador + 1;
     ICX = translation[0];
     ICY = translation[1];
     
     if(contador == 5){
       Mov = false;
     }
   }

    wb_motor_set_velocity(MotorEsquerdo,6.28*AceleradorEsquerdo);
    wb_motor_set_velocity(MotorDireito, 6.28*AceleradorDireito);
   const double *translation2 = wb_supervisor_node_get_position(WoodenBox);
    FCX = translation2[0];
    FCY = translation2[1];
    
    if(ICX != FCX || ICY != FCY){
      
      wb_motor_set_velocity(MotorEsquerdo, 0);
      wb_motor_set_velocity(MotorDireito, 0);
      
      for (int i = 0; i < QtddLeds; i++) {
        wb_led_set(Leds[i], 1);
    }
      break;
    }
  };
  wb_robot_cleanup();
  return 0;
}
