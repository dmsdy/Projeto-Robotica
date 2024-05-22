#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h> 

#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10
#define MAX_CONTADOR_PARADO 20


int main(int argc, char **argv) {
  int i = 0;
  char texto[256];
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;
  int virando = 0;
  int direcao = 1;
  int contadorParado = 0;

  wb_robot_init();
  
  WbNodeRef caixa = wb_supervisor_node_get_from_def("CAIXA");
  
  const double *posicao_inicio = wb_supervisor_node_get_position(caixa);
  double posicao_inicio_x = posicao_inicio[0];
  double posicao_inicio_y = posicao_inicio[1];
  
  for (i = 0; i < 256; i++) texto[i] = '0';

  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);

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
  Leds[1] = wb_robot_get_device("led1");
  Leds[2] = wb_robot_get_device("led2");
  Leds[3] = wb_robot_get_device("led3");
  Leds[4] = wb_robot_get_device("led4");
  Leds[5] = wb_robot_get_device("led5");
  Leds[6] = wb_robot_get_device("led6");
  Leds[7] = wb_robot_get_device("led7");
  Leds[8] = wb_robot_get_device("led8");
  Leds[9] = wb_robot_get_device("led9");
  
  

  if (caixa == NULL) {
    printf("Erro: não foi possível encontrar a caixa.\n");
    wb_robot_cleanup();
    return -1;
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    for (i = 0; i < 256; i++) texto[i] = 0;

    for (i = 0; i < QtddSensoresProx; i++) {
      LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]) - 60;
      sprintf(texto, "%s|%d: %5.2f  ", texto, i, LeituraSensorProx[i]);
    }
    printf("%s\n", texto);
    //wb_led_set(Leds[0], wb_led_get(Leds[0]) * -1);
    
    const double *posicao = wb_supervisor_node_get_position(caixa);

    printf("Posição da caixa inicio: x=%f, y=%f\n", posicao_inicio_x, posicao_inicio_y);
    printf("Posição da caixa: x=%f, y=%f\n", posicao[0], posicao[1]);
    
    if (posicao[0] != posicao_inicio_x || posicao[1] != posicao_inicio_y) {
      
      for (i = 0; i < QtddLeds; i++) {
        wb_led_set(Leds[i], 1);
        
      }
     
     
      wb_motor_set_velocity(MotorEsquerdo, 0);
      wb_motor_set_velocity(MotorDireito, 0);
      printf("Caixa leve encontrada: x=%f, y=%f\n", posicao[0], posicao[1]);
      break;
    };
    
    
    if (virando > 0) {
      virando--;
      if (virando == 0) {
        AceleradorDireito = 1;
        AceleradorEsquerdo = 1;
      }
    } else {
     
      if (LeituraSensorProx[0] > 150 || LeituraSensorProx[7] > 150) {
        AceleradorDireito = -1;
        AceleradorEsquerdo = 1;
        virando = 10;
        direcao = -direcao;
        contadorParado = 0;
      } else {
        AceleradorDireito = 1;
        AceleradorEsquerdo = 1;
        contadorParado++;
      }


   
      if (contadorParado > MAX_CONTADOR_PARADO) {
        AceleradorDireito = -1 * direcao;
        AceleradorEsquerdo = 1 * direcao;
        virando = 10;
        contadorParado = 0;
        
      }
    }
    
    
    wb_motor_set_velocity(MotorEsquerdo, 6.28 * AceleradorEsquerdo);
    wb_motor_set_velocity(MotorDireito, 6.28 * AceleradorDireito);
  }

   
  wb_robot_cleanup();

  
  return 0;
}
