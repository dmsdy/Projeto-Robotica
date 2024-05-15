#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>

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

  for (i = 0; i < 256; i++) texto[i] = '0';

  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);

  WbDeviceTag SensorProx[QtddSensoresProx];
  WbDeviceTag SensorProx[QtddSensoresProx];
  for (i = 0; i < QtddSensoresProx; i++) {
    SensorProx[i] = wb_robot_get_device("ps" + i);  // deve ser ajustado para concatenar corretamente
    wb_distance_sensor_enable(SensorProx[i], TIME_STEP);
  }

  WbDeviceTag Leds[QtddLeds];
  //Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0], -1);

  while (wb_robot_step(TIME_STEP) != -1) {
    for (i = 0; i < 256; i++) texto[i] = 0;

    for (i = 0; i < QtddSensoresProx; i++) {
      LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]) - 60;
      sprintf(texto, "%s|%d: %5.2f  ", texto, i, LeituraSensorProx[i]);
    }
    printf("%s\n", texto);
    //wb_led_set(Leds[0], wb_led_get(Leds[0]) * -1);

   
    if (virando > 0) {
      virando--;
      if (virando == 0) {
        AceleradorDireito = 1;
        AceleradorEsquerdo = 1;
      }
    } else {
     
      if (LeituraSensorProx[0] > 150 || LeituraSensorProx[1] > 150 || LeituraSensorProx[2] > 150) {
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
