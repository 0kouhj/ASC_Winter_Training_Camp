#include "MotorHandle.h"
#include "bsp_motor.h"
#include "bsp_imu.h"
#include "Control.h"
#include "bsp_encoder.h"
#include "Kalman.h"

void Handle(void){
	ICM_Update();
  Attitude_Update();  
	encoder_update();
	Control();
//	minimal_test();
}
