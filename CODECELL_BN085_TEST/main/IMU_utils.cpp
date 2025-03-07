#pragma once
#include <IMU_utils.h>
#include <BNO085.h>




bool IMU_Init(class BNO085 IMU)
{

	
   Wire.begin(8,9,400000);
   if (!IMU.begin(Wire)) {
        Serial.printf("Error: No se pudo inicializar el BNO085 \n");
		  return 0;
    }
	 else{
    	Serial.println("BNO085 inicializado correctamente");
	 	return 1;
	 }

	 //_i2c_write_size = 0;
    Wire.endTransmission();
}

