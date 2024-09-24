#ifndef Gyro_LSM6DS3_H
#define Gyro_LSM6DS3_H

//--------------------------------------------------------
//	Gyro_LSM6DS3.h
//--------------------------------------------------------
//	Description :	Fonctions pour gyroscope
//                      ( en utilisant le LSM6DS3 )
//	Auteur 		: 	J. Shifteh
//      Date            :       23.09.2024
//	Version		:	V1.0
//	Compilateur	:	XC32 V1.31
// Modifications :
//
/*--------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

void Write_Registre_LSM6DS3(uint8_t Addr_Registre, uint8_t data);
uint8_t Read_Registre_LSM6DS3(uint8_t Addr_Registre);
void LSM6DS3_init();
void LSM6DS3_LectureGyroscope(int16_t *ax, int16_t *ay, int16_t *az);
void ConvertToAngle(int16_t gx, int16_t gy, int16_t gz, float* angleX, float* angleY, float* angleZ);
bool LSM6DS3_Nouvelle_Data();

#endif

 

