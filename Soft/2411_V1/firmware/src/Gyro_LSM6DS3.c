//--------------------------------------------------------
//	Gyro_LSM6DS3.c
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
#include "Mc32_I2cUtilCCS.h"

// définitions pour le gyroscope LSM6DS3
#define LSM6DS3_Addr 0x6A

#define CTRL2_G_Registre 0x11
//#define CTR1_XL_Registre 0x10 

#define Config_Hz 0x80 // mise a 104 Hz et 245 dps
#define Gyro_DPS 245
#define Time_delay 0.05

/*--------------------------------------------------------*/
// Fonction Write_Registre_LSM6DS3
/*--------------------------------------------------------*/
void Write_Registre_LSM6DS3(uint8_t Addr_Registre, uint8_t data)
{
    i2c_start();    //Lancer la communication en i2c
    
    i2c_write((LSM6DS3_Addr << 1) | 0);   //envoie de l'adresse et du bit d'écriture
    
    i2c_write(Addr_Registre);   //envoie l'adresse du registre
    
    i2c_write(data);    //envoie les data
    
    i2c_stop(); //stoper la communcation i2c
}

/*--------------------------------------------------------*/
// Fonction Read_Registre_LSM6DS3
/*--------------------------------------------------------*/
uint8_t Read_Registre_LSM6DS3(uint8_t Addr_Registre)
{
    uint8_t data;
    
    i2c_start();
    
    i2c_write((LSM6DS3_Addr << 1) | 0);   //envoie de l'adresse et du bit d'écriture
    
    i2c_write(Addr_Registre);
    
    i2c_reStart();
    
    i2c_write((LSM6DS3_Addr << 1) | 1);   //envoie de l'adresse et du bit d'écriture
    
    data = i2c_read(1); //Lecture des data avec ACK (acknowledge)
    
    i2c_read(0);
    
    i2c_stop();
    
    return data;
}

/*--------------------------------------------------------*/
// Fonction LSM6DS3_init
/*--------------------------------------------------------*/
void LSM6DS3_init()
{
    Write_Registre_LSM6DS3(CTRL2_G_Registre, Config_Hz);    //initialisation gyroscope en 104 Hz
    //Write_Registre_LSM6DS3(CTR1_XL_Registre, Config_Hz);  //initialisation accelerometre en 104 Hz
}

/*--------------------------------------------------------*/
// Fonction LSM6DS3_LectureGyroscope
/*--------------------------------------------------------*/
void LSM6DS3_LectureGyroscope(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t x_lsb = Read_Registre_LSM6DS3(0x22);
    uint8_t x_msb = Read_Registre_LSM6DS3(0x23);
    *ax = (x_msb << 8) | x_lsb;
    
    uint8_t y_lsb = Read_Registre_LSM6DS3(0x24);
    uint8_t y_msb = Read_Registre_LSM6DS3(0x25);
    *ay = (y_msb << 8) | y_lsb;
    
    uint8_t z_lsb = Read_Registre_LSM6DS3(0x26);
    uint8_t z_msb = Read_Registre_LSM6DS3(0x27);
    *az = (z_msb << 8) | z_lsb;
}

/*--------------------------------------------------------*/
// Fonction ConvertToAngle
/*--------------------------------------------------------*/
void ConvertToAngle(int16_t gx, int16_t gy, int16_t gz, float* angleX, float* angleY, float* angleZ)
{
    // Convertir les données brutes en dps
    float gyroX_dps = (gx / 32768.0) * Gyro_DPS;
    float gyroY_dps = (gy / 32768.0) * Gyro_DPS;
    float gyroZ_dps = (gz / 32768.0) * Gyro_DPS;

    // Intégration pour obtenir l'angle (en degrés)
    *angleX += gyroX_dps * Time_delay;  // Angle autour de l'axe X
    *angleY += gyroY_dps * Time_delay;  // Angle autour de l'axe Y
    *angleZ += gyroZ_dps * Time_delay;  // Angle autour de l'axe Z
}

/*--------------------------------------------------------*/
// Fonction LSM6DS3_Nouvelle_Data
/*--------------------------------------------------------*/
bool LSM6DS3_Nouvelle_Data()
{
    uint8_t NouvelleDonnee = Read_Registre_LSM6DS3(0x1E);
    
    return ((NouvelleDonnee >> 1) & 1);
}