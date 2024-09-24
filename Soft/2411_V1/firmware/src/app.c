/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include "app.h"
#include "system_config.h"
#include "Mc32DriverAdc.h"
#include "GesFifoTh32.h"
#include "Gyro_LSM6DS3.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

//Utilisation de mes structure 
APP_DATA appData;
S_ADCResults Value_ADC;
Gest_Buffer Gestion_Trame;
S_fifo descrFifoRX;

//Création des variables globales utiles
static uint16_t Pot_R_Raw, Pot_L_Raw;

float last_motor_error_L = 0;
float proportional_L = 200;
float integral_L = 10;
float integrated_motor_error_L = 0;
float K_motor_L = 1.0;

float last_motor_error_2 = 0;
float proportional_2 = 200;
float integral_2 = 10;
float integrated_motor_error_2 = 0;
float K_motor_R = 1.0;

uint8_t fifoRX[16];          //Taille de la trame
uint8_t DeadZone = 100;     //Afin de reduire les vibrations du moteurs

//Création de mes constantes utiles
#define MOTOR_1_GAIN   50.0     
#define MOTOR_2_GAIN   50.0

#define Pot_Val_Min    136
#define Pot_Val_Max    888

#define Valeur_OC_Max  2399
#define Valeur_OC_Min  0
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
/* TODO:  Add any necessary local functions.
*/
void DataFromSimtools(void);
void FeedbackPot(void);
uint16_t ReMapping(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
int16_t Limite(int16_t value, int16_t min, int16_t max);
int16_t PIDupdateMoteurL(int16_t targetPosition, int16_t currentPosition);
int16_t PIDupdateMoteurR(int16_t targetPosition, int16_t currentPosition);
float Conversion_ADC_a_cm(float Valeur_ADC);
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /*
    static float Pot_L_U, Pot_R_U;
    static float Capt_1, Capt_2, Capt_3, Capt_4;
    static float Distance_Capt_1, Distance_Capt_2, Distance_Capt_3, Distance_Capt_4;
    
    int16_t gx, gy, gz;
    float angleX, angleY, angleZ;  // Variables pour stocker les angles
    */
    
    //Création de mes variables utiles dans le APP_Tasks
    static uint8_t indexReceive;
    
    uint8_t Axes1a, Axes1b;
    uint8_t Axes2a, Axes2b;
    uint8_t Start;
    uint8_t debut_L, debut_R;
    uint8_t End;
    uint16_t Axes1, Target_AxesL,  Axes2, Target_AxesR;   
    
    uint8_t flag_calibration;
    
    uint8_t New_PID_Moteur;
    uint16_t EnvoieOCL, EnvoieOCR;
    int16_t EnvoieMoteurL, EnvoieMoteurR;

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            
            //init adc
            BSP_InitADC10();
            //init I2C
           // i2c_init(0);
            //init LSM6DS3
           // LSM6DS3_init();
            //init Fifo
            InitFifo (&descrFifoRX, 16, fifoRX, 0);
            
            //initialisation de mes variables
            /*
            Capt_1 = 0;
            Capt_2 = 0;
            Capt_3 = 0;
            Capt_4 = 0;
            Distance_Capt_1 = 0;
            Distance_Capt_2 = 0;
            Distance_Capt_3 = 0;
            Distance_Capt_4 = 0;
            */
            Pot_R_Raw = 0;
            Pot_L_Raw = 0;
            
            Axes1a = 0;
            Axes1b = 0;
            Axes1 = 0;
            Target_AxesL = 512;
            Axes2a = 0;
            Axes2b = 0;
            Axes2 = 0;
            Target_AxesR = 512;
            Start = 0;
            debut_L = 0;
            debut_R = 0;
            End = 0;
            
            New_PID_Moteur = 0;
            EnvoieOCL = 0;
            EnvoieOCR = 0;
            EnvoieMoteurL = 0;
            EnvoieMoteurR = 0;
            
            indexReceive = 0;
            
            flag_calibration = 1;
            
            //init timer
            DRV_TMR0_Start();
            DRV_TMR1_Start();
            //init oc
            DRV_OC0_Start();
            DRV_OC1_Start();
            
            //passe a l'etat attente
            APP_UpdateState(APP_STATE_SERVICE_WAIT);
            break;
        }
        case APP_STATE_SERVICE_WAIT:
        {
            //rien faire
            break;
        }
        case APP_STATE_SERVICE_TASKS:
        {
            //Lecture de tout mes ADCs
            appData.ValAdc = BSP_ReadAllADC();
            
            //Utilisation de ma fonction FeedbackPot
            FeedbackPot();
            
            //Partie calibrage afin de mettre mes moteurs dans la zone de mes potentiomètres 
            while(((Pot_L_Raw < 500) || (Pot_L_Raw > 524)) || ((Pot_R_Raw < 500) || (Pot_R_Raw > 524)) && (flag_calibration == 1))
            {
                if(Pot_L_Raw < 500)
                {
                    //Désactivation de la PIN A1  afin de choisir le sens horaire
                    A1Off();
                    DRV_OC0_PulseWidthSet(800);
                }
                else if(Pot_L_Raw > 524)
                {
                    //Activation de la PIN A1 afin de choisir le sens anti-horaire
                    A1On();
                    DRV_OC0_PulseWidthSet(800);
                }
                else if((Pot_L_Raw >= Pot_Val_Min) && (Pot_L_Raw <= Pot_Val_Max))
                {
                    //Désactivation de la PIN A1
                    A1Off();
                    DRV_OC0_PulseWidthSet(0);
                }
                if(Pot_R_Raw < 500)
                {
                    //Désactivation de la PIN A2  afin de choisir le sens horaire
                    A2Off();
                    DRV_OC1_PulseWidthSet(800);
                }
                else if(Pot_R_Raw > 524)
                {
                    //Activation de la PIN A2 afin de choisir le sens anti-horaire
                    A2On();
                    DRV_OC1_PulseWidthSet(800);
                }
                else if((Pot_R_Raw >= Pot_Val_Min) && (Pot_R_Raw <= Pot_Val_Max))
                {
                    //Désactivation de la PIN A2
                    A2Off();
                    DRV_OC1_PulseWidthSet(0);
                }
                    
            }
            //Lorsque qu'ils sont bien calibré, il sort du mode de calibration
            if((Pot_L_Raw >= Pot_Val_Min) && (Pot_L_Raw <= Pot_Val_Max) && (Pot_R_Raw >= Pot_Val_Min) && (Pot_R_Raw <= Pot_Val_Max) && (flag_calibration == 1))
            {
                DRV_OC0_PulseWidthSet(0);
                DRV_OC1_PulseWidthSet(0);
                flag_calibration = 2;
            }
            
            //Lecture du Fifo rempli avec la trame envoyée depuis Simtools
            indexReceive = GetReadSize(&descrFifoRX);
            if(indexReceive >= 7)
            {
                GetCharFromFifo (&descrFifoRX, &Start);
                if(Start == 'X')
                {
                    GetCharFromFifo (&descrFifoRX, &debut_L);
                    GetCharFromFifo (&descrFifoRX, &Axes1a);
                    GetCharFromFifo (&descrFifoRX, &Axes1b);
                    GetCharFromFifo (&descrFifoRX, &debut_R);
                    GetCharFromFifo (&descrFifoRX, &Axes2a);
                    GetCharFromFifo (&descrFifoRX, &Axes2b);
                    GetCharFromFifo (&descrFifoRX, &End);
                }
            }
            //Vérification si la trame lue comportes les bons termes aux bons endroits
            if(Start == 'X')
            {
                if(debut_L == 'L')
                {
                    if(debut_R == 'R')
                    {
                        if(End == 'C')
                        {
                            //Si tout est okay activation du flag Set_Config
                            Gestion_Trame.Set_Config = 1;
                            Gestion_Trame.Error = 0;
                        }
                    }
                }
            }
            //Sinon, désactivation du flag Set_Config et activation du flag Error
            else
            {
                Gestion_Trame.Set_Config = 0;
                Gestion_Trame.Error = 1;
            }
            
            //Si le flag Error n'est pas actif
            if(Gestion_Trame.Error == 0)
            {
                LED_De_VieToggle();
                
                //Consignes lorsque on arrete la communication avec le jeu et simtools afin de recentrer les moteurs
                if((Axes1a == 255) && (Axes1b == 255) &&(Axes2a == 255) && (Axes2b == 255) && (Gestion_Trame.Set_Config ==1))
                {
                    Target_AxesL = 512; //Mise au point milieu de mes 2 axes
                    Target_AxesR = 512;
                    New_PID_Moteur = 1; //Activation du flag New_PID_Moteur
                }
                //Sinon, si on recois d'autres trames
                else if(Gestion_Trame.Set_Config == 1)
                {
                    //Mise en 1 varaible uint16_t les 2 bytes d'un axe
                    Axes1 = (Axes1a * 256) + Axes1b;
                    Axes2 = (Axes2a * 256) + Axes2b;
                    
                    //Transformation de cette variable en une range par rapport à la valeur min et max du potentiomètres
                    Target_AxesL = ReMapping(Axes1, 0, 1023, Pot_Val_Min, Pot_Val_Max);
                    Target_AxesR = ReMapping(Axes2, 0, 1023, Pot_Val_Min, Pot_Val_Max);
                    
                    //Limitation de cette variable afin d'être sur qu'elle ne dépasse pas la valeur souhaité
                    Limite(Target_AxesL, Pot_Val_Min, Pot_Val_Max);
                    Limite(Target_AxesR, Pot_Val_Min, Pot_Val_Max);
                    
                    New_PID_Moteur = 1; //Activation du flag New_PID_Moteur
                    Gestion_Trame.Set_Config = 0;   //désactivation du flag Set_Config
                }
            }
            
            //Si le flag New_PID_Moteur est activé
            if(New_PID_Moteur == 1)
            {
                New_PID_Moteur = 0; //Désactivation du flag New_PID_Moteur
                
                //Calcul et reprise de la valeur du PI affecté aux moteurs
                EnvoieMoteurL = PIDupdateMoteurL(Target_AxesL, Pot_L_Raw);
                EnvoieMoteurR = PIDupdateMoteurR(Target_AxesR, Pot_R_Raw);
                
                //Protection au cas ou le moteur se retrouve dans une range non-valide
                if((Pot_L_Raw < Pot_Val_Min) || (Pot_L_Raw > Pot_Val_Max) || (Pot_R_Raw < Pot_Val_Min) || (Pot_R_Raw > Pot_Val_Max))
                {
                    flag_calibration = 1;
                }
                //Ici, on choisi le sens de rotation avec A1 et A2 en fonction de ce que la trame de simtools me renvoie
                if((EnvoieMoteurL < 0) && ((abs(Target_AxesL - Pot_L_Raw) > DeadZone)) && (Pot_L_Raw > Pot_Val_Min) && (Pot_L_Raw < Pot_Val_Max))
                {
                    //Désactivation des PINs A1 et A2 afin de choisir le sens horaire
                    A1Off();
                    A2Off();
                    //Transformation de cette variable en une range par rapport à la valeur min et max de l'OC
                    EnvoieOCL = ReMapping(abs(EnvoieMoteurL), 0, 255, Valeur_OC_Min, Valeur_OC_Max);
                    EnvoieOCR = ReMapping(abs(EnvoieMoteurR), 0, 255, Valeur_OC_Min, Valeur_OC_Max);
                    //Envoie de ces valeurs aux OCs
                    DRV_OC0_PulseWidthSet(EnvoieOCL/1.5);
                    DRV_OC1_PulseWidthSet(EnvoieOCR/1.5);
                }
                else if((EnvoieMoteurL > 0) && (abs(Target_AxesL - Pot_L_Raw) > DeadZone) && (Pot_L_Raw > Pot_Val_Min) && (Pot_L_Raw < Pot_Val_Max))
                {
                    //Désactivation des PINs A1 et A2 afin de choisir le sens anti-horaire
                    A1On();
                    A2On();
                    //Transformation de cette variable en une range par rapport à la valeur min et max de l'OC
                    EnvoieOCL = ReMapping(EnvoieMoteurL, 0, 255, Valeur_OC_Min, Valeur_OC_Max);
                    EnvoieOCR = ReMapping(EnvoieMoteurR, 0, 255, Valeur_OC_Min, Valeur_OC_Max);
                    //Envoie de ces valeurs aux OCs
                    DRV_OC0_PulseWidthSet(EnvoieOCL/1.5);
                    DRV_OC1_PulseWidthSet(EnvoieOCR/1.5);
                }
                else
                {
                    //Désactivation des PINs A1 et A2
                    A1Off();
                    A2Off();
                    //Mise à 0 les valeurs aux OCs
                    DRV_OC0_PulseWidthSet(0);
                    DRV_OC1_PulseWidthSet(0);
                }
                
            }
            
            /*
            LSM6DS3_LectureGyroscope(&gx, &gy, &gz);    //lire les donnees brute du gyroscope
            if(LSM6DS3_Nouvelle_Data()) 
            {
                ConvertToAngle(gx, gy, gz, &angleX, &angleY, &angleZ);  // convertir les données en angle
            }
            //Mise en un signal 0-3.3V la valeur des potentiomètres
            Pot_L_U = Pot_L_Raw * (3.3/1023);
            Pot_R_U = Pot_R_Raw * (3.3/1023); 
            //Récolte des ADC pour les capteurs et mise en valeurs de tension entre 0-3.3V
            Capt_1 = appData.ValAdc.Chan5 * (3.3/1023);
            Capt_2 = appData.ValAdc.Chan4 * (3.3/1023);
            Capt_3 = appData.ValAdc.Chan3 * (3.3/1023); 
            Capt_4 = appData.ValAdc.Chan2 * (3.3/1023); 
            //Transformation de ces valeurs ADCs en cm
            Distance_Capt_1 = Conversion_ADC_a_cm(Capt_1);
            Distance_Capt_2 = Conversion_ADC_a_cm(Capt_2);
            Distance_Capt_3 = Conversion_ADC_a_cm(Capt_3);
            Distance_Capt_4 = Conversion_ADC_a_cm(Capt_4);
            */
            
            //passe a l'etat attente
            APP_UpdateState(APP_STATE_SERVICE_WAIT);
            break;
        }

        /* TODO: implement your application state machine.*/
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

void APP_UpdateState(APP_STATES NewState)
{
    appData.state = NewState;
} 
/*******************************************************************************
 End of File
 */
/*--------------------------------------------------------*/
// Fonction FeedbackPot
/*--------------------------------------------------------*/
//Création de la fonction Feedback afin de lire la valeur des potentiomètres 
void FeedbackPot(void)
{
  uint8_t i = 0;
  //Mise à 0 la valeur de mes potentiomètres
  Pot_R_Raw = 0;        
  Pot_L_Raw = 0;
  //Mise en place d'un boucle FOR afin de prendre 5 fois la valeur des potentiomètres
  for(i; i < 10; i++)
  {
    Pot_R_Raw += appData.ValAdc.Chan1;
    Pot_L_Raw += appData.ValAdc.Chan0;
  }
  //Divison de la variable par 5 afin d'avoir la moyenne de ces 5 valeurs par potentiomètres
  Pot_L_Raw /= 10;
  Pot_R_Raw /= 10;
}

/*--------------------------------------------------------*/
// Fonction ReMapping
/*--------------------------------------------------------*/
//Création de la fonction ReMapping afin de transformer une range en une autre range
uint16_t ReMapping(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) 
{
    //Soustraction de in_min à x
    uint32_t Result = (x - in_min);
    //Multiplication de Result et (out_max - out_min)
    Result = Result * (out_max - out_min);
    //Division de Result et (in_max - in_min)
    Result = Result / (in_max - in_min);
    //Addition de Result et out_min
    Result = Result + out_min;
    //On retourne la valeur de Result
    return Result;
}

/*--------------------------------------------------------*/
// Fonction Limite
/*--------------------------------------------------------*/
//Création de la fonction Limite afin de limiter une valeur entre le min et max voulu 
int16_t Limite(int16_t value, int16_t min, int16_t max) 
{
    //Si value est plus petit que min
    if (value < min) return min;
    //Sinon, si value est plus grand que max
    else if (value > max) return max;
    //Sinon on retourne la valeur de value
    else return value;   
}

/*--------------------------------------------------------*/
// Fonction PIDupdateMoteurL
/*--------------------------------------------------------*/
//Création de la fonction PID pour le moteur L
int16_t PIDupdateMoteurL(int16_t targetPosition, int16_t currentPosition)   
{
    //Calcul de l'erreur
    float error = targetPosition - currentPosition;
    //Calcul du terme proportionnel
    float pTerm_motor_L = (proportional_L * error);
    //Accumulation de l'erreur pour le terme intégral
    integrated_motor_error_L += error;
    //Fixer l'overflow de l'erreur de l'intégral
    integrated_motor_error_L = Limite(integrated_motor_error_L, -MOTOR_1_GAIN, MOTOR_1_GAIN);
    //Calcul du terme intégral avec une limite
    float iTerm_motor_L = (integral_L * integrated_motor_error_L);
    
    //Calcul de la sortie du PID
    float output_PID = K_motor_L * (pTerm_motor_L + iTerm_motor_L);
    return Limite(output_PID, -255, 255);
}

/*--------------------------------------------------------*/
// Fonction PIDupdateMoteurR
/*--------------------------------------------------------*/
//Création de la fonction PID pour le moteur R
int16_t PIDupdateMoteurR(int16_t targetPosition, int16_t currentPosition)   
{
    //Calcul de l'erreur
    float error = (float)targetPosition - (float)currentPosition;
    //Calcul du terme proportionnel
    float pTerm_motor_R = proportional_2 * error;
    //Accumulation de l'erreur pour le terme intégral
    integrated_motor_error_2 += error;
    //Fixer l'overflow de l'erreur de l'intégral
    integrated_motor_error_2 = Limite(integrated_motor_error_2, -MOTOR_2_GAIN, MOTOR_2_GAIN);
    //Calcul du terme intégral
    float iTerm_motor_R = (integral_2 * integrated_motor_error_2);

    float output_PID = K_motor_R * (pTerm_motor_R + iTerm_motor_R);
    return Limite(output_PID, -255, 255);
}

/*--------------------------------------------------------*/
// Fonction Conversion_ADC_a_cm
/*--------------------------------------------------------*/
//Création de la fonction Conversion_ADC_a_cm afin récolter les cm mesuré sur mes capteurs
float Conversion_ADC_a_cm(float Valeur_ADC)
{
    if(Valeur_ADC > 2.3)
    {
        return 0;
    }
    else if((Valeur_ADC >= 1.3) && (Valeur_ADC <= 2.3))
    {
        return (-10 * Valeur_ADC) + 33;
    }
    else if((Valeur_ADC > 0.72) && (Valeur_ADC < 1.3))
    {
        return (-34.48 * Valeur_ADC) + 64.82;
    }
    else if((Valeur_ADC >= 0.4) && (Valeur_ADC <= 0.72))
    {
        return (-108.11 * Valeur_ADC) + 117.84;
    }
    else
    {
        return 0;
    }
}
