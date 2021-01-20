/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Container for gesture data */
typedef struct gesture_data_type {
    uint8_t u_data[32];
    uint8_t d_data[32];
    uint8_t l_data[32];
    uint8_t r_data[32];
    uint8_t index;
    uint8_t total_gestures;
    uint8_t in_threshold;
    uint8_t out_threshold;
} gesture_data_type;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Debug */
#define DEBUG                   0

/* APDS-9960 I2C address */
#define APDS9960_I2C_ADDR       0x39 << 1

/* Gesture parameters */
#define GESTURE_THRESHOLD_OUT   10
#define GESTURE_SENSITIVITY_1   50
#define GESTURE_SENSITIVITY_2   20

/* Error code for returned values */
#define ERROR                   0xFF

/* Acceptable device IDs */
#define APDS9960_ID_1           0xAB
#define APDS9960_ID_2           0x9C 

/* Misc parameters */
#define FIFO_PAUSE_TIME         30      // Wait period (ms) between FIFO reads

/* APDS-9960 register addresses */
#define APDS9960_ENABLE         0x80
#define APDS9960_ATIME          0x81
#define APDS9960_WTIME          0x83
#define APDS9960_AILTL          0x84
#define APDS9960_AILTH          0x85
#define APDS9960_AIHTL          0x86
#define APDS9960_AIHTH          0x87
#define APDS9960_PILT           0x89
#define APDS9960_PIHT           0x8B
#define APDS9960_PERS           0x8C
#define APDS9960_CONFIG1        0x8D
#define APDS9960_PPULSE         0x8E
#define APDS9960_CONTROL        0x8F
#define APDS9960_CONFIG2        0x90
#define APDS9960_ID             0x92
#define APDS9960_STATUS         0x93
#define APDS9960_CDATAL         0x94
#define APDS9960_CDATAH         0x95
#define APDS9960_RDATAL         0x96
#define APDS9960_RDATAH         0x97
#define APDS9960_GDATAL         0x98
#define APDS9960_GDATAH         0x99
#define APDS9960_BDATAL         0x9A
#define APDS9960_BDATAH         0x9B
#define APDS9960_PDATA          0x9C
#define APDS9960_POFFSET_UR     0x9D
#define APDS9960_POFFSET_DL     0x9E
#define APDS9960_CONFIG3        0x9F
#define APDS9960_GPENTH         0xA0
#define APDS9960_GEXTH          0xA1
#define APDS9960_GCONF1         0xA2
#define APDS9960_GCONF2         0xA3
#define APDS9960_GOFFSET_U      0xA4
#define APDS9960_GOFFSET_D      0xA5
#define APDS9960_GOFFSET_L      0xA7
#define APDS9960_GOFFSET_R      0xA9
#define APDS9960_GPULSE         0xA6
#define APDS9960_GCONF3         0xAA
#define APDS9960_GCONF4         0xAB
#define APDS9960_GFLVL          0xAE
#define APDS9960_GSTATUS        0xAF
#define APDS9960_IFORCE         0xE4
#define APDS9960_PICLEAR        0xE5
#define APDS9960_CICLEAR        0xE6
#define APDS9960_AICLEAR        0xE7
#define APDS9960_GFIFO_U        0xFC
#define APDS9960_GFIFO_D        0xFD
#define APDS9960_GFIFO_L        0xFE
#define APDS9960_GFIFO_R        0xFF

/* Bit fields */
#define APDS9960_PON            0x01
#define APDS9960_AEN            0x02
#define APDS9960_PEN            0x04
#define APDS9960_WEN            0x08
#define APSD9960_AIEN           0x10
#define APDS9960_PIEN           0x20
#define APDS9960_GEN            0x40
#define APDS9960_GVALID         0x01

/* On/Off definitions */
#define OFF                     0
#define ON                      1

/* Acceptable parameters for setMode */
#define POWER                   0
#define AMBIENT_LIGHT           1
#define PROXIMITY               2
#define WAIT                    3
#define AMBIENT_LIGHT_INT       4
#define PROXIMITY_INT           5
#define GESTURE                 6
#define ALL                     7

/* LED Drive values */
#define LED_DRIVE_100MA         0
#define LED_DRIVE_50MA          1
#define LED_DRIVE_25MA          2
#define LED_DRIVE_12_5MA        3

/* Proximity Gain (PGAIN) values */
#define PGAIN_1X                0
#define PGAIN_2X                1
#define PGAIN_4X                2
#define PGAIN_8X                3

/* ALS Gain (AGAIN) values */
#define AGAIN_1X                0
#define AGAIN_4X                1
#define AGAIN_16X               2
#define AGAIN_64X               3

/* Gesture Gain (GGAIN) values */
#define GGAIN_1X                0
#define GGAIN_2X                1
#define GGAIN_4X                2
#define GGAIN_8X                3

/* LED Boost values */
#define LED_BOOST_100           0
#define LED_BOOST_150           1
#define LED_BOOST_200           2
#define LED_BOOST_300           3    

/* Gesture wait time values */
#define GWTIME_0MS              0
#define GWTIME_2_8MS            1
#define GWTIME_5_6MS            2
#define GWTIME_8_4MS            3
#define GWTIME_14_0MS           4
#define GWTIME_22_4MS           5
#define GWTIME_30_8MS           6
#define GWTIME_39_2MS           7

/* Default values */
#define DEFAULT_ATIME           219     // 103ms
#define DEFAULT_WTIME           246     // 27ms
#define DEFAULT_PROX_PPULSE     0x87    // 16us, 8 pulses
#define DEFAULT_GESTURE_PPULSE  0x89    // 16us, 10 pulses
#define DEFAULT_POFFSET_UR      0       // 0 offset
#define DEFAULT_POFFSET_DL      0       // 0 offset      
#define DEFAULT_CONFIG1         0x60    // No 12x wait (WTIME) factor
#define DEFAULT_LDRIVE          LED_DRIVE_12_5MA  
#define DEFAULT_PGAIN           PGAIN_4X
#define DEFAULT_AGAIN           AGAIN_4X
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_AILT            0xFFFF  // Force interrupt for calibration
#define DEFAULT_AIHT            0
#define DEFAULT_PERS            0x11    // 2 consecutive prox or ALS for int.
#define DEFAULT_CONFIG2         0x01    // No saturation interrupts or LED boost  
#define DEFAULT_CONFIG3         0       // Enable all photodiodes, no SAI
#define DEFAULT_GPENTH          40      // Threshold for entering gesture mode
#define DEFAULT_GEXTH           30      // Threshold for exiting gesture mode    
#define DEFAULT_GCONF1          0x40    // 4 gesture events for int., 1 for exit
#define DEFAULT_GGAIN           GGAIN_4X
#define DEFAULT_GLDRIVE         LED_DRIVE_12_5MA  
#define DEFAULT_GWTIME          GWTIME_2_8MS
#define DEFAULT_GOFFSET         0       // No offset scaling for gesture mode
#define DEFAULT_GPULSE          0xC9    // 32us, 10 pulses
#define DEFAULT_GCONF3          0       // All photodiodes active during gesture
#define DEFAULT_GIEN            0       // Disable gesture interrupts

/* Direction definitions */
enum {
  DIR_NONE,
  DIR_LEFT,
  DIR_RIGHT,
  DIR_UP,
  DIR_DOWN,
  DIR_NEAR,
  DIR_FAR,
  DIR_ALL
};

/* State definitions */
enum {
  NA_STATE,
  NEAR_STATE,
  FAR_STATE,
  ALL_STATE
};


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
    gesture_data_type gesture_data_;
    int gesture_ud_delta_;
    int gesture_lr_delta_;
    int gesture_ud_count_;
    int gesture_lr_count_;
    int gesture_near_count_;
    int gesture_far_count_;
    int gesture_state_;
    int gesture_motion_;
		
		uint8_t i2ctransmitBuffer = 0;
    uint8_t i2creceiveBuffer = 0;
		uint8_t senddatabuf[20];
		uint8_t proximitydata;
	  uint16_t ambientdata,redsensdata,greensensdata,bluesensdata;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/*******************************************************************************
 * Raw I2C Reads and Writes
 ******************************************************************************/

/**
 * @brief Writes a single byte to the I2C device (no register)
 *
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool wireWriteByte(uint8_t val)
{
	
    HAL_I2C_Master_Transmit(&hi2c2,APDS9960_I2C_ADDR,&val,1,HAL_MAX_DELAY);  
    return true;
}


bool wireWriteDataByte(uint8_t reg, uint8_t val)
{
	  uint8_t sendatabuf[2] = {reg,val};
    HAL_I2C_Master_Transmit(&hi2c2,APDS9960_I2C_ADDR,sendatabuf,2,HAL_MAX_DELAY);
    return true;
}

bool wireWriteDataBlock(  uint8_t reg, 
                                        uint8_t *val, 
                                        unsigned int len)
{
		uint8_t* sendatabuf;
	
		for(int i = 0; i < len; i++) {
        sendatabuf[i + 1] = val[i];
    }
    HAL_I2C_Master_Transmit(&hi2c2,APDS9960_I2C_ADDR,sendatabuf,len + 1,HAL_MAX_DELAY);
    return true;
}

bool wireReadDataByte(uint8_t reg, uint8_t* val)
{
    
    /* Indicate which register we want to read from */
    if (!wireWriteByte(reg)) {
        return false;
    }
    
    /* Read from register */
		HAL_I2C_Master_Receive(&hi2c2,APDS9960_I2C_ADDR,val,1,HAL_MAX_DELAY);
    return true;
}


int wireReadDataBlock(   uint8_t reg, 
                                        uint8_t *val, 
                                        unsigned int len)
{
    unsigned char i = 0;
    
    /* Indicate which register we want to read from */
    if (!wireWriteByte(reg)) {
        return -1;
    }
    
    /* Read block data */
		HAL_I2C_Master_Receive(&hi2c2,APDS9960_I2C_ADDR,val,len,HAL_MAX_DELAY);
    return i;
}

uint8_t getMode()
{
    uint8_t enable_value;
    
    /* Read current ENABLE register */
    if( !wireReadDataByte(APDS9960_ENABLE, &enable_value) ) {
        return ERROR;
    }
    
    return enable_value;
}


/**
 * @brief Enables or disables a feature in t
he APDS-9960
 *
 * @param[in] mode which feature to enable
 * @param[in] enable ON (1) or OFF (0)
 * @return True if operation success. False otherwise.
 */
bool setMode(uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;

    /* Read current ENABLE register */
    reg_val = getMode();
    if( reg_val == ERROR ) {
        return false;
    }
    
    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if( mode >= 0 && mode <= 6 ) {
        if (enable) {
            reg_val |= (1 << mode);
        } else {
            reg_val &= ~(1 << mode);
        }
    } else if( mode == ALL ) {
        if (enable) {
            reg_val = 0x7F;
        } else {
            reg_val = 0x00;
        }
    }
        
    /* Write value back to ENABLE register */
    if( !wireWriteDataByte(APDS9960_ENABLE, reg_val) ) {
        return false;
    }
        
    return true;
}

/**
 * Turn the APDS-9960 on
 *
 * @return True if operation successful. False otherwise.
 */
bool enablePower()
{
    if( !setMode(POWER, 1) ) {
        return false;
    }
    
    return true;
}

/**
 * Turn the APDS-9960 off
 *
 * @return True if operation successful. False otherwise.
 */
bool disablePower()
{
    if( !setMode(POWER, 0) ) {
        return false;
    }
    
    return true;
} 

/**
 * @brief Turns proximity interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool setProximityIntEnable(uint8_t enable)
{
    uint8_t val;
    
    /* Read value from ENABLE register */
    if( !wireReadDataByte(APDS9960_ENABLE, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 5;
    val &= 0xDF;
    val |= enable;
    
    /* Write register value back into ENABLE register */
    if( !wireWriteDataByte(APDS9960_ENABLE, val) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Turns gesture-related interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool setGestureIntEnable(uint8_t enable)
{
    uint8_t val;
    
    /* Read value from GCONF4 register */
    if( !wireReadDataByte(APDS9960_GCONF4, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    enable &= 0X01;
    enable = enable << 1;
    val &= 0xFD;
    val |= enable;
    
    /* Write register value back into GCONF4 register */
    if( !wireWriteDataByte(APDS9960_GCONF4, val) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Tells the state machine to either enter or exit gesture state machine
 *
 * @param[in] mode 1 to enter gesture state machine, 0 to exit.
 * @return True if operation successful. False otherwise.
 */
bool setGestureMode(uint8_t mode)
{
    uint8_t val;
    
    /* Read value from GCONF4 register */
    if( !wireReadDataByte(APDS9960_GCONF4, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    mode &= 0x01;
    val &= 0xFE;
    val |= mode;
    
    /* Write register value back into GCONF4 register */
    if( !wireWriteDataByte(APDS9960_GCONF4, val) ) {
        return false;
    }
    
    return true;
}

void resetGestureParameters()
{
    gesture_data_.index = 0;
    gesture_data_.total_gestures = 0;
    
    gesture_ud_delta_ = 0;
    gesture_lr_delta_ = 0;
    
    gesture_ud_count_ = 0;
    gesture_lr_count_ = 0;
    
    gesture_near_count_ = 0;
    gesture_far_count_ = 0;
    
    gesture_state_ = 0;
    gesture_motion_ = DIR_NONE;
}

/**
 * @brief Turns ambient light interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool setAmbientLightIntEnable(uint8_t enable)
{
    uint8_t val;
    
    /* Read value from ENABLE register */
    if( !wireReadDataByte(APDS9960_ENABLE, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 4;
    val &= 0xEF;
    val |= enable;
    
    /* Write register value back into ENABLE register */
    if( !wireWriteDataByte(APDS9960_ENABLE, val) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Sets the LED current boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] drive the value (0-3) for current boost (100-300%)
 * @return True if operation successful. False otherwise.
 */
bool setLEDBoost(uint8_t boost)
{
    uint8_t val;
    
    /* Read value from CONFIG2 register */
    if( !wireReadDataByte(APDS9960_CONFIG2, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    boost &= 0x03;
    boost = boost << 4;
    val &= 0xCF;
    val |= boost;
    
    /* Write register value back into CONFIG2 register */
    if( !wireWriteDataByte(APDS9960_CONFIG2, val) ) {
        return false;
    }
    
    return true;
}


/**
 * @brief Sets the receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
bool setAmbientLightGain(uint8_t drive)
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9960_CONTROL, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    drive &= 0x03;
    val &= 0xFC;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9960_CONTROL, val) ) {
        return false;
    }
    
    return true;
}


/**
 * @brief Sets the receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
bool setProximityGain(uint8_t drive)
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9960_CONTROL, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 2;
    val &= 0xF3;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9960_CONTROL, val) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Sets the LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value (0-3) for the LED drive strength
 * @return True if operation successful. False otherwise.
 */
bool setLEDDrive(uint8_t drive)
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9960_CONTROL, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 6;
    val &= 0x3F;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9960_CONTROL, val) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Reads the proximity level as an 8-bit value
 *
 * @param[out] val value of the proximity sensor.
 * @return True if operation successful. False otherwise.
 */
bool readProximity(uint8_t* val)
{
    *val = 0;
    
    /* Read value from proximity data register */
    if( !wireReadDataByte(APDS9960_PDATA, val) ) {
        return false;
    }
    
    return true;
}



/**
 * @brief Determines swipe direction or near/far state
 *
 * @return True if near/far event. False otherwise.
 */
bool decodeGesture()
{
    /* Return if near or far event is detected */
    if( gesture_state_ == NEAR_STATE ) {
        gesture_motion_ = DIR_NEAR;
        return true;
    } else if ( gesture_state_ == FAR_STATE ) {
        gesture_motion_ = DIR_FAR;
        return true;
    }
    
    /* Determine swipe direction */
    if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 0) ) {
        gesture_motion_ = DIR_UP;
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 0) ) {
        gesture_motion_ = DIR_DOWN;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 1) ) {
        gesture_motion_ = DIR_RIGHT;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == -1) ) {
        gesture_motion_ = DIR_LEFT;
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_UP;
        } else {
            gesture_motion_ = DIR_RIGHT;
        }
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_DOWN;
        } else {
            gesture_motion_ = DIR_LEFT;
        }
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_UP;
        } else {
            gesture_motion_ = DIR_LEFT;
        }
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_DOWN;
        } else {
            gesture_motion_ = DIR_RIGHT;
        }
    } else {
        return false;
    }
    
    return true;
}

/*******************************************************************************
 * High-level gesture controls
 ******************************************************************************/

/**
 * @brief Resets all the parameters in the gesture data member
 */


/**
 * @brief Processes the raw gesture data to determine swipe direction
 *
 * @return True if near or far state seen. False otherwise.
 */
bool processGestureData()
{
    uint8_t u_first = 0;
    uint8_t d_first = 0;
    uint8_t l_first = 0;
    uint8_t r_first = 0;
    uint8_t u_last = 0;
    uint8_t d_last = 0;
    uint8_t l_last = 0;
    uint8_t r_last = 0;
    int ud_ratio_first;
    int lr_ratio_first;
    int ud_ratio_last;
    int lr_ratio_last;
    int ud_delta;
    int lr_delta;
    int i;

    /* If we have less than 4 total gestures, that's not enough */
    if( gesture_data_.total_gestures <= 4 ) {
        return false;
    }
    
    /* Check to make sure our data isn't out of bounds */
    if( (gesture_data_.total_gestures <= 32) &&
        (gesture_data_.total_gestures > 0) ) {
        
        /* Find the first value in U/D/L/R above the threshold */
        for( i = 0; i < gesture_data_.total_gestures; i++ ) {
            if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
                
                u_first = gesture_data_.u_data[i];
                d_first = gesture_data_.d_data[i];
                l_first = gesture_data_.l_data[i];
                r_first = gesture_data_.r_data[i];
                break;
            }
        }
        
        /* If one of the _first values is 0, then there is no good data */
        if( (u_first == 0) || (d_first == 0) ||
            (l_first == 0) || (r_first == 0) ) {
            
            return false;
        }
        /* Find the last value in U/D/L/R above the threshold */
        for( i = gesture_data_.total_gestures - 1; i >= 0; i-- ) {
#if DEBUG
            Serial.print(F("Finding last: "));
            Serial.print(F("U:"));
            Serial.print(gesture_data_.u_data[i]);
            Serial.print(F(" D:"));
            Serial.print(gesture_data_.d_data[i]);
            Serial.print(F(" L:"));
            Serial.print(gesture_data_.l_data[i]);
            Serial.print(F(" R:"));
            Serial.println(gesture_data_.r_data[i]);
#endif
            if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
                
                u_last = gesture_data_.u_data[i];
                d_last = gesture_data_.d_data[i];
                l_last = gesture_data_.l_data[i];
                r_last = gesture_data_.r_data[i];
                break;
            }
        }
    }
    
    /* Calculate the first vs. last ratio of up/down and left/right */
    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);
       
#if DEBUG
    Serial.print(F("Last Values: "));
    Serial.print(F("U:"));
    Serial.print(u_last);
    Serial.print(F(" D:"));
    Serial.print(d_last);
    Serial.print(F(" L:"));
    Serial.print(l_last);
    Serial.print(F(" R:"));
    Serial.println(r_last);

    Serial.print(F("Ratios: "));
    Serial.print(F("UD Fi: "));
    Serial.print(ud_ratio_first);
    Serial.print(F(" UD La: "));
    Serial.print(ud_ratio_last);
    Serial.print(F(" LR Fi: "));
    Serial.print(lr_ratio_first);
    Serial.print(F(" LR La: "));
    Serial.println(lr_ratio_last);
#endif
       
    /* Determine the difference between the first and last ratios */
    ud_delta = ud_ratio_last - ud_ratio_first;
    lr_delta = lr_ratio_last - lr_ratio_first;
    
#if DEBUG
    Serial.print("Deltas: ");
    Serial.print("UD: ");
    Serial.print(ud_delta);
    Serial.print(" LR: ");
    Serial.println(lr_delta);
#endif

    /* Accumulate the UD and LR delta values */
    gesture_ud_delta_ += ud_delta;
    gesture_lr_delta_ += lr_delta;
    
#if DEBUG
    Serial.print("Accumulations: ");
    Serial.print("UD: ");
    Serial.print(gesture_ud_delta_);
    Serial.print(" LR: ");
    Serial.println(gesture_lr_delta_);
#endif
    
    /* Determine U/D gesture */
    if( gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = 1;
    } else if( gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = -1;
    } else {
        gesture_ud_count_ = 0;
    }
    
    /* Determine L/R gesture */
    if( gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = 1;
    } else if( gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = -1;
    } else {
        gesture_lr_count_ = 0;
    }
    
    /* Determine Near/Far gesture */
    if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 0) ) {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) &&
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
            
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            } else if( (ud_delta != 0) || (lr_delta != 0) ) {
                gesture_far_count_++;
            }
            
            if( (gesture_near_count_ >= 10) && (gesture_far_count_ >= 2) ) {
                if( (ud_delta == 0) && (lr_delta == 0) ) {
                    gesture_state_ = NEAR_STATE;
                } else if( (ud_delta != 0) && (lr_delta != 0) ) {
                    gesture_state_ = FAR_STATE;
                }
                return true;
            }
        }
    } else {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) &&
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
                
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            }
            
            if( gesture_near_count_ >= 10 ) {
                gesture_ud_count_ = 0;
                gesture_lr_count_ = 0;
                gesture_ud_delta_ = 0;
                gesture_lr_delta_ = 0;
            }
        }
    }
    
#if DEBUG
    Serial.print("UD_CT: ");
    Serial.print(gesture_ud_count_);
    Serial.print(" LR_CT: ");
    Serial.print(gesture_lr_count_);
    Serial.print(" NEAR_CT: ");
    Serial.print(gesture_near_count_);
    Serial.print(" FAR_CT: ");
    Serial.println(gesture_far_count_);
    Serial.println("----------");
#endif
    
    return false;
}

/**
 * @brief Starts the proximity sensor on the APDS-9960
 *
 * @param[in] interrupts true to enable hardware external interrupt on proximity
 * @return True if sensor enabled correctly. False on error.
 */
bool enableProximitySensor(bool interrupts)
{
    /* Set default gain, LED, interrupts, enable power, and enable sensor */
    if( !setProximityGain(DEFAULT_PGAIN) ) {
        return false;
    }
    if( !setLEDDrive(DEFAULT_LDRIVE) ) {
        return false;
    }
    if( interrupts ) {
        if( !setProximityIntEnable(1) ) {
            return false;
        }
    } else {
        if( !setProximityIntEnable(0) ) {
            return false;
        }
    }
    if( !enablePower() ){
        return false;
    }
    if( !setMode(PROXIMITY, 1) ) {
        return false;
    }
    
    return true;
}



bool enableLightSensor(bool interrupts)
{
    
    /* Set default gain, interrupts, enable power, and enable sensor */
    if( !setAmbientLightGain(DEFAULT_AGAIN) ) {
        return false;
    }
    if( interrupts ) {
        if( !setAmbientLightIntEnable(1) ) {
            return false;
        }
    } else {
        if( !setAmbientLightIntEnable(0) ) {
            return false;
        }
    }
    if( !enablePower() ){
        return false;
    }
    if( !setMode(AMBIENT_LIGHT, 1) ) {
        return false;
    }
    
    return true;

}

/**
 * @brief Ends the light sensor on the APDS-9960
 *
 * @return True if sensor disabled correctly. False on error.
 */
bool disableLightSensor()
{
    if( !setAmbientLightIntEnable(0) ) {
        return false;
    }
    if( !setMode(AMBIENT_LIGHT, 0) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Ends the proximity sensor on the APDS-9960
 *
 * @return True if sensor disabled correctly. False on error.
 */
bool disableProximitySensor()
{
	if( !setProximityIntEnable(0) ) {
		return false;
	}
	if( !setMode(PROXIMITY, 0) ) {
		return false;
	}

	return true;
}

/**
 * @brief Starts the gesture recognition engine on the APDS-9960
 *
 * @param[in] interrupts true to enable hardware external interrupt on gesture
 * @return True if engine enabled correctly. False on error.
 */
bool enableGestureSensor(bool interrupts)
{
    
    /* Enable gesture mode
       Set ENABLE to 0 (power off)
       Set WTIME to 0xFF
       Set AUX to LED_BOOST_300
       Enable PON, WEN, PEN, GEN in ENABLE 
    */
    resetGestureParameters();
    if( !wireWriteDataByte(APDS9960_WTIME, 0xFF) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE) ) {
        return false;
    }
    if( !setLEDBoost(LED_BOOST_300) ) {
        return false;
    }
    if( interrupts ) {
        if( !setGestureIntEnable(1) ) {
            return false;
        }
    } else {
        if( !setGestureIntEnable(0) ) {
            return false;
        }
    }
    if( !setGestureMode(1) ) {
        return false;
    }
    if( !enablePower() ){
        return false;
    }
    if( !setMode(WAIT, 1) ) {
        return false;
    }
    if( !setMode(PROXIMITY, 1) ) {
        return false;
    }
    if( !setMode(GESTURE, 1) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Ends the gesture recognition engine on the APDS-9960
 *
 * @return True if engine disabled correctly. False on error.
 */
bool disableGestureSensor()
{
    resetGestureParameters();
    if( !setGestureIntEnable(0) ) {
        return false;
    }
    if( !setGestureMode(0) ) {
        return false;
    }
    if( !setMode(GESTURE, 0) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Determines if there is a gesture available for reading
 *
 * @return True if gesture available. False otherwise.
 */
bool isGestureAvailable()
{
    uint8_t val;
    
    /* Read value from GSTATUS register */
    if( !wireReadDataByte(APDS9960_GSTATUS, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out GVALID bit */
    val &= APDS9960_GVALID;
    
    /* Return true/false based on GVALID bit */
    if( val == 1) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Processes a gesture event and returns best guessed gesture
 *
 * @return Number corresponding to gesture. -1 on error.
 */
int readGesture()
{
    uint8_t fifo_level = 0;
    uint8_t bytes_read = 0;
    uint8_t fifo_data[128];
    uint8_t gstatus;
    int motion;
    int i;
    
    /* Make sure that power and gesture is on and data is valid */
    if( !isGestureAvailable() ) {
        return DIR_NONE;
    }
    
    /* Keep looping as long as gesture data is valid */
    while(1) {
    
        /* Wait some time to collect next batch of FIFO data */
        HAL_Delay(FIFO_PAUSE_TIME);
        /* Get the contents of the STATUS register. Is data still valid? */
        if( !wireReadDataByte(APDS9960_GSTATUS, &gstatus) ) {
            return ERROR;
        }
        
        /* If we have valid data, read in FIFO */
        if( (gstatus & APDS9960_GVALID) == APDS9960_GVALID ) {
        
            /* Read the current FIFO level */
            if( !wireReadDataByte(APDS9960_GFLVL, &fifo_level) ) {
                return ERROR;
            }


            /* If there's stuff in the FIFO, read it into our data block */
            if( fifo_level > 0) {
                bytes_read = wireReadDataBlock(  APDS9960_GFIFO_U, 
                                                (uint8_t*)fifo_data, 
                                                (fifo_level * 4) );
  

                /* If at least 1 set of data, sort the data into U/D/L/R */
                if( 1 ) {

                    for( i = 0; i < fifo_level * 4; i += 4 ) {
                        gesture_data_.u_data[gesture_data_.index] = 
                                                            fifo_data[i + 0];
                        gesture_data_.d_data[gesture_data_.index] = 
                                                            fifo_data[i + 1];
                        gesture_data_.l_data[gesture_data_.index] = 
                                                            fifo_data[i + 2];
                        gesture_data_.r_data[gesture_data_.index] = 
                                                            fifo_data[i + 3];
                        gesture_data_.index++;
                        gesture_data_.total_gestures++;
                    }


                    /* Filter and process gesture data. Decode near/far state */
                    if( processGestureData() ) {
                        if( decodeGesture() ) {
															motion = gesture_motion_;
															resetGestureParameters();
													return motion;
                        }
                    }
                    
                    /* Reset data */
                    gesture_data_.index = 0;
                    gesture_data_.total_gestures = 0;
                }
            }
        } else {
    
            /* Determine best guessed gesture and clean up */
					HAL_Delay(FIFO_PAUSE_TIME);
            decodeGesture();
            motion = gesture_motion_;

            resetGestureParameters();
            return motion;
        }
    }
}

/*******************************************************************************
 * Ambient light and color sensor controls
 ******************************************************************************/

/**
 * @brief Reads the ambient (clear) light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool readAmbientLight(uint16_t* val)
{
    uint8_t val_byte;
    *val = 0;
    
    /* Read value from clear channel, low byte register */
    if( !wireReadDataByte(APDS9960_CDATAL, &val_byte) ) {
        return false;
    }
    *val = val_byte;
    
    /* Read value from clear channel, high byte register */
    if( !wireReadDataByte(APDS9960_CDATAH, &val_byte) ) {
        return false;
    }
    *val = *val + ((uint16_t)val_byte << 8);
    
    return true;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool readRedLight(uint16_t* val)
{
    uint8_t val_byte;
    *val = 0;
    
    /* Read value from clear channel, low byte register */
    if( !wireReadDataByte(APDS9960_RDATAL, &val_byte) ) {
        return false;
    }
    *val = val_byte;
    
    /* Read value from clear channel, high byte register */
    if( !wireReadDataByte(APDS9960_RDATAH, &val_byte) ) {
        return false;
    }
    *val = *val + ((uint16_t)val_byte << 8);
    
    return true;
}
 
/**
 * @brief Reads the green light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool readGreenLight(uint16_t* val)
{
    uint8_t val_byte;
    *val = 0;
    
    /* Read value from clear channel, low byte register */
    if( !wireReadDataByte(APDS9960_GDATAL, &val_byte) ) {
        return false;
    }
    *val = val_byte;
    
    /* Read value from clear channel, high byte register */
    if( !wireReadDataByte(APDS9960_GDATAH, &val_byte) ) {
        return false;
    }
    *val = *val + ((uint16_t)val_byte << 8);
    
    return true;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool readBlueLight(uint16_t* val)
{
    uint8_t val_byte;
    *val = 0;
    
    /* Read value from clear channel, low byte register */
    if( !wireReadDataByte(APDS9960_BDATAL,&val_byte) ) {
        return false;
    }
    *val = val_byte;
    
    /* Read value from clear channel, high byte register */
    if( !wireReadDataByte(APDS9960_BDATAH, &val_byte) ) {
        return false;
    }
    *val = *val + ((uint16_t)val_byte << 8);
    
    return true;
}

/*******************************************************************************
 * Proximity sensor controls
 ******************************************************************************/


/*******************************************************************************
 * Getters and setters for register values
 ******************************************************************************/



/**
 * @brief Returns the lower threshold for proximity detection
 *
 * @return lower threshold
 */
uint8_t getProxIntLowThresh()
{
    uint8_t val;
    
    /* Read value from PILT register */
    if( !wireReadDataByte(APDS9960_PILT, &val) ) {
        val = 0;
    }
    
    return val;
}

/**
 * @brief Sets the lower threshold for proximity detection
 *
 * @param[in] threshold the lower proximity threshold
 * @return True if operation successful. False otherwise.
 */
bool setProxIntLowThresh(uint8_t threshold)
{
    if( !wireWriteDataByte(APDS9960_PILT, threshold) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Returns the high threshold for proximity detection
 *
 * @return high threshold
 */
uint8_t getProxIntHighThresh()
{
    uint8_t val;
    
    /* Read value from PIHT register */
    if( !wireReadDataByte(APDS9960_PIHT, &val) ) {
        val = 0;
    }
    
    return val;
}

/**
 * @brief Sets the high threshold for proximity detection
 *
 * @param[in] threshold the high proximity threshold
 * @return True if operation successful. False otherwise.
 */
bool setProxIntHighThresh(uint8_t threshold)
{
    if( !wireWriteDataByte(APDS9960_PIHT, threshold) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Returns LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the value of the LED drive strength. 0xFF on failure.
 */
uint8_t getLEDDrive()
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9960_CONTROL, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out LED drive bits */
    val = (val >> 6) & 0x03;
    
    return val;
}


/**
 * @brief Returns receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the value of the proximity gain. 0xFF on failure.
 */
uint8_t getProximityGain()
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9960_CONTROL, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out PDRIVE bits */
    val = (val >> 2) & 0x03;
    
    return val;
}



/**
 * @brief Returns receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @return the value of the ALS gain. 0xFF on failure.
 */
uint8_t getAmbientLightGain()
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9960_CONTROL, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out ADRIVE bits */
    val &= 0x03;
    
    return val;
}



/**
 * @brief Get the current LED boost value
 * 
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @return The LED boost value. 0xFF on failure.
 */
uint8_t getLEDBoost()
{
    uint8_t val;
    
    /* Read value from CONFIG2 register */
    if( !wireReadDataByte(APDS9960_CONFIG2, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out LED_BOOST bits */
    val = (val >> 4) & 0x03;
    
    return val;
}

    
   
/**
 * @brief Gets proximity gain compensation enable
 *
 * @return 1 if compensation is enabled. 0 if not. 0xFF on error.
 */
uint8_t getProxGainCompEnable()
{
    uint8_t val;
    
    /* Read value from CONFIG3 register */
    if( !wireReadDataByte(APDS9960_CONFIG3, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out PCMP bits */
    val = (val >> 5) & 0x01;
    
    return val;
}

/**
 * @brief Sets the proximity gain compensation enable
 *
 * @param[in] enable 1 to enable compensation. 0 to disable compensation.
 * @return True if operation successful. False otherwise.
 */
 bool setProxGainCompEnable(uint8_t enable)
{
    uint8_t val;
    
    /* Read value from CONFIG3 register */
    if( !wireReadDataByte(APDS9960_CONFIG3, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 5;
    val &= 0xDF;
    val |= enable;
    
    /* Write register value back into CONFIG3 register */
    if( !wireWriteDataByte(APDS9960_CONFIG3, val) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Gets the current mask for enabled/disabled proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @return Current proximity mask for photodiodes. 0xFF on error.
 */
uint8_t getProxPhotoMask()
{
    uint8_t val;
    
    /* Read value from CONFIG3 register */
    if( !wireReadDataByte(APDS9960_CONFIG3, &val) ) {
        return ERROR;
    }
    
    /* Mask out photodiode enable mask bits */
    val &= 0x0F;
    
    return val;
}

/**
 * @brief Sets the mask for enabling/disabling proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @param[in] mask 4-bit mask value
 * @return True if operation successful. False otherwise.
 */
bool setProxPhotoMask(uint8_t mask)
{
    uint8_t val;
    
    /* Read value from CONFIG3 register */
    if( !wireReadDataByte(APDS9960_CONFIG3, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    mask &= 0x0F;
    val &= 0xF0;
    val |= mask;
    
    /* Write register value back into CONFIG3 register */
    if( !wireWriteDataByte(APDS9960_CONFIG3, val) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Gets the entry proximity threshold for gesture sensing
 *
 * @return Current entry proximity threshold.
 */
uint8_t getGestureEnterThresh()
{
    uint8_t val;
    
    /* Read value from GPENTH register */
    if( !wireReadDataByte(APDS9960_GPENTH, &val) ) {
        val = 0;
    }
    
    return val;
}

/**
 * @brief Sets the entry proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to start gesture mode
 * @return True if operation successful. False otherwise.
 */
bool setGestureEnterThresh(uint8_t threshold)
{
    if( !wireWriteDataByte(APDS9960_GPENTH, threshold) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Gets the exit proximity threshold for gesture sensing
 *
 * @return Current exit proximity threshold.
 */
uint8_t getGestureExitThresh()
{
    uint8_t val;
    
    /* Read value from GEXTH register */
    if( !wireReadDataByte(APDS9960_GEXTH, &val) ) {
        val = 0;
    }
    
    return val;
}

/**
 * @brief Sets the exit proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to end gesture mode
 * @return True if operation successful. False otherwise.
 */
bool setGestureExitThresh(uint8_t threshold)
{
    if( !wireWriteDataByte(APDS9960_GEXTH, threshold) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Gets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the current photodiode gain. 0xFF on error.
 */
uint8_t getGestureGain()
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !wireReadDataByte(APDS9960_GCONF2, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out GGAIN bits */
    val = (val >> 5) & 0x03;
    
    return val;
}

/**
 * @brief Sets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] gain the value for the photodiode gain
 * @return True if operation successful. False otherwise.
 */
bool setGestureGain(uint8_t gain)
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !wireReadDataByte(APDS9960_GCONF2, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    gain &= 0x03;
    gain = gain << 5;
    val &= 0x9F;
    val |= gain;
    
    /* Write register value back into GCONF2 register */
    if( !wireWriteDataByte(APDS9960_GCONF2, val) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Gets the drive current of the LED during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the LED drive current value. 0xFF on error.
 */
uint8_t getGestureLEDDrive()
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !wireReadDataByte(APDS9960_GCONF2, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out GLDRIVE bits */
    val = (val >> 3) & 0x03;
    
    return val;
}

/**
 * @brief Sets the LED drive current during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value for the LED drive current
 * @return True if operation successful. False otherwise.
 */
bool setGestureLEDDrive(uint8_t drive)
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !wireReadDataByte(APDS9960_GCONF2, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 3;
    val &= 0xE7;
    val |= drive;
    
    /* Write register value back into GCONF2 register */
    if( !wireWriteDataByte(APDS9960_GCONF2, val) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Gets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @return the current wait time between gestures. 0xFF on error.
 */
uint8_t getGestureWaitTime()
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !wireReadDataByte(APDS9960_GCONF2, &val) ) {
        return ERROR;
    }
    
    /* Mask out GWTIME bits */
    val &= 0x07;
    
    return val;
}

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] the value for the wait time
 * @return True if operation successful. False otherwise.
 */
bool setGestureWaitTime(uint8_t time)
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !wireReadDataByte(APDS9960_GCONF2, &val) ) {
        return false;
    }
    
    /* Set bits in register to given value */
    time &= 0x07;
    val &= 0xF8;
    val |= time;
    
    /* Write register value back into GCONF2 register */
    if( !wireWriteDataByte(APDS9960_GCONF2, val) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Gets the low threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool getLightIntLowThreshold(uint16_t* threshold)
{
    uint8_t val_byte;
    *threshold = 0;
    
    /* Read value from ambient light low threshold, low byte register */
    if( !wireReadDataByte(APDS9960_AILTL, &val_byte) ) {
        return false;
    }
    *threshold = val_byte;
    
    /* Read value from ambient light low threshold, high byte register */
    if( !wireReadDataByte(APDS9960_AILTH, &val_byte) ) {
        return false;
    }
    *threshold = *threshold + ((uint16_t)val_byte << 8);
    
    return true;
}

/**
 * @brief Sets the low threshold for ambient light interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool setLightIntLowThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;
    
    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;
    
    /* Write low byte */
    if( !wireWriteDataByte(APDS9960_AILTL, val_low) ) {
        return false;
    }
    
    /* Write high byte */
    if( !wireWriteDataByte(APDS9960_AILTH, val_high) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Gets the high threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool getLightIntHighThreshold(uint16_t* threshold)
{
    uint8_t val_byte;
    *threshold = 0;
    
    /* Read value from ambient light high threshold, low byte register */
    if( !wireReadDataByte(APDS9960_AIHTL, &val_byte) ) {
        return false;
    }
    *threshold = val_byte;
    
    /* Read value from ambient light high threshold, high byte register */
    if( !wireReadDataByte(APDS9960_AIHTH, &val_byte) ) {
        return false;
    }
    *threshold = *threshold + ((uint16_t)val_byte << 8);
    
    return true;
}

/**
 * @brief Sets the high threshold for ambient light interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool setLightIntHighThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;
    
    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;
    
    /* Write low byte */
    if( !wireWriteDataByte(APDS9960_AIHTL, val_low) ) {
        return false;
    }
    
    /* Write high byte */
    if( !wireWriteDataByte(APDS9960_AIHTH, val_high) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Gets the low threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool getProximityIntLowThreshold(uint8_t* threshold)
{
    *threshold = 0;
    
    /* Read value from proximity low threshold register */
    if( !wireReadDataByte(APDS9960_PILT, threshold) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Sets the low threshold for proximity interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool setProximityIntLowThreshold(uint8_t threshold)
{
    
    /* Write threshold value to register */
    if( !wireWriteDataByte(APDS9960_PILT, threshold) ) {
        return false;
    }
    
    return true;
}
    
/**
 * @brief Gets the high threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool getProximityIntHighThreshold(uint8_t* threshold)
{
    *threshold = 0;
    
    /* Read value from proximity low threshold register */
    if( !wireReadDataByte(APDS9960_PIHT, threshold) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Sets the high threshold for proximity interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool setProximityIntHighThreshold(uint8_t threshold)
{
    
    /* Write threshold value to register */
    if( !wireWriteDataByte(APDS9960_PIHT, threshold) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Gets if ambient light interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getAmbientLightIntEnable()
{
    uint8_t val;
    
    /* Read value from ENABLE register */
    if( !wireReadDataByte(APDS9960_ENABLE, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out AIEN bit */
    val = (val >> 4) & 0x01;
    
    return val;
}



/**
 * @brief Gets if proximity interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getProximityIntEnable()
{
    uint8_t val;
    
    /* Read value from ENABLE register */
    if( !wireReadDataByte(APDS9960_ENABLE, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out PIEN bit */
    val = (val >> 5) & 0x01;
    
    return val;
}



/**
 * @brief Gets if gesture interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getGestureIntEnable()
{
    uint8_t val;
    
    /* Read value from GCONF4 register */
    if( !wireReadDataByte(APDS9960_GCONF4, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out GIEN bit */
    val = (val >> 1) & 0x01;
    
    return val;
}



/**
 * @brief Clears the ambient light interrupt
 *
 * @return True if operation completed successfully. False otherwise.
 */
bool clearAmbientLightInt()
{
    uint8_t throwaway;
    if( !wireReadDataByte(APDS9960_AICLEAR, &throwaway) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Clears the proximity interrupt
 *
 * @return True if operation completed successfully. False otherwise.
 */
bool clearProximityInt()
{
    uint8_t throwaway;
    if( !wireReadDataByte(APDS9960_PICLEAR, &throwaway) ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Tells if the gesture state machine is currently running
 *
 * @return 1 if gesture state machine is running, 0 if not. 0xFF on error.
 */
uint8_t getGestureMode()
{
    uint8_t val;
    
    /* Read value from GCONF4 register */
    if( !wireReadDataByte(APDS9960_GCONF4, &val) ) {
        return ERROR;
    }
    
    /* Mask out GMODE bit */
    val &= 0x01;
    
    return val;
}



/**
 * @brief Configures I2C communications and initializes registers to defaults
 *
 * @return True if initialized successfully. False otherwise.
 */
bool init()
{
    uint8_t id;
    /* Read ID register and check against known values for APDS-9960 */
    if( !wireReadDataByte(APDS9960_ID, &id) ) {
        return false;
    }

    if( !(id == APDS9960_ID_1 || id == APDS9960_ID_2) ) {

			//Sending in normal mode
        return false;
    }
     
    /* Set ENABLE register to 0 (disable all features) */
    if( !setMode(ALL, OFF) ) {
        return false;
    }
    
    /* Set default values for ambient light and proximity registers */
    if( !wireWriteDataByte(APDS9960_ATIME, DEFAULT_ATIME) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_WTIME, DEFAULT_WTIME) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_CONFIG1, DEFAULT_CONFIG1) ) {
        return false;
    }
    if( !setLEDDrive(DEFAULT_LDRIVE) ) {
        return false;
    }
    if( !setProximityGain(DEFAULT_PGAIN) ) {
        return false;
    }
    if( !setAmbientLightGain(DEFAULT_AGAIN) ) {
        return false;
    }
    if( !setProxIntLowThresh(DEFAULT_PILT) ) {
        return false;
    }
    if( !setProxIntHighThresh(DEFAULT_PIHT) ) {
        return false;
    }
    if( !setLightIntLowThreshold(DEFAULT_AILT) ) {
        return false;
    }
    if( !setLightIntHighThreshold(DEFAULT_AIHT) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_PERS, DEFAULT_PERS) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_CONFIG2, DEFAULT_CONFIG2) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_CONFIG3, DEFAULT_CONFIG3) ) {
        return false;
    }
    
    /* Set default values for gesture sense registers */
    if( !setGestureEnterThresh(DEFAULT_GPENTH) ) {
        return false;
    }
    if( !setGestureExitThresh(DEFAULT_GEXTH) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GCONF1, DEFAULT_GCONF1) ) {
        return false;
    }
    if( !setGestureGain(DEFAULT_GGAIN) ) {
        return false;
    }
    if( !setGestureLEDDrive(DEFAULT_GLDRIVE) ) {
        return false;
    }
    if( !setGestureWaitTime(DEFAULT_GWTIME) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GPULSE, DEFAULT_GPULSE) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GCONF3, DEFAULT_GCONF3) ) {
        return false;
    }
    if( !setGestureIntEnable(DEFAULT_GIEN) ) {
        return false;
    }
    
    return true;
}


const char *GetGestureName(uint8_t code)	{
	switch (code) {
		 
		case DIR_NONE:				return "NONE"; break;
		case DIR_LEFT:				return "LEFT"; break;
		case DIR_RIGHT:				return "RIGHT"; break;
		case DIR_UP:					return "UP"; break;
		case DIR_DOWN:				return "DOWN"; break;
		case DIR_NEAR:				return "NEAR"; break;
		case DIR_FAR:					return "FAR"; break;
		case DIR_ALL:					return "ALL"; break;
		default:              return "Unknown Movement";break;
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  //MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */;
  //HAL_GPIO_WritePin(VL_GPIO_Port,VL_Pin,GPIO_PIN_SET);
	init();
	HAL_Delay(100);
	init();
	HAL_Delay(100);
	enablePower();
	HAL_I2C_EnableListen_IT(&hi2c1);
	enableProximitySensor(0);
	//enableGestureSensor(0);
	enableLightSensor(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t direction, uint16_t addrMatchCode) { 
	if(hi2c -> Instance == I2C1)
	{
	switch (direction) {
		case I2C_DIRECTION_TRANSMIT:

			if (HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME) != HAL_OK) {
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME);
			}
		break;
 
		case I2C_DIRECTION_RECEIVE:

			readProximity(&proximitydata);
			senddatabuf[0] = proximitydata;
		  
		  
			readAmbientLight(&ambientdata);
			senddatabuf[10] = ambientdata & 0xff;
			senddatabuf[9] = (ambientdata >> 8);
		
			readRedLight(&redsensdata);
			senddatabuf[4] = redsensdata & 0xff;
			senddatabuf[3] = (redsensdata >> 8);
		
			readGreenLight(&greensensdata);
			senddatabuf[6] = greensensdata & 0xff;
			senddatabuf[5] = (greensensdata >> 8);
		
		
			readBlueLight(&bluesensdata);
			senddatabuf[8] = bluesensdata & 0xff;
			senddatabuf[7] = (bluesensdata >> 8);
			
			i2ctransmitBuffer = senddatabuf[i2creceiveBuffer];
			
			if (HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, &i2ctransmitBuffer, 1, I2C_LAST_FRAME) != HAL_OK) {
			HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, &i2ctransmitBuffer, 1, I2C_LAST_FRAME);
			}
		break;
	}
}
}
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) { 
	HAL_I2C_EnableListen_IT(hi2c); // Restart
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
