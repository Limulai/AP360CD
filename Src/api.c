#include "api.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
//#include <stdin.h>

static uint32_t FilterAdcBuff(uint32_t *buff);



volatile static _deviceState deviceState;
_deviceStatus GetGlobalDeviceStatus(void) {
    return deviceState.deviceStatus;
}
void SetGlobalDeviceStatus(_deviceStatus x) {
    deviceState.deviceStatus = x;
}

_fanLevel GetGlobalFanlevel(void) {
    return deviceState.fanLever;
}
void SetGlobalFanlevel(_fanLevel x) {
    deviceState.fanLever = x;
}

unsigned char GetGlobalBlinkBits(void) {
    return deviceState.blinkBits;
}
void SetGlobalBlinkBits(unsigned char x) {
    deviceState.blinkBits = x;
}

_bool GetGlobalSettingMode(void) {
    return deviceState.settingMode;
}
void SetGlobalSettingMode(_bool x) {
    deviceState.settingMode = x;
}

_bool GetGlobalTime2Clean(void) {
    return deviceState.clean.time2clean;
}
void SetGlobalTime2Clean(_bool x) {
    deviceState.clean.time2clean = x;
}

void GetGlobalCleanAcc(unsigned char* x) {
    *x =  deviceState.clean.cleanAcc[0];
    *(x+1) = deviceState.clean.cleanAcc[1];

}
void SetGlobalCleanAcc(unsigned char* x) {
    deviceState.clean.cleanAcc[0] = *x;
    deviceState.clean.cleanAcc[1] = *(x+1);
}

_bool GetGlobalTimerEnable(void) {
    return deviceState.timer.enable;
}
void SetGlobalTimerEnable(_bool x) {
    deviceState.timer.enable = x;
}

_timerValue GetGlobalTimerValue(void) {
    return deviceState.timer.timerValue;
}
void SetGlobalTimerValue(_timerValue x) {
    deviceState.timer.timerValue = x;
}

_bool GetGlobalFaultEvent(void) {
    return deviceState.gotFaultevent;
}
void SetGlobalFaultEvent(_bool x) {
    deviceState.gotFaultevent = x;
}

_bool GetGlobalFaultEnable(void) {
    return deviceState.faultEnable;
}
void SetGlobalFaultEnable(_bool x) {
    deviceState.faultEnable = x;
}

_bool GetGlobalHvflag(void) {
    return deviceState.hvFlag;
}
void SetGlobalHvflag(_bool x) {
    deviceState.hvFlag = x;
}

_bool GetGlobalFaultEnableFromEEPROM(void) {
    return deviceState.faultFlagFromEEPROM;
}
void SetGlobalFaultEnableFromEEMROM(_bool x) {
    deviceState.faultFlagFromEEPROM = x;
}

/************  fan object  **********************************/
extern TIM_HandleTypeDef htim3;
void FanPWMStop(void){
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}
void FanPWMStart(void){
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}
unsigned char SetFanPWM(unsigned char percent) {
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (percent <= 100) {
        sConfigOC.Pulse = percent;
        HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
        FanPWMStart();
        return 1;
    }
    else
        return 0;
}

static unsigned char fan_percent_last = 0;
static unsigned char fan_percent_current = 0;
unsigned char GetFanPercentLast(void){
    return fan_percent_last;
}
unsigned char GetFanPercentCurrent(void){
    return fan_percent_current;
}
void SetFanPercentLast(unsigned char value){
    fan_percent_last = value;
}
void SetFanPercentCurrent(unsigned char value){
    fan_percent_current = value;
}

extern osTimerId adjFanGradullyTimerHandle;
unsigned char SetFanPWMGradually(unsigned char value) {
    fan_percent_current = value;
    // start the timer 
    osTimerStart(adjFanGradullyTimerHandle, 300);
	
		return 0;
}

/*
本来level_silent 默认无处理内容
修改后 处理内容为SetFanPWMGradually(FAN_STOP_PERCENT)
*/
void SetFanLevel(void){
    switch (deviceState.fanLever){
        case LEVEL_SILENT:
					   SetFanPWMGradually(FAN_STOP_PERCENT);
        break;
        case LEVEL_LOW:
            SetFanPWMGradually(FAN_LOW_PERCENT);
        break;
        case LEVEL_MID:
            SetFanPWMGradually(FAN_MID_PERCENT);
        break;
        case LEVEL_HIGH:
            SetFanPWMGradually(FAN_HIGH_PERCENT);
        break;
        case LEVEL_TURBO:
            SetFanPWMGradually(FAN_TURBO_PERCENT);
        break;
        default:
        break;
    }
}
void StopFan(void) {
    //FanPWMStop();
    SetFanPWMGradually(0);
}
void ResumeFan(void) {
    SetFanLevel();
    //FanPWMStart();
}


/************  High Volt  **********************************/
extern TIM_HandleTypeDef htim4;

//unsigned char GetAlarmVolt(void) {}
void TurnOffHighVolt(void){
    HAL_GPIO_WritePin(HV_ONOFF_GPIO_Port, HV_ONOFF_Pin, GPIO_PIN_RESET);
    deviceState.hvFlag = FALSE;
    SetFaultEnable(FALSE);
}
void TurnOnHighVolt(void){
    HAL_GPIO_WritePin(HV_ONOFF_GPIO_Port, HV_ONOFF_Pin, GPIO_PIN_SET);
    deviceState.hvFlag = TRUE;
    SetFaultEnable(TRUE);
}
unsigned char SetHighVolt(unsigned char percent){
    TIM_OC_InitTypeDef sConfigOC;
    if (percent <= 100) {
        sConfigOC.Pulse = percent;
        HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
        return 1;
    }
    else
        return 0;
}

extern osTimerId hvDelayTimerHandle;
void SetFaultEnable(_bool x){
    if (x == TRUE) {
            // delay 3s to let high volt be stable
        osTimerStart(hvDelayTimerHandle, 3000);
        //if (deviceState.hvFlag == TRUE) {
            //after 2s, enabel fault detect. deviceState.faultEnable = TRUE;
        //}
    }
    else {
        osTimerStop(hvDelayTimerHandle);
        deviceState.faultEnable = FALSE;
    }
}

/**************   EEPROM object *************************/
extern I2C_HandleTypeDef hi2c1;
void GetCleanAccFromEEPROM(unsigned char* v) {
    //unsigned short returnvalue;
    HAL_I2C_Mem_Read(&hi2c1, (unsigned short)ADDR_EEPROM_READ, 
                    (unsigned short)WORD_ADDR_CLEAN_ACC_HIGH8BIT, 2,
                    v, 2, 10);
}
void SetCleanAcc2EEPROM(unsigned char * clean_acc_value) {
    HAL_I2C_Mem_Write(&hi2c1, (unsigned short)ADDR_EEPROM_WRITE, 
                    (unsigned short)WORD_ADDR_CLEAN_ACC_HIGH8BIT, 2, 
                    clean_acc_value, 2 ,10);
}
void ClearCleanAcc(void){
    unsigned char v[2] = {0, 0};
    SetCleanAcc2EEPROM(v);
}

void SetFaultEnableFlag2EEPROM(_bool x) {
    unsigned char v;
    if(x == TRUE) {
        v = 1;
        HAL_I2C_Mem_Write(&hi2c1, (unsigned short)ADDR_EEPROM_WRITE, 
                    (unsigned short)WORD_ADDR_FAULT_ENABLE_FLAG, 1,
                    &v, 1, 10);
    }
    else if (x == FALSE) {
        v = 0;
        HAL_I2C_Mem_Write(&hi2c1, (unsigned short)ADDR_EEPROM_WRITE, 
                    (unsigned short)WORD_ADDR_FAULT_ENABLE_FLAG, 1,
                    &v, 1, 10);
    }
}

_bool GetFaultEnableFlagFromEEPROM(void) {
    unsigned char v;
    HAL_I2C_Mem_Read(&hi2c1, (unsigned short)ADDR_EEPROM_READ, 
                    (unsigned short)WORD_ADDR_FAULT_ENABLE_FLAG, 1,
                    &v, 1, 10);
    if (v == 1) {
        return TRUE;
    }
    else if (v == 0) {
        return FALSE;
    }
		else 
			return FALSE;
}

void ToggleFaultEnableFlag(void){
    if (GetFaultEnableFlagFromEEPROM()){
        SetFaultEnableFlag2EEPROM(FALSE);
    }
    else {
        SetFaultEnableFlag2EEPROM(TRUE);
    }
}

void FreshenFaultEnableFlagFromEEPROM(void) {
    //_bool v;
    deviceState.faultFlagFromEEPROM = GetFaultEnableFlagFromEEPROM();
}

_bool GetTurboTestFlag(void){
    unsigned char v;
    HAL_I2C_Mem_Read(&hi2c1, (unsigned short)ADDR_EEPROM_READ, 
                    (unsigned short)WORD_ADDR_TURBO_TEST_FLAG, 1,
                    &v, 1, 10);
    if (v == 1) {
        return TRUE;
    }
    else if (v == 0) {
        return FALSE;
    }
		else 
			return FALSE;
}

void SetTurboTestFlag(_bool x){
    unsigned char v;
    if(x == TRUE) {
        v = 1;
        HAL_I2C_Mem_Write(&hi2c1, (unsigned short)ADDR_EEPROM_WRITE, 
                    (unsigned short)WORD_ADDR_TURBO_TEST_FLAG, 1,
                    &v, 1, 10);
    }
    else if (x == FALSE) {
        v = 0;
        HAL_I2C_Mem_Write(&hi2c1, (unsigned short)ADDR_EEPROM_WRITE, 
                    (unsigned short)WORD_ADDR_TURBO_TEST_FLAG, 1,
                    &v, 1, 10);
    }
}

void ToggleTurboTestFlag(void){
    if (GetTurboTestFlag()){
        SetTurboTestFlag(FALSE);
    }
    else {
        SetTurboTestFlag(TRUE);
    }
}

/**************   adc  *************************/




/******************************** led buttons *************/
void cleanLedOn(void){
    HAL_GPIO_WritePin(LED_CLEAN_GPIO_Port, LED_CLEAN_Pin, GPIO_PIN_RESET);  
}
void cleanLedOff(void) {
    HAL_GPIO_WritePin(LED_CLEAN_GPIO_Port, LED_CLEAN_Pin, GPIO_PIN_SET); 
}

void powerLedOn(void) {
    HAL_GPIO_WritePin(LED_POWER_GREEN_GPIO_Port, LED_POWER_GREEN_Pin, GPIO_PIN_RESET);
}

void powerLedOff(void) {
    HAL_GPIO_WritePin(LED_POWER_GREEN_GPIO_Port, LED_POWER_GREEN_Pin, GPIO_PIN_SET);
}

void plusTimerLedOn(void) {
    HAL_GPIO_WritePin(LED_TIMER_PLUS_GPIO_Port, LED_TIMER_PLUS_Pin, GPIO_PIN_RESET);
}

void plusTimerLedOff(void) {
    HAL_GPIO_WritePin(LED_TIMER_PLUS_GPIO_Port, LED_TIMER_PLUS_Pin, GPIO_PIN_SET);
}

void timerLedOn(void) {
    HAL_GPIO_WritePin(LED_TIMER_GPIO_Port, LED_TIMER_Pin, GPIO_PIN_RESET);
}

void timerLedOff(void) {
    HAL_GPIO_WritePin(LED_TIMER_GPIO_Port, LED_TIMER_Pin, GPIO_PIN_SET);
}
void lowLedOn(void) {
    HAL_GPIO_WritePin(LED_LOW_GPIO_Port, LED_LOW_Pin, GPIO_PIN_RESET);
}
void lowLedOff(void){
    HAL_GPIO_WritePin(LED_LOW_GPIO_Port, LED_LOW_Pin, GPIO_PIN_SET);
}

void midLedOn(void){
    HAL_GPIO_WritePin(LED_MID_GPIO_Port,LED_MID_Pin, GPIO_PIN_RESET);
}
void midLedOff(void){
    HAL_GPIO_WritePin(LED_MID_GPIO_Port,LED_MID_Pin, GPIO_PIN_SET);
}
void highLedOn(void){
    HAL_GPIO_WritePin(LED_HIGH_GPIO_Port,LED_HIGH_Pin, GPIO_PIN_RESET);
}

void highLedOff(void){
    HAL_GPIO_WritePin(LED_HIGH_GPIO_Port, LED_HIGH_Pin, GPIO_PIN_SET);
}

void turboLedOn(void){
    HAL_GPIO_WritePin(LED_TURBO_GPIO_Port, LED_TURBO_Pin, GPIO_PIN_RESET);
}
void turboLedOff(void){
    HAL_GPIO_WritePin(LED_TURBO_GPIO_Port, LED_TURBO_Pin, GPIO_PIN_SET);
}

void fanLedSet(unsigned char i){
    switch (i) {
        case 0:
            HAL_GPIO_WritePin(LED_LOW_GPIO_Port, LED_LOW_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_MID_GPIO_Port, LED_MID_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_HIGH_GPIO_Port, LED_HIGH_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_TURBO_GPIO_Port, LED_TURBO_Pin, GPIO_PIN_SET);
        break;
        case 1:
            HAL_GPIO_WritePin(LED_LOW_GPIO_Port, LED_LOW_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_MID_GPIO_Port, LED_MID_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_HIGH_GPIO_Port, LED_HIGH_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_TURBO_GPIO_Port, LED_TURBO_Pin, GPIO_PIN_SET);
        break;
        case 2:
            HAL_GPIO_WritePin(LED_LOW_GPIO_Port, LED_LOW_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_MID_GPIO_Port, LED_MID_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_HIGH_GPIO_Port, LED_HIGH_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_TURBO_GPIO_Port, LED_TURBO_Pin, GPIO_PIN_SET);
        break;
        case 3:
            HAL_GPIO_WritePin(LED_LOW_GPIO_Port, LED_LOW_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_MID_GPIO_Port, LED_MID_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_HIGH_GPIO_Port, LED_HIGH_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_TURBO_GPIO_Port, LED_TURBO_Pin, GPIO_PIN_SET);
        break;
        case 4:
            HAL_GPIO_WritePin(LED_LOW_GPIO_Port, LED_LOW_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_MID_GPIO_Port, LED_MID_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_HIGH_GPIO_Port, LED_HIGH_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_TURBO_GPIO_Port, LED_TURBO_Pin, GPIO_PIN_RESET);
        break;
        default:

        break;
    }
}


/***********  blink functions  ************/
void timerSettingStartBlink(unsigned char timervalue){
    // 1, clear all timer level led
    deviceState.blinkBits &= ~0xf0;
    // 2, set tiemr level led
    switch (timervalue) {
        case 1:
            deviceState.blinkBits |= led_low_blink_bit;
        break;
        case 2:
            deviceState.blinkBits |= led_low_blink_bit + led_mid_blink_bit;
        break;
        case 3:
            deviceState.blinkBits |= led_low_blink_bit + led_mid_blink_bit + led_high_blink_bit;
        break;
        case 4:
            deviceState.blinkBits |= led_low_blink_bit + led_mid_blink_bit + led_high_blink_bit + led_turbo_blink_bit;
        break;
        default:
        break;
    }
    
    deviceState.blinkBits |= led_timer_plus_blink_bit;
}

void timerSettingStopBlink(void) {
    deviceState.blinkBits &= ~(led_low_blink_bit + led_mid_blink_bit + led_high_blink_bit + led_turbo_blink_bit | led_timer_plus_blink_bit);
    fanLedSet(0);
}

void cleanStartBlink(void){
    deviceState.blinkBits |= led_clean_blink_bit;
}

void cleanStopBlink(void) {
    deviceState.blinkBits &= ~led_clean_blink_bit;
    cleanLedOff();
}

void faultStartBlink(void) {
    deviceState.blinkBits = 0xff;
}
void faultStopBlink(void) {
    deviceState.blinkBits = 0;
    fanLedSet(0);
    cleanLedOff();
    powerLedOff();
    timerLedOff();
    plusTimerLedOff();
}

void allLedsStopBlink(void) {
    faultStopBlink();
}


/********************** Touch buttons ******************************/
touch_value InterpretTouchKeyValue(uint8_t * keyvalue) {
    uint16_t keyvalue16 = keyvalue[0] * 256 + keyvalue[1];
    touch_value returnvalue = BUTTON_MAX;
//	switch (~keyvalue16 & 0xffff) {
//        case 0x1000 :
//            //returnvalue = BUTTON_TIMER;
//				      returnvalue = BUTTON_AUTO;
//        break;
//        case 0x0800 :
//					    returnvalue = BUTTON_FOUR;
//        break;
//        case 0x0400 :
//					   returnvalue = BUTTON_THREE;
//        break;
//        case 0x0200 :
//					     returnvalue = BUTTON_TWO;
//        break;
//        case 0x0100 :
//					      returnvalue = BUTTON_ONE;
//        break;
//        case 0x0080 :
//					       returnvalue = BUTTON_POWER;
//        break;
//        case 0x2000 :
//					        returnvalue = BUTTON_CLEAN;
//        break;
//        default:
//        break;
//    }
	switch (~keyvalue16 & 0xffff) {
		    case 0x1000 :
					        returnvalue = BUTTON_CLEAN;
        break;
        case 0x0800 :
            //returnvalue = BUTTON_TIMER;
				      returnvalue = BUTTON_AUTO;
        break;
        case 0x0400 :
					    returnvalue = BUTTON_FOUR;
        break;
       case 0x0200 :
				
					   returnvalue = BUTTON_THREE;
        break;
        case 0x0100 :
					     returnvalue = BUTTON_TWO;
        break;
      
        case 0x0080 :
					       returnvalue = BUTTON_POWER;
        break;
				 case 0x0040 :
				 
					      returnvalue = BUTTON_ONE;
        break;
        
        default:
        break;
    }
		
		return returnvalue;
//    switch (~keyvalue16 & 0xffff) {
//        case 0x1000 :
//            returnvalue = BUTTON_TIMER;
//        break;
//        case 0x0800 :
//        break;
//        case 0x0400 :
//        break;
//        case 0x0200 :
//        break;
//        case 0x0100 :
//        break;
//        case 0x0080 :
//        break;
//        case 0x0040 :
//            returnvalue = BUTTON_FAN;
//        break;
//        case 0x0020 :
//            returnvalue = BUTTON_CLEAN;
//        break;
//        case 0x0010 :
//        break;
//        default:
//        break;
//    }
//		
//		return returnvalue;
}


/****************** state machine******************************/
extern osTimerId settingTimerHandle;
void ChangeStatus2TimerSetting(void){
	// 1, HMI
    fanLedSet(0);
    SetGlobalTimerValue(HOUR_2);
    timerSettingStartBlink(GetGlobalTimerValue());
    osTimerStart (settingTimerHandle, 10000);
    // deviceState.timer.enable = TRUE;
    SetGlobalTimerEnable(TRUE);
}

extern osTimerId fanTimerHandle;
void OutOfTimerSettingStatus(void) {
    unsigned long fantimer;
    fantimer = 1000 * (unsigned long)(deviceState.timer.timerValue) * 60 * 60 *2;
    // fantimer = 1000 * (unsigned long)(deviceState.timer.timerValue) * 10;

    timerSettingStopBlink();
    if (GetGlobalTimerValue() == HOUR_0) {
        SetGlobalTimerEnable(FALSE);
    }
    if (GetGlobalTimerEnable() == TRUE) {
        osTimerStart(fanTimerHandle, fantimer);
    }
    
}

/*
normal本来是风扇一档，亮一档LED和电源LED，开高压
修改后风扇0档，仅亮电源LED，开高压
*/
void ChangeStatus2Normal(void) {
	// 1, HMI
    powerLedOn();
    timerLedOn();
    if (deviceState.timer.enable == TRUE){
		plusTimerLedOn();
	}
    else {
        plusTimerLedOff();
    }
    if (deviceState.clean.time2clean == TRUE) {
        cleanStartBlink();
    }
    else {
        cleanStopBlink();
    }
		
    fanLedSet(deviceState.fanLever);

    // Fan and high volt
    ResumeFan();
		if(deviceState.hvFlag==0)
    TurnOnHighVolt();
}

//void ChangeStatus2Normal(void) {
//	// 1, HMI
//    powerLedOn();
//    timerLedOn();
//    if (deviceState.timer.enable == TRUE){
//		plusTimerLedOn();
//	}
//    else {
//        plusTimerLedOff();
//    }
//    if (deviceState.clean.time2clean == TRUE) {
//        cleanStartBlink();
//    }
//    else {
//        cleanStopBlink();
//    }
//    fanLedSet(deviceState.fanLever);

//    // Fan and high volt
//    ResumeFan();
//    TurnOnHighVolt();
//}


void ChangeStatus2Off(void) {
	// 1, HMI
    
    allLedsStopBlink();
    powerLedOff();
    timerLedOff();
    plusTimerLedOff();
    cleanLedOff();
    fanLedSet(0);
    // 2, fan
    StopFan();
    // 3, High Voltage
    TurnOffHighVolt();
}

void ChangeStatus2Fault(void) {
    faultStartBlink();
    // 2, fan
    StopFan();
    // 3, High Voltage
    TurnOffHighVolt();
}

void ChangeStatus2TurboTest(void){
		powerLedOn();
    SetGlobalFanlevel(LEVEL_TURBO);
    SetFanLevel();
    fanLedSet(LEVEL_TURBO);
		TurnOnHighVolt();
}

void OutOfFaultStatus(void) {
    faultStopBlink();
}


/************* interrupt callbacks  **********************************/
//extern ADC_HandleTypeDef hadc1;
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
    // if (deviceState.gotFaultevent == FALSE) {
    //     deviceState.gotFaultevent = TRUE;
        
    // }       
}

#define ALARM_HIGH_THRSHOLD 1241
#define ALARM_LOW_THRESHOLD 250
#define ADC_BUFF_SIZE 8

extern ADC_HandleTypeDef hadc1;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    static uint8_t i = 0;
    static uint16_t duration = 0;
    static uint32_t adc_buff[8];
    uint32_t adc_value;

    if (hadc == &hadc1){
            // this duration for making 100 times sample cycle
        if(duration++ >= 100) {
            duration = 0;
            adc_buff[i++] = HAL_ADC_GetValue(&hadc1);
            if (i>=8) {
                // 1, handle the data in adc_buff
                adc_value = FilterAdcBuff(adc_buff);
                            // out of the alram range
                if (adc_value>ALARM_HIGH_THRSHOLD | adc_value<ALARM_LOW_THRESHOLD){
                        // send fault event
                    if (GetGlobalFaultEvent() == FALSE) {
                        SetGlobalFaultEvent(TRUE); 
                    }
                }
                // 2, clear all data and reset other data
                for(uint8_t j=0; j<8; j++){
                    adc_buff[j] = 0;
                }
                i = 0;
            }
        }
        
    }
	
}

uint32_t FilterAdcBuff(uint32_t *buff){
    uint32_t temp;
    uint32_t sum;
            // rank the data from min to max
    for(uint8_t i=0; i<8-1; i++) {
        for(uint8_t j=0; j<8-1-i; j++) {
            if(buff[j] > buff[j+1]){
                temp = buff[j+1];
                buff[j+1] = buff[j];
                buff[j] = temp;

            }
        }
    }
    sum = buff[2] + buff[3] + buff[4] + buff[5];
    sum = sum / 4;
    return sum;
}





void StartSignal(void){

    powerLedOn();
    HAL_Delay(100);
    powerLedOff();
		 lowLedOn();
    HAL_Delay(100);
    lowLedOff();
		 midLedOn();
    HAL_Delay(100);
    midLedOff();
		highLedOn();
    HAL_Delay(100);
    highLedOff();
		turboLedOn();
    HAL_Delay(100);
    turboLedOff();
		plusTimerLedOn();//对应auto
		HAL_Delay(100);
		plusTimerLedOff();//
		 cleanLedOn();
    HAL_Delay(100);
    cleanLedOff();
		
		
//    turboLedOn();
//    HAL_Delay(100);
//    turboLedOff();
//    highLedOn();
//    HAL_Delay(100);
//    highLedOff();
//    midLedOn();
//    HAL_Delay(100);
//    midLedOff();
//    lowLedOn();
//    HAL_Delay(100);
//    lowLedOff();
//    cleanLedOn();
//    HAL_Delay(100);
//    cleanLedOff();
//    powerLedOn();
//    HAL_Delay(100);
//    powerLedOff();

	
}


