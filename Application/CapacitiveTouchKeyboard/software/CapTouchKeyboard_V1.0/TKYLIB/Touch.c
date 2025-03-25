/* ********************************* (C) COPYRIGHT ***************************
 * File Name: Touch.C
 * Author: WCH
 * Version: V1.6
 * Date: 2021/12/1
 * Description: Touch key routine
 ********************************************************************************************* */

/*********************************************************************
 * INCLUDES
 */
#include "Touch.h"

/*********************
 *      DEFINES
 *********************/
#define WAKEUPTIME  50     //Sleep Time = 250 * SLEEP_TRIGGER_TIME(100ms) = 25s

/**********************
 *      VARIABLES
 **********************/
__attribute__((aligned(4))) uint8_t TKY_MEMBUF[ TKY_MEMHEAP_SIZE ];
uint8_t wakeUpCount = 0, wakeupflag = 0;
uint16_t keyData = 0, scanData = 0;
uint32_t tkyPinAll = 0;
uint16_t tkyQueueAll = 0;
static const TKY_ChannelInitTypeDef my_tky_ch_init[TKY_QUEUE_END] = {TKY_CHS_INIT};

static const uint32_t TKY_Pin[ 14 ] = {
    GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_12, GPIO_Pin_13,GPIO_Pin_14, GPIO_Pin_15, GPIO_Pin_3,
    GPIO_Pin_2, GPIO_Pin_1, GPIO_Pin_0,GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9
};

/*Corresponds to the channel sequence initialized in TKY_CHS_INIT.
 * If set to 1, it means the channel is pressed.
 */
uint16_t KEY_TAB[KEY_COUNT] = {
    0x0401,0x0402,0x0404,0x0408,0x0410,0x0420,0x0440,0x0840,0x0820,0x0810,0x0808,0x0804,
    0x0201,0x0202,0x0204,0x0208,0x0210,0x0220,0x0240,0x1040,0x1020,0x1010,0x1008,0x1004,
    0x0101,0x0102,0x0104,0x0108,0x0110,0x0120,0x0140,0x2040,0x2020,0x2010,0x2008,0x2004,
    0x0081,0x0082,0x0084,0x0088,0x0090,0x00a0,0x00c0//,0x0040,0x0020,0x0010,0x0008,0x0004
};

/**********************
 *  STATIC PROTOTYPES
 **********************/
static KEY_T s_tBtn[KEY_COUNT];
static KEY_FIFO_T s_tKey;       /* Key FIFO variable, structure */
static void touch_InitKeyHard(void);
static void touch_InitKeyVar(void);
static void touch_DetectKey(uint16_t keydata);//static void touch_DetectKey(uint8_t i);
static void touch_Baseinit(void);
static void touch_Channelinit(void);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/* *******************************************************************************************************
 * @fn touch_InitKey
 *
 * @brief Initialize the key. This function is called by tky_Init().
 *
 * @return none */
void touch_InitKey(void)
{
    touch_InitKeyHard();          /* Initialize key hardware */
    touch_InitKeyVar();           /* Initialize key variables */
}

/* *******************************************************************************************************
 * @fn touch_PutKey
 * @brief Press 1 key value into the key FIFO buffer. Can be used to simulate a key.
 * @param _KeyCode - key code
 * @return none */
void touch_PutKey(uint8_t _KeyCode)
{
    s_tKey.Buf[s_tKey.Write] = _KeyCode;
// PRINT("KeyCode:%d\r\n",s_tKey.Buf[s_tKey.Write]);
    if (++s_tKey.Write  >= KEY_FIFO_SIZE)
    {
        s_tKey.Write = 0;
    }
}

/* *******************************************************************************************************
 * @fn touch_GetKey
 * @brief Read a key value from the key FIFO buffer.
 * @param None
 * @return key code */
uint8_t touch_GetKey(void)
{
    uint8_t ret;

    if (s_tKey.Read == s_tKey.Write)
    {
        return KEY_NONE;
    }
    else
    {
        ret = s_tKey.Buf[s_tKey.Read];
//         PRINT("ret:%d\r\n",ret);
        if (++s_tKey.Read >= KEY_FIFO_SIZE)
        {
            s_tKey.Read = 0;
        }
        return ret;
    }
}

/* *******************************************************************************************************
 * @fn touch_GetKeyState
 * @brief The state of the key is read
 * @param _ucKeyID - key ID, starting from 0
 * @return 1 - Press
 * 0 - Not pressed
********************************************************************************************************* */
uint8_t touch_GetKeyState(KEY_ID_E _ucKeyID)
{
    return s_tBtn[_ucKeyID].State;
}

/* *******************************************************************************************************
 * @fn touch_SetKeyParam
 * @brief Set key parameters
 * @param _ucKeyID - key ID, starting from 0
 * _LongTime - Long press event time
 * _RepeatSpeed ​​- continuous sending speed
 * @return none */
void touch_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatSpeed)
{
    s_tBtn[_ucKeyID].LongTime = _LongTime;          /* Long press time 0 means that long press event is not detected */
    s_tBtn[_ucKeyID].RepeatSpeed = _RepeatSpeed;            /* The speed of continuous sending by pressing the button, 0 means that continuous sending is not supported. */
    s_tBtn[_ucKeyID].RepeatCount = 0;                       /* Continuous send counter */
}


/* *******************************************************************************************************
 * @fn touch_ClearKey
 * @brief Clear key FIFO buffer
 * @param None
 * @return key code */
void touch_ClearKey(void)
{
    s_tKey.Read = s_tKey.Write;
}

/* *******************************************************************************************************
 * @fn touch_ScanWakeUp
 * @brief Touch Scan Wake-up Function
 * @param None
 * @return None */
void touch_ScanWakeUp(void)
{
    wakeUpCount = WAKEUPTIME; // ---Wake-up time---
    wakeupflag = 1;           // Set to wake up

    TKY_SetSleepStatusValue( ~tkyQueueAll ); // ---Set channels 0~11 to non-sleep state to prepare for continuous scanning in the next few seconds---
    dg_log("wake up for a while\n");
    TKY_SaveAndStop();    // ---Save the relevant registers---
    touch_GPIOSleep();
}

/* *******************************************************************************************************
 * @fn touch_ScanEnterSleep
 * @brief Touch Scan Hibernation Function
 * @param None
 * @return None */
void touch_ScanEnterSleep(void)
{
    TKY_SaveAndStop();    // ---Save the relevant registers---
    touch_GPIOSleep();
    wakeupflag = 0;       // Set to sleep state: 0, wake up state: 1
    TKY_SetSleepStatusValue( tkyQueueAll );
    dg_log("Ready to sleep\n");
}

/* *******************************************************************************************************
 * @fn touch_KeyScan
 * @brief Scan all keys. Non-blocking, periodic calls by systick interrupts
 * @param None
 * @return None */
void touch_KeyScan(void)
{
    uint8_t i;
    TKY_LoadAndRun();          // ---Separate settings saved before loading hibernation---
    keyData = TKY_PollForFilter();
//     PRINT("keyData:%04x\r\n",keyData);
#if TKY_SLEEP_EN
    if (keyData)
    {
        wakeUpCount = WAKEUPTIME;          // ---Wake-up time---
    }
#endif
    touch_DetectKey (keyData);
    TKY_SaveAndStop();          // ---Save the relevant registers---
}

/* *******************************************************************************************************
 * @fn touch_GPIOModeCfg
 * @brief Touch button mode configuration
 * @param None
 * @return None */
void touch_GPIOModeCfg(GPIOModeTypeDef mode)
{
    uint32_t pin = tkyPinAll;
    switch(mode)
    {
        case GPIO_ModeIN_Floating:
            R32_PA_PD_DRV &= ~pin;
            R32_PA_PU &= ~pin;
            R32_PA_DIR &= ~pin;
            break;

        case GPIO_ModeOut_PP_5mA:
            R32_PA_PU &= ~pin;
            R32_PA_PD_DRV &= ~pin;
            R32_PA_DIR |= pin;
            break;
        default:
            break;
    }
}


/* *******************************************************************************************************
 * @fn touch_GPIOSleep
 * @brief Configure the touch button to sleep
 * @param None
 * @return None */
void touch_GPIOSleep(void)
{
    uint32_t pin = tkyPinAll;

    R32_PA_PU &= ~pin;
    R32_PA_PD_DRV &= ~pin;
    R32_PA_DIR |= pin;
    R32_PA_CLR |= pin;
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

/* *******************************************************************************************************
 * @fn touch_InitKeyHard
 * @brief Initialize the touch button
 * @param None
 * @return None */
static void touch_InitKeyHard(void)
{
    touch_Baseinit( );
    touch_Channelinit( );
}


/* *******************************************************************************************************
 * @fn touch_InitKeyVar
 * @brief Initialize the touch button variable
 * @param None
 * @return None */
static void touch_InitKeyVar(void)
{
    uint8_t i;

    /* Clear the key FIFO read and write pointer */
    s_tKey.Read = 0;
    s_tKey.Write = 0;
    for (i = 0; i < KEY_FIFO_SIZE; i++)
    {
        s_tKey.Buf[i] = 0;
    }

    /* Assign a set of default values ​​to each key structure member variable */
    for (i = 0; i < KEY_COUNT; i++)
    {
        s_tBtn[i].LongTime = KEY_LONG_TIME;             /* Long press time 0 means that long press event is not detected */
        s_tBtn[i].Count = KEY_FILTER_TIME / 2;          /* The counter is set to half of the filtering time */
        s_tBtn[i].State = 0;                            /* The default state of the button is 0. */
        s_tBtn[i].RepeatSpeed = 0;                      /* The speed of continuous sending by pressing the button, 0 means that continuous sending is not supported. */
        s_tBtn[i].RepeatCount = 0;                      /* Continuous send counter */
    }

    /* If you need to change the parameters of a key separately, you can reassign the value separately here */
    /* For example, we hope that the same key value will be automatically resented after pressing key 1 for more than 1 second. */
//    s_tBtn[KID_K1].LongTime = 100;
// s_tBtn[KID_K1].RepeatSpeed ​​= 5; /* Automatically send key values ​​every 50ms */

}

/* *******************************************************************************************************
 * @fn IsKeyDownX
 * @brief To determine whether the button is pressed, the user can re-implement the function function by himself
 * @param None
 * @return 1 - Press
 * 0 - Not pressed */
static uint8_t IsKeyDown1(void)
{
    if (keyData & 0x0001)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown2(void)
{
    if (keyData & 0x0002)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown3(void)
{
    if (keyData & 0x0004)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown4(void)
{
    if (keyData & 0x0008)   return 1;
    else                    return 0;
}


static uint8_t IsKeyDown5(void)
{
    if (keyData & 0x0010)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown6(void)
{
    if (keyData & 0x0020)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown7(void)
{
    if (keyData & 0x0040)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown8(void)
{
    if (keyData & 0x0080)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown9(void)
{
    if (keyData & 0x0100)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown10(void)
{
    if (keyData & 0x0200)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown11(void)
{
    if (keyData & 0x0400)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown12(void)
{
    if (keyData & 0x0800)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown13(void)
{
    if (keyData & 0x1000)   return 1;
    else                    return 0;
}

static uint8_t IsKeyDown14(void)
{
    if (keyData & 0x2000)   return 1;
    else                    return 0;
}

/* *******************************************************************************************************
 * @fn touch_InfoDebug
 * @brief Touch data printing function
 * @param None
 * @return None */
 
void touch_InfoDebug(void)
{
    uint8_t i;
    int16_t data_dispNum[ TKY_MAX_QUEUE_NUM ]={0};
	int16_t bl,vl;

    for (i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
#if TKY_FILTER_MODE == FILTER_MODE_1
        bl = TKY_GetCurQueueBaseLine( i );
        vl = TKY_GetCurQueueValue( i );
        if(bl>vl)   data_dispNum[ i ] =  bl-vl ;
        else        data_dispNum[ i ] =  vl-bl ;
#else
        data_dispNum[ i ] = TKY_GetCurQueueValue( i );
#endif
    }

    for (i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
        dg_log("%04d,", data_dispNum[i]);
    } dg_log("\n");

    for (i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
        data_dispNum[ i ] = TKY_GetCurQueueBaseLine( i );
    }

    for (i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
        dg_log("%04d,", data_dispNum[i]);
    } dg_log("\n");
#if TKY_FILTER_MODE == FILTER_MODE_1
    for (i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
        dg_log("%04d,", TKY_GetCurQueueValue( i ));
    }dg_log("\n");
#endif
    for (i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
        dg_log("%04d,", TKY_GetCurQueueRealVal( i ));
    }dg_log("\r\n");
#if TKY_FILTER_MODE == FILTER_MODE_7
    for (i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
    	dg_log("%04d,", TKY_GetCurQueueValue2( i ));
    }dg_log("\r\n");
    for (i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
    	dg_log("%04d,", TKY_GetCurQueueBaseLine2( i ));
    }dg_log("\r\n");
    for (i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
    	dg_log("%04d,", TKY_GetCurQueueRealVal2( i ));
    }dg_log("\r\n");
#endif
    dg_log("\r\n");

}

/* *******************************************************************************************************
 * @fn touch_DetectKey
 * @brief Detect a key. The non-blocking state must be called periodically.
 * @param i - key structure variable pointer
 * @return None */
static void touch_DetectKey (uint16_t keydata)
{
    KEY_T* pBtn;
    uint8_t i = 0xff;
    for (uint8_t k = 0; k < KEY_COUNT; k++)
    {
        if (keydata == KEY_TAB[ k ]) //Corresponding to the key mapping table
        {
            i = k + 1;
            pBtn = &s_tBtn[ k ];
            break;
        }
    }

    if (i == 0xff)
    {
        for (uint8_t k = 0; k < KEY_COUNT; k++)
        {
            if (s_tBtn[ k ].State)
            {
                s_tBtn[ k ].State = 0;
                touch_PutKey ((uint8_t) (3 * (k+1) + 1));
            }
            s_tBtn[ k ].LongCount = 0;
            s_tBtn[ k ].RepeatCount = 0;
        }
        return;
    }
    // PRINT("key:%d;keyval:%04X\r\n",i,KEY_TAB[ i-1 ]);
    /* Press the button */
    // PRINT("pBtn->State%d;\r\n",pBtn->State);
    if (pBtn->State == 0)
    {
        pBtn->State = 1;
#if !KEY_MODE
        /* Send a message pressed by a button */
        touch_PutKey (3 * i + 0);
        // PRINT("1keyval:%d\r\n",(3 * i + 1));
#endif
    }

    /* Process long press */
    if (pBtn->LongTime > 0)
    {
        if (pBtn->LongCount < pBtn->LongTime)
        {
            /* Send a message with long pressing of buttons */
            if (++pBtn->LongCount == pBtn->LongTime)
            {
#if !KEY_MODE
                pBtn->State = 2;
                /* Put the key value into the FIFO key */
                touch_PutKey ((uint8_t) (3 * i + 2));
                // PRINT("3keyval:%d\r\n",(3 * i + 3));
#endif
            }
        }
        else
        {
            if (pBtn->RepeatSpeed > 0)
            {
                if (++pBtn->RepeatCount >= pBtn->RepeatSpeed)
                {
                    pBtn->RepeatCount = 0;
#if !KEY_MODE
                    /* After long pressing the key, send 1 key every pBtn->RepeatSpeed*10ms */
                    touch_PutKey ((uint8_t) (3 * i + 0));
                    // PRINT("4keyval:%d\r\n",(3 * i + 0));
#endif
                }
            }
        }
    }
}
/* *******************************************************************************************************
 * @fn touch_Baseinit
 * @brief Touch basic library initialization
 * @param None
 * @return None */
static void touch_Baseinit(void)
{
    TKY_BaseInitTypeDef TKY_BaseInitStructure = {0};
    for(uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++)  // Initialize tkyPinAll and tkyQueueAll variables
    {
        tkyPinAll |= TKY_Pin[my_tky_ch_init[i].channelNum];
        tkyQueueAll |= 1<<i;
    }
    dg_log("tP : %08x, tQ : %04x\n",tkyPinAll,tkyQueueAll);

#if (TKY_SHIELD_EN)&&((TKY_FILTER_MODE != FILTER_MODE_9))
    tkyPinAll |= TKY_SHIELD_PIN;
#endif

    touch_GPIOSleep();  // Pull down all touch pin feet

#if (TKY_SHIELD_EN)&&((TKY_FILTER_MODE != FILTER_MODE_9))
    tkyPinAll &= ~TKY_SHIELD_PIN;
    GPIOA_ModeCfg(TKY_SHIELD_PIN, GPIO_ModeIN_Floating);//Shield Pin， only for CH58x series
#endif

    // -------------------------------------
    TKY_BaseInitStructure.filterMode = TKY_FILTER_MODE;
#if (TKY_FILTER_MODE != FILTER_MODE_9)
    TKY_BaseInitStructure.shieldEn = TKY_SHIELD_EN;
#else
    TKY_BaseInitStructure.shieldEn = 0;
#endif
    TKY_BaseInitStructure.singlePressMod = TKY_SINGLE_PRESS_MODE;
    TKY_BaseInitStructure.filterGrade = TKY_FILTER_GRADE;
    TKY_BaseInitStructure.maxQueueNum = TKY_MAX_QUEUE_NUM;
    TKY_BaseInitStructure.baseRefreshOnPress = TKY_BASE_REFRESH_ON_PRESS;
    // ---Baseline update speed, baseRefreshSampleNum and filterGrade, are inversely proportional to the baseline update speed. The baseline update speed is also related to the code structure. You can observe it through the function GetCurQueueBaseLine--
    TKY_BaseInitStructure.baseRefreshSampleNum = TKY_BASE_REFRESH_SAMPLE_NUM;
    TKY_BaseInitStructure.baseUpRefreshDouble = TKY_BASE_UP_REFRESH_DOUBLE;
    TKY_BaseInitStructure.baseDownRefreshSlow = TKY_BASE_DOWN_REFRESH_SLOW;
    TKY_BaseInitStructure.tkyBufP = TKY_MEMBUF;
    TKY_BaseInit( TKY_BaseInitStructure );
}

/* *******************************************************************************************************
 * @fn touch_Channelinit
 * @brief Touch channel initialization
 * @param None
 * @return None */
static void touch_Channelinit(void)
{

    uint8_t error_flag = 0;
    uint16_t chx_mean = 0;

    for(uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
    	TKY_CHInit(my_tky_ch_init[i]);
    }

#if (TKY_FILTER_MODE != FILTER_MODE_9)
    for(uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {

    	chx_mean = TKY_GetCurChannelMean(my_tky_ch_init[i].channelNum, my_tky_ch_init[i].chargeTime,
										 my_tky_ch_init[i].disChargeTime, 1000);

    	if(chx_mean < 3400 || chx_mean > 3800)
    	{
    		error_flag = 1;
    	}
    	else
    	{
    		TKY_SetCurQueueBaseLine(i, chx_mean);
    	}
    	dg_log("queue : %d ch : %d , mean : %d\n",i,my_tky_ch_init[i].channelNum,chx_mean);

    }
    // The charge and discharge baseline value is abnormal, recalibrate the baseline value
    if(error_flag != 0)
    {
    	touch_GPIOSleep();  // Pull down all touch pin feet
        dg_log("\n\nCharging parameters error, preparing for recalibration ...\n\n");
        uint8_t charge_time;
        for (uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++) {       // ADC channel conversion by maximum number of sequences
          charge_time = 0,chx_mean = 0;
          GPIOA_ModeCfg(TKY_Pin[my_tky_ch_init[i].channelNum],GPIO_ModeIN_Floating);
          while (1)
          {
              chx_mean = TKY_GetCurChannelMean(my_tky_ch_init[i].channelNum, charge_time,3, 1000);

// dg_log("testing... chg : %d, baseline : %d\n",charge_time,chx_mean);//Print baseline value

              if ((charge_time == 0) && ((chx_mean > 3800))) {// Below the minimum charging parameter
                  dg_log("Error, %u KEY%u Too small Cap,Please check the hardware !\r\n",chx_mean,i);
                  break;
              }
              else {
                  if ((chx_mean > 3200) &&(chx_mean < 3800)) {// Charging parameters are normal
                      TKY_SetCurQueueBaseLine(i, chx_mean);
                      TKY_SetCurQueueChargeTime(i,charge_time,3);
                      dg_log("channel:%u, chargetime:%u,BaseLine:%u\r\n",
                            i, charge_time, chx_mean);
                      break;
                  }else if(chx_mean >= 3800)
                  {
                	  TKY_SetCurQueueBaseLine(i, TKY_GetCurChannelMean(my_tky_ch_init[i].channelNum, charge_time-1,3, 20));
                	  TKY_SetCurQueueChargeTime(i,charge_time-1,3);
                	  dg_log("Warning,channel:%u Too large Current, chargetime:%u,BaseLine:%u\r\n",
                	                              i, charge_time, chx_mean);
                	  break;
                  }
                  charge_time++;
                  if (charge_time > 0x1f) {    // Maximum charging parameters exceeded
                      dg_log("Error, Chargetime Max,KEY%u Too large Cap,Please check the hardware !\r\n",i);
                      break;
                  }
              }
          }
          GPIOA_ModeCfg(TKY_Pin[my_tky_ch_init[i].channelNum],GPIO_ModeOut_PP_5mA);
          GPIOA_ResetBits(TKY_Pin[my_tky_ch_init[i].channelNum]);
        }
    }
#endif
#if (TKY_FILTER_MODE == FILTER_MODE_9) ||(TKY_FILTER_MODE == FILTER_MODE_7)
#if TKY_SHIELD_EN
    TKY_ChannelInitTypeDef TKY_ChannelInitStructure = {0};
	// --------Initialize touch channel 0 and is ranked as the 13th position in the detection queue-----------
	TKY_ChannelInitStructure.queueNum = 12;
	TKY_ChannelInitStructure.channelNum = 0;
	TKY_ChannelInitStructure.threshold = 40; // ---The threshold threshold is related to PCB board, please adjust according to actual situation---
	TKY_ChannelInitStructure.threshold2 = 30;
	TKY_ChannelInitStructure.sleepStatus = 1;
	TKY_ChannelInitStructure.baseLine = 600;
	TKY_CHInit( TKY_ChannelInitStructure );
#endif
    // In Filter 9 mode, you need to use a separate function to calibrate the baseline value
    TKY_CaliCrowedModBaseLine(0, 1000);
    for (uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
#if(TKY_FILTER_MODE == FILTER_MODE_7)
    	TKY_SetCurQueueThreshold2(i, my_tky_ch_init[i].threshold, my_tky_ch_init[i].threshold2);
#elif(TKY_FILTER_MODE == FILTER_MODE_9)
    	TKY_SetCurQueueThreshold(i, my_tky_ch_init[i].threshold, my_tky_ch_init[i].threshold2);
#endif

		dg_log("key:%u -> thresholdUp:%u;  thresholdDown:%u;\r\n",i,
			   my_tky_ch_init[i].threshold, my_tky_ch_init[i].threshold2);
    }
#endif
    TKY_SaveAndStop();
}

/* *******************************************************************************************************
 * @fn touch_DetectWheelSlider
 * @brief Touch pulley data processing
 * @param None
 * @return pulley coordinates */
uint16_t touch_DetectWheelSlider(void)
{
	uint8_t  loop;
	uint8_t  max_data_num;
	uint16_t d1;
	uint16_t d2;
	uint16_t d3;
	uint16_t wheel_rpos;
	uint16_t dsum;
	int16_t dval;
	uint16_t unit;
	uint16_t wheel_data[TOUCH_WHEEL_ELEMENTS] = {0};
	uint8_t num_elements=TOUCH_WHEEL_ELEMENTS;
	uint16_t p_threshold = 60;

	if (num_elements < 3)
	{
		return 0;
	}

	for (loop = 0; loop < num_elements; loop++)
	{
		dval = TKY_GetCurQueueValue( loop );
		if(dval>0)
		{
			wheel_data[ loop ] = (uint16_t)dval;
		}
		else {
			wheel_data[ loop ] = 0;
		}
	}
	/* Search max data in slider */
	max_data_num = 0;
	for (loop = 0; loop < (num_elements - 1); loop++)
	{
		if (wheel_data[max_data_num] < wheel_data[loop + 1])
		{
			max_data_num = (uint8_t) (loop + 1);
		}
	}
	/* Array making for wheel operation          */
	/*    Maximum change CH_No -----> Array"0"    */
	/*    Maximum change CH_No + 1 -> Array"2"    */
	/*    Maximum change CH_No - 1 -> Array"1"    */
	if (0 == max_data_num)
	{
		d1 = (uint16_t) (wheel_data[0] - wheel_data[num_elements - 1]);
		d2 = (uint16_t) (wheel_data[0] - wheel_data[1]);
		dsum = (uint16_t) (wheel_data[0] + wheel_data[1] + wheel_data[num_elements - 1]);
	}
	else if ((num_elements - 1) == max_data_num)
	{
		d1 = (uint16_t) (wheel_data[num_elements - 1] - wheel_data[num_elements - 2]);
		d2 = (uint16_t) (wheel_data[num_elements - 1] - wheel_data[0]);
		dsum = (uint16_t) (wheel_data[0] + wheel_data[num_elements - 2] + wheel_data[num_elements - 1]);
	}
	else
	{
		d1 = (uint16_t) (wheel_data[max_data_num] - wheel_data[max_data_num - 1]);
		d2 = (uint16_t) (wheel_data[max_data_num] - wheel_data[max_data_num + 1]);
		dsum = (uint16_t) (wheel_data[max_data_num + 1] + wheel_data[max_data_num] + wheel_data[max_data_num - 1]);
	}

	if (0 == d1)
	    {
	        d1 = 1;
	    }
	    /* Constant decision for operation of angle of wheel    */
	    if (dsum > p_threshold)
	    {
	        d3 = (uint16_t) (TOUCH_DECIMAL_POINT_PRECISION + ((d2 * TOUCH_DECIMAL_POINT_PRECISION) / d1));

	        unit       = (uint16_t) (TOUCH_WHEEL_RESOLUTION / num_elements);
	        wheel_rpos = (uint16_t) (((unit * TOUCH_DECIMAL_POINT_PRECISION) / d3) + (unit * max_data_num));

	        /* Angle division output */
	        /* diff_angle_ch = 0 -> 359 ------ diff_angle_ch output 1 to 360 */
	        if (0 == wheel_rpos)
	        {
	            wheel_rpos = TOUCH_WHEEL_RESOLUTION;
	        }
	        else if ((TOUCH_WHEEL_RESOLUTION + 1) < wheel_rpos)
	        {
	            wheel_rpos = 1;
	        }
	        else
	        {
	            /* Do Nothing */
	        }
	    }
	    else
	    {
	        wheel_rpos = TOUCH_OFF_VALUE;
	    }

	return wheel_rpos;
}

/* *******************************************************************************************************
 * @fn touch_DetectWheelSlider
 * @brief Touch slider data processing
 * @param None
 * @return slider coordinates */
uint16_t touch_DetecLineSlider(void) 
{
    uint8_t loop;
    uint8_t max_data_num;
    uint16_t d1;
    uint16_t d2;
    uint16_t d3;
    uint16_t slider_rpos;
    uint16_t resol_plus;
    uint16_t dsum;
    int16_t dval;
    uint16_t slider_data[TOUCH_SLIDER_ELEMENTS] = {0};
    uint8_t num_elements = TOUCH_SLIDER_ELEMENTS;
    uint16_t p_threshold = 60;

    if (num_elements < 3) {
        return 0;
    }

    for (loop = 0; loop < num_elements; loop++) {
        dval = TKY_GetCurQueueValue (loop);
        if (dval > 0) {
            slider_data[loop] = (uint16_t)dval;
        } else {
            slider_data[loop] = 0;
        }
    }
    /* Search max data in slider */
    max_data_num = 0;
    for (loop = 0; loop < (num_elements - 1); loop++) {
        if (slider_data[max_data_num] < slider_data[loop + 1]) {
            max_data_num = (uint8_t)(loop + 1);
        }
    }

    /* Array making for slider operation-------------*/
    /*     |    Maximum change CH_No -----> Array"0"    */
    /*     |    Maximum change CH_No + 1 -> Array"2"    */
    /*     |    Maximum change CH_No - 1 -> Array"1"    */
    if (0 == max_data_num) {
        d1 = (uint16_t)(slider_data[0] - slider_data[2]);
        d2 = (uint16_t)(slider_data[0] - slider_data[1]);
    } else if ((num_elements - 1) == max_data_num) {
        d1 = (uint16_t)(slider_data[num_elements - 1] - slider_data[num_elements - 2]);
        d2 = (uint16_t)(slider_data[num_elements - 1] - slider_data[num_elements - 3]);
    } else {
        d1 = (uint16_t)(slider_data[max_data_num] - slider_data[max_data_num - 1]);
        d2 = (uint16_t)(slider_data[max_data_num] - slider_data[max_data_num + 1]);
    }

    dsum = (uint16_t)(d1 + d2);

    /* Constant decision for operation of angle of slider */
    /* Scale results to be 0-TOUCH_SLIDER_RESOLUTION */
    if (dsum > p_threshold) {
        if (0 == d1) {
            d1 = 1;
        }

        /* x : y = d1 : d2 */
        d3 = (uint16_t)(TOUCH_DECIMAL_POINT_PRECISION + ((d2 * TOUCH_DECIMAL_POINT_PRECISION) / d1));

        slider_rpos = (uint16_t)(((TOUCH_DECIMAL_POINT_PRECISION * TOUCH_SLIDER_RESOLUTION) / d3) +
                                 (TOUCH_SLIDER_RESOLUTION * max_data_num));

        resol_plus = (uint16_t)(TOUCH_SLIDER_RESOLUTION * (num_elements - 1));

        if (0 == slider_rpos) {
            slider_rpos = 1;
        } else if (slider_rpos >= resol_plus) {
            slider_rpos = (uint16_t)(((slider_rpos - resol_plus) * 2) + resol_plus);
            if (slider_rpos > (TOUCH_SLIDER_RESOLUTION * num_elements)) {
                slider_rpos = TOUCH_SLIDER_RESOLUTION;
            } else {
                slider_rpos = (uint16_t)(slider_rpos / num_elements);
            }
        } else if (slider_rpos <= TOUCH_SLIDER_RESOLUTION) {
            if (slider_rpos < (TOUCH_SLIDER_RESOLUTION / 2)) {
                slider_rpos = 1;
            } else {
                slider_rpos = (uint16_t)(slider_rpos - (TOUCH_SLIDER_RESOLUTION / 2));
                if (0 == slider_rpos) {
                    slider_rpos = 1;
                } else {
                    slider_rpos = (uint16_t)((slider_rpos * 2) / num_elements);
                }
            }
        } else {
            slider_rpos = (uint16_t)(slider_rpos / num_elements);
        }
    } else {
        slider_rpos = TOUCH_OFF_VALUE;
    }

    return slider_rpos;
}
