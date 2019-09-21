/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     misonyo   first version
 */
 
/****************************************************************
A 通道通过timer3计数，输入管脚为PD2
B 通道通过timer4计数，输入管脚为PE0
C 输出管脚通过参数配置可以为PA0-PA9，作为模式3时，只能PA1输出
  模式3指定频率输出使用timer5的pwm模式
  模式2的同步信号使用timer2的计数模式，输入管脚为
  输入使能信号为PE2引脚，低电平使能
****************************************************************/


#include "math.h"
#include "main.h"
#include <string.h>
#include "stm32f4xx_hal.h"
#include "input_cap_timer.h"


#define FRE_10M      10000000
#define FRE_1K       1000

#define TIM7_PVALUE_1K  (((SystemCoreClock /2) / FRE_1K) - 1)
#define TIM9_PVALUE_10M  (((SystemCoreClock /2) / FRE_10M) - 1)

#define RECORE_DATA_NUMS 20

#define CMD_HEAD_H  0x55
#define CMD_HEAD_L  0xaa


#define  TICK_PER_SECOND   1000
#define  PIN_PLUS_OUTPUT   GPIOB,GPIO_PIN_9

#define  GAP_MODE_INPUT_PIN0       GPIOD,GPIO_PIN_4
#define  GAP_MODE_INPUT_PIN1       GPIOD,GPIO_PIN_5
#define  GAP_MODE_INPUT_PIN2       GPIOD,GPIO_PIN_6
#define  GAP_MODE_INPUT_PIN3       GPIOD,GPIO_PIN_7

#define  PIN_HIGH          1
#define  PIN_LOW           0

struct usr_tcp_buf_s input_cmd_buf,reply_cmd_buf;
struct mode_config g_mode_config_data;
struct edge_recored_s  edge_recorer;
	
TIM_HandleTypeDef      TimHandle1;
TIM_HandleTypeDef      TimHandle2;
TIM_HandleTypeDef      TimHandle3;
TIM_HandleTypeDef      TimHandle4;
TIM_HandleTypeDef      TimHandle8;
TIM_HandleTypeDef      TimHandle7;
TIM_HandleTypeDef      TimHandle9;

TIM_HandleTypeDef      TimHandle5;

uint8_t check_plus_type(void);

void delay_100ns(uint32_t time_100ns)
{
    uint32_t delta;
    time_100ns = time_100ns * (SysTick->LOAD/(10000000/TICK_PER_SECOND));
    delta = SysTick->VAL;
    while (delta - SysTick->VAL < time_100ns);
}

void input_cap_init()
{
    TIM_ClockConfigTypeDef sClockSourceConfig;

    /*timer1 初始化为计数模式，ETR*/
    TimHandle1.Instance = TIM1; 
    TimHandle1.Init.Prescaler = 0; 
    TimHandle1.Init.CounterMode=TIM_COUNTERMODE_UP; 
    TimHandle1.Init.Period=0xFFFF; 
    TimHandle1.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; 
    HAL_TIM_Base_Init(&TimHandle1);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = TIM_ETRPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_ETRPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0x0;
    HAL_TIM_ConfigClockSource(&TimHandle1, &sClockSourceConfig);
    __HAL_TIM_SET_COUNTER(&TimHandle1,0);
    HAL_TIM_Base_Start(&TimHandle1);
	
	/*timer2 初始化为计数模式，ETR*/
	TimHandle2.Instance = TIM2; 
    TimHandle2.Init.Prescaler = 0; 
    TimHandle2.Init.CounterMode=TIM_COUNTERMODE_UP; 
    TimHandle2.Init.Period=0xFFFF; 
    TimHandle2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; 
    HAL_TIM_Base_Init(&TimHandle2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = TIM_ETRPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_ETRPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0x0;
    HAL_TIM_ConfigClockSource(&TimHandle2, &sClockSourceConfig);
    __HAL_TIM_SET_COUNTER(&TimHandle2,0);
    HAL_TIM_Base_Start(&TimHandle2);
	
	/*timer3 初始化为计数模式，ETR*/
	TimHandle3.Instance = TIM3; 
    TimHandle3.Init.Prescaler = 0; 
    TimHandle3.Init.CounterMode=TIM_COUNTERMODE_UP; 
    TimHandle3.Init.Period=0xFFFF; 
    TimHandle3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; 
    HAL_TIM_Base_Init(&TimHandle3);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = TIM_ETRPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_ETRPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0x0;
    HAL_TIM_ConfigClockSource(&TimHandle3, &sClockSourceConfig);
    __HAL_TIM_SET_COUNTER(&TimHandle3,0);
    HAL_TIM_Base_Start(&TimHandle3);

	/*timer4 初始化为计数模式，ETR*/
    TimHandle4.Instance = TIM4; 
    TimHandle4.Init.Prescaler = 0; 
    TimHandle4.Init.CounterMode=TIM_COUNTERMODE_UP; 
    TimHandle4.Init.Period=0xFFFF; 
    TimHandle4.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; 
    HAL_TIM_Base_Init(&TimHandle4);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = TIM_ETRPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_ETRPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0x0;
    HAL_TIM_ConfigClockSource(&TimHandle4, &sClockSourceConfig);
    __HAL_TIM_SET_COUNTER(&TimHandle4,0);
    HAL_TIM_Base_Start(&TimHandle4);
	
    /*timer8 初始化为计数模式，ETR*/
    TimHandle8.Instance = TIM8; 
    TimHandle8.Init.Prescaler = 0; 
    TimHandle8.Init.CounterMode=TIM_COUNTERMODE_UP; 
    TimHandle8.Init.Period=0xFFFF; 
    TimHandle8.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; 
    HAL_TIM_Base_Init(&TimHandle8);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = TIM_ETRPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_ETRPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0x0;
    HAL_TIM_ConfigClockSource(&TimHandle8, &sClockSourceConfig);
    __HAL_TIM_SET_COUNTER(&TimHandle8,0);
    HAL_TIM_Base_Start(&TimHandle8);    
    
	/*timer7 初始化为普通定时器*/
    TimHandle7.Instance = TIM7; 
    TimHandle7.Init.Prescaler = TIM7_PVALUE_1K; 
    TimHandle7.Init.CounterMode=TIM_COUNTERMODE_UP; 
    TimHandle7.Init.Period=100; 
    TimHandle7.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; 
    HAL_TIM_Base_Init(&TimHandle7);
    HAL_TIM_Base_Start_IT(&TimHandle7);
		
	/*timer9 初始化为普通定时器*/
    TimHandle9.Instance = TIM9; 
    TimHandle9.Init.Prescaler = TIM9_PVALUE_10M; 
    TimHandle9.Init.CounterMode=TIM_COUNTERMODE_UP; 
    TimHandle9.Init.Period=0xFFFF; 
    TimHandle9.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; 
    HAL_TIM_Base_Init(&TimHandle9);
    HAL_TIM_Base_Start(&TimHandle9);
}

void pwm_output_conifg(uint32_t fre, float duty)
{
    TimHandle4.Instance = TIM4;
    TIM_OC_InitTypeDef sConfig; 
    uint32_t uhPrescalerValue,period_value;
    
	HAL_TIM_Base_DeInit(&TimHandle4);
	
    /*tim4时钟为84M*/
    uhPrescalerValue = (uint32_t)((SystemCoreClock /2)/84000000) - 1;
    /*根据需要的pwm输出频率，计算period_value*/
    period_value = (uint32_t)84000000/fre;
    
    TimHandle4.Init.Prescaler = uhPrescalerValue;
    TimHandle4.Init.Period = period_value;
    TimHandle4.Init.ClockDivision = 0;
    TimHandle4.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle4);
	
	TIM_ClockConfigTypeDef sClockSourceConfig;
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    sClockSourceConfig.ClockPolarity = TIM_ETRPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_ETRPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0x0;
    HAL_TIM_ConfigClockSource(&TimHandle4, &sClockSourceConfig);  
	

    /*##-2- Configure the PWM channels #########################################*/ 
    /* Common configuration for all channels */
    sConfig.OCMode = TIM_OCMODE_PWM1;
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig.OCFastMode = TIM_OCFAST_DISABLE;

    /* Set the pulse value for channel 4 */
    sConfig.Pulse = duty*period_value;  
    HAL_TIM_PWM_ConfigChannel(&TimHandle4, &sConfig, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_4);
}

void one_plus_init_send(uint32_t duration)
{
    HAL_GPIO_WritePin(PIN_PLUS_OUTPUT, PIN_HIGH);
    delay_100ns(duration);
    HAL_GPIO_WritePin(PIN_PLUS_OUTPUT, PIN_LOW);
}


void get_cap_nums(void)
{
	uint32_t cnt3,cnt4;
	cnt3 = __HAL_TIM_GET_COUNTER(&TimHandle3);
	cnt4 = __HAL_TIM_GET_COUNTER(&TimHandle4);
}

void reset_counter3_4(void)
{
	__HAL_TIM_SET_COUNTER(&TimHandle3,0);
	__HAL_TIM_SET_COUNTER(&TimHandle4,0);
	__HAL_TIM_SET_COUNTER(&TimHandle1,0);
	__HAL_TIM_SET_COUNTER(&TimHandle2,0);
}


static uint16_t wait_sync_plus(uint16_t wait_us)
{
    if(wait_us == 0)
    {
        return 1;
    }
    while(wait_us)
    {
		/*TimHandle8作为计数器，当有同步信号输入时，则跳出*/
        if(__HAL_TIM_GET_COUNTER(&TimHandle8)>0)
        {
            __HAL_TIM_SET_COUNTER(&TimHandle8,0);
            return 1;
        }
        delay_100ns(1);
        wait_us--;
    }
    return 0;
}

static void set_mode2(void)
{
    uint32_t frequence;
    float duty;
    frequence = g_mode_config_data.mode2.fre;
    duty = g_mode_config_data.mode2.duty/100.0f;
    pwm_output_conifg(frequence,duty);
}


float total_conut(void)
{
    uint32_t a1 = __HAL_TIM_GET_COUNTER(&TimHandle1);
    uint32_t a2 = __HAL_TIM_GET_COUNTER(&TimHandle2);
    uint32_t b1 = __HAL_TIM_GET_COUNTER(&TimHandle3);
    uint32_t b2 = __HAL_TIM_GET_COUNTER(&TimHandle4);
    uint32_t a,b;
    a = a1+a2;
    b = b1+b2;
    return a*a+b*b;
}

static uint8_t get_input_mode(void)
{
	uint8_t bit0,bit1,bit2,bit3;
	/*读取四个输入的状态*/
	bit0 = !HAL_GPIO_ReadPin(GAP_MODE_INPUT_PIN0);
	bit1 = !HAL_GPIO_ReadPin(GAP_MODE_INPUT_PIN1);
	bit2 = !HAL_GPIO_ReadPin(GAP_MODE_INPUT_PIN2);
	bit3 = !HAL_GPIO_ReadPin(GAP_MODE_INPUT_PIN3);
	return bit0+(bit1<<1)+(bit2<<2)+(bit3<<3);
}




void start_work(void)
{
	static uint8_t pre_mode_type=0,pre_en_state=0;
	float counts=0;
	struct mode_config *cfg_data;
	static uint32_t  cnt_cmp =0;
	cfg_data = &g_mode_config_data;
 
	if(cfg_data->run_state ==1)
    {
		if(pre_mode_type !=1)
		{
			pre_mode_type=cfg_data->run_state;
			set_mode1();
		}
        if(0 < get_input_mode())
        {   
			/*通过软件设定的模式值以及输入管脚选择的模式，最终确定cnt_cmp值*/
			cnt_cmp = cfg_data->gap[get_input_mode()-1]*cfg_data->gap[get_input_mode()-1]*100/4;
            counts = total_conut();
            if(counts>=cnt_cmp)
            {
                reset_counter3_4();
                /*等待同步信号*/
                if(wait_sync_plus(cfg_data->mode1.wait_max_time))
                {
                    delay_100ns(cfg_data->mode1.delay_time_ns);
                    one_plus_init_send(cfg_data->mode1.c_duration);
                }
            }
		}
		else
		{
			pre_en_state=0;
		}
    }
	else if(cfg_data->run_state ==2)
	{
		if(pre_mode_type !=2)
		{
			pre_mode_type =2;
			set_mode2();
		}
	}
}


void pin_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	GPIO_InitStructure.Pin = GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* 作为输入引脚 X0-X3*/
	GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void set_mode1(void)
{
   /*输入脉冲计数器初始化*/
    pin_config();
    /*输入脉冲计数器初始化*/
    input_cap_init();
}


uint8_t  recored_plus(uint16_t max_wait_time_us)
{
	reset_counter3_4();
	__HAL_TIM_SET_COUNTER(&TimHandle9,0);
	while((edge_recorer.ch1_cnt < RECORE_DATA_NUMS)&&(edge_recorer.ch2_cnt < RECORE_DATA_NUMS))
	{
		if(__HAL_TIM_GET_COUNTER(&TimHandle1) > edge_recorer.ch1_cnt)
		{
			edge_recorer.ch1_cnt = __HAL_TIM_GET_COUNTER(&TimHandle1); 
			edge_recorer.ch1_edge[edge_recorer.ch1_cnt].time = __HAL_TIM_GET_COUNTER(&TimHandle9);
		}
		if(__HAL_TIM_GET_COUNTER(&TimHandle2) > edge_recorer.ch2_cnt)	
		{
			edge_recorer.ch2_cnt = __HAL_TIM_GET_COUNTER(&TimHandle2); 
			edge_recorer.ch2_edge[edge_recorer.ch2_cnt].time = __HAL_TIM_GET_COUNTER(&TimHandle9);
		}
		if(__HAL_TIM_GET_COUNTER(&TimHandle9)>max_wait_time_us*10)
		{
			memset(&edge_recorer,0,sizeof(edge_recorer));
			return 0;
		}
	}
	return 1;
}

uint8_t parse_plus(void)
{
	float ratio;
	uint8_t i,j;
	uint16_t ch1_cycle_time[RECORE_DATA_NUMS],ch2_cycle_time[RECORE_DATA_NUMS]; 
	uint16_t ch1_ch2_offset_time[RECORE_DATA_NUMS];
	for(i=1;i<RECORE_DATA_NUMS;i++)
	{
		ch1_cycle_time[i-1] = edge_recorer.ch1_edge[i].time - edge_recorer.ch1_edge[i-1].time;
		ch2_cycle_time[i-1] = edge_recorer.ch2_edge[i].time - edge_recorer.ch2_edge[i-1].time;
		ch1_ch2_offset_time[i-1] = edge_recorer.ch2_edge[i].time-edge_recorer.ch1_edge[i].time;
	}
	sort(ch1_cycle_time,RECORE_DATA_NUMS);
	sort(ch2_cycle_time,RECORE_DATA_NUMS);
	sort(ch1_ch2_offset_time,RECORE_DATA_NUMS);
	/*取中位数*/
	edge_recorer.ch1_cycle_time = ch1_cycle_time[RECORE_DATA_NUMS/2];
	edge_recorer.ch2_cycle_time = ch1_cycle_time[RECORE_DATA_NUMS/2];
	edge_recorer.ch1_ch2_offset_time  = ch1_ch2_offset_time[RECORE_DATA_NUMS/2];
	
	if(abs(edge_recorer.ch1_cycle_time -edge_recorer.ch1_cycle_time)< 500)
	{
		ratio = abs(edge_recorer.ch1_ch2_offset_time)*1.0/edge_recorer.ch1_cycle_time*1.0;
		if(((ratio>0.25)&&(ratio<0.35))||((ratio>0.65)&&(ratio<0.76)))
		{
			memset(&edge_recorer,0,sizeof(edge_recorer));
			return 1;
		}
	}
	memset(&edge_recorer,0,sizeof(edge_recorer));
	return 0;
}



uint8_t check_plus_type(void)
{
	uint8_t ret;
	/*记录波形，超时时间设置为20ms*/
	ret = recored_plus(20000);
	if(ret)
	{
		return parse_plus();
	}
	return 0;
}


void clear_usr_buf(void)
{
	memset(&input_cmd_buf,0,sizeof(input_cmd_buf));
}

void handle_cmd_data(void)
{
	uint8_t ret=1;
	if(input_cmd_buf.len)
	{
		/*配置模式1的参数*/
		if((input_cmd_buf.buf[0] == CMD_HEAD_H)&&(input_cmd_buf.buf[1] == CMD_HEAD_L)&&(input_cmd_buf.buf[2] ==1))
		{
			memcpy(&g_mode_config_data.mode1,&input_cmd_buf.buf[2],8);
			g_mode_config_data.run_state =1;
			write_config_par(&g_mode_config_data,sizeof(g_mode_config_data));
			ret=0;
		}
		/*配置模式2的参数*/
		if((input_cmd_buf.buf[0] == CMD_HEAD_H)&&(input_cmd_buf.buf[1] == CMD_HEAD_L)&&(input_cmd_buf.buf[2] ==2))
		{
			memcpy(&g_mode_config_data.mode2,&input_cmd_buf.buf[2],8);
			g_mode_config_data.run_state =2;
			write_config_par(&g_mode_config_data,sizeof(g_mode_config_data));
			ret=0;
		}
		/*配置计数间隔模式值，总共15个模式，每个模式对应一个值*/
		if((input_cmd_buf.buf[0] == CMD_HEAD_H)&&(input_cmd_buf.buf[1] == CMD_HEAD_L)&&(input_cmd_buf.buf[2] ==3))
		{
			memcpy(&g_mode_config_data.gap,&input_cmd_buf.buf[3],15);
			write_config_par(&g_mode_config_data,sizeof(g_mode_config_data));
			ret=0;
		}		
		clear_usr_buf();		
		reply_cmd_buf.len = 10+2;
		reply_cmd_buf.buf[0] =CMD_HEAD_H;
		reply_cmd_buf.buf[1] =CMD_HEAD_L;
		memcpy(&reply_cmd_buf.buf[2],&g_mode_config_data,sizeof(g_mode_config_data));

		usr_tcp_send_data(reply_cmd_buf.buf, sizeof(g_mode_config_data));
	}
}

void test(void)
{
	uint8_t test_data1[]={85,170,1,1,50,50,200,0,0,0};
	uint8_t test_data2[]={85,170,2,50,232,3,0,0,0,0};
	uint8_t test_data3[]={85,170,3,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

	memcpy(input_cmd_buf.buf,test_data2,8);
	input_cmd_buf.len=8;
	handle_cmd_data();
	memcpy(input_cmd_buf.buf,test_data3,15);
	input_cmd_buf.len=15;
	handle_cmd_data();
	memcpy(input_cmd_buf.buf,test_data1,8);
	input_cmd_buf.len=8;
	handle_cmd_data();
}

void sort(uint16_t *a, uint16_t len)
{
	uint32_t i=0;
	uint32_t j,t;
	
	for(i=0;i<len-1;i++)
	{
		for(j=0;j<len-i-1;j++)
		{
			if(a[j]>a[j+1])
			{
				t=a[j];
				a[j]=a[j+1];
				a[j+1]=t;
			}
		}
	}
}


