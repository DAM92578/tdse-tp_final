/*
 * Copyright (c) 2023 Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file   : task_adc.c
 * @date   : Set 26, 2023
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/
/* Project includes. */
#include "main.h"

/* Demo includes. */
#include "logger.h"
#include "dwt.h"

/* Application & Tasks includes. */
#include "board.h"
#include "app.h"
/********************** macros and definitions *******************************/
#define SAMPLES_COUNTER (10)
#define AVERAGER_SIZE (10)

/********************** internal data declaration ****************************/
uint32_t averaged;
volatile uint32_t sample_cnt=0;
uint32_t first_sample=1;
/********************** internal functions declaration ***********************/
/********************** internal data definition *****************************/
const char *p_task_adc 		= "Task ADC";
/********************** external data declaration *****************************/
extern ADC_HandleTypeDef hadc1;
volatile uint16_t sample_to_take ;
extern uint32_t temp_ambiente;
/********************** internal functions declaration ***********************/
/********************** external functions definition ************************/
void task_adc_init(void *parameters){
	/* Print out: Task Initialized */
	LOGGER_LOG("  %s is running - %s\r\n", GET_NAME(task_adc_init), p_task_adc);
    }

void task_adc_update(void *parameters){
	  uint16_t value;

	  if(0<first_sample){
		  HAL_ADC_Start_IT(&hadc1);
          first_sample=0;
          sample_to_take=1;
	  }

    if(sample_cnt< AVERAGER_SIZE ){

    	  if(0<sample_to_take){
    		  sample_to_take--;
		    value = HAL_ADC_GetValue(&hadc1);
		    HAL_ADC_Start_IT(&hadc1);
		    averaged += value;
		    sample_cnt++;
		    }
		}
		else{

			 averaged = averaged / AVERAGER_SIZE;
			 temp_ambiente= (averaged);//->pasar dato por cola.
			 averaged =0;
			 sample_cnt=0;
			 sample_to_take=0;
			 first_sample=1;
			 }
  return;
}

//	Requests start of conversion, waits until conversion done
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	// Check which version of the adc triggered this callback
	if (hadc == &hadc1){
		sample_to_take=1;}
      }

/********************** end of file ******************************************/
