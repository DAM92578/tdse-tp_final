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
 *
 * @file   : task_system.c
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
#include "display.h"

#include "task_adc_interface.h"
//#include "task_adc_attribute.h"

#include "task_normal_attribute.h"
#include "task_normal_interface.h"

#include "task_actuator_attribute.h"
#include "task_actuator_interface.h"

#include "task_menu_interface.h"

/********************** macros and definitions *******************************/
#define G_TASK_SYS_CNT_INI			0ul
#define G_TASK_SYS_TICK_CNT_INI		0ul

#define DEL_SYS_XX_MIN				0ul
#define DEL_SYS_XX_MED				50ul
#define DEL_SYS_XX_MAX				500ul
#define MAX_TICK_ALARM				2000ul

/********************** internal data declaration ****************************/

uint32_t tempAmb = 0;


task_system_dta_t task_system_dta =
	{DEL_SYS_XX_MIN, ST_NORMAL_01_MONITOR , EV_NORMAL_01_MONITOR, false};

#define SYSTEM_DTA_QTY	(sizeof(task_system_dta)/sizeof(task_system_dta_t))

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/
const char *p_task_system 		= "Task System (System Statechart)";
const char *p_task_system_ 		= "Non-Blocking & Update By Time Code";

/********************** external data declaration ****************************/
uint32_t g_task_system_cnt;
volatile uint32_t g_task_system_tick_cnt;

/********************** external functions definition ************************/
void task_system_init(void *parameters)
{
	task_system_dta_t 	*p_task_system_dta;
	task_system_st_t	state;
	task_system_ev_t	event;
	bool b_event;

	/* Print out: Task Initialized */
	LOGGER_LOG("  %s is running - %s\r\n", GET_NAME(task_system_init), p_task_system);
	LOGGER_LOG("  %s is a %s\r\n", GET_NAME(task_system), p_task_system_);

	g_task_system_cnt = G_TASK_SYS_CNT_INI;

	/* Print out: Task execution counter */
	LOGGER_LOG("   %s = %lu\r\n", GET_NAME(g_task_system_cnt), g_task_system_cnt);

	init_queue_event_task_system();

	/* Update Task Actuator Configuration & Data Pointer */
	p_task_system_dta = &task_system_dta;

	/* Print out: Task execution FSM */
	state = p_task_system_dta->state;
	LOGGER_LOG("   %s = %lu", GET_NAME(state), (uint32_t)state);

	event = p_task_system_dta->event;
	LOGGER_LOG("   %s = %lu", GET_NAME(event), (uint32_t)event);

	b_event = p_task_system_dta->flag;
	LOGGER_LOG("   %s = %s\r\n", GET_NAME(b_event), (b_event ? "true" : "false"));

	g_task_system_tick_cnt = G_TASK_SYS_TICK_CNT_INI;
}

void task_system_update(void *parameters)
{
	task_system_dta_t *p_task_system_dta;
	bool b_time_update_required = false;

	/* Update Task System Counter */
	g_task_system_cnt++;

	/* Protect shared resource (g_task_system_tick) */
	__asm("CPSID i");	/* disable interrupts*/
    if (G_TASK_SYS_TICK_CNT_INI < g_task_system_tick_cnt)
    {
    	g_task_system_tick_cnt--;
    	b_time_update_required = true;
    }
    __asm("CPSIE i");	/* enable interrupts*/

    while (b_time_update_required)
    {
		/* Protect shared resource (g_task_system_tick) */
		__asm("CPSID i");	/* disable interrupts*/
		if (G_TASK_SYS_TICK_CNT_INI < g_task_system_tick_cnt)
		{
			g_task_system_tick_cnt--;
			b_time_update_required = true;
		}
		else
		{
			b_time_update_required = false;
		}
		__asm("CPSIE i");	/* enable interrupts*/
		/* Update Task System Data Pointer */
		p_task_system_dta = &task_system_dta;

		if (true == any_event_task_system())
		{
			p_task_system_dta->flag = true;
			p_task_system_dta->event = get_event_task_system();
		}


		switch (p_task_system_dta->state)
		{
		/********************************************************************************************/
		case ST_NORMAL_01_MONITOR:
			if (true == p_task_system_dta->flag){ // verifico que haya una task de system

				switch(p_task_system_dta->event){

					case EV_NORMAL_01_NEX_ACTIVE: // si se presionan cualquiera de los botones del display en monitor pasa a setup
					case EV_NORMAL_01_ENT_ACTIVE:

						// Titilo las luces de usuario y apago todo lo demas
						put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_A);
						put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_B);

						put_event_task_actuator(EV_LED_XX_OFF,ID_BUZZER_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_B);
						//cambio a estado standby
						p_task_system_dta->state = ST_NORMAL_01_STANDBY;

						break;

					case EV_NORMAL_01_FAILURE:
					case EV_NORMAL_01_ALARM_MONITOR: //Entra en estado de alarma
						 //comienza el tick de alarma y pasa al estado de alarma
						task_system_dta->tick = MAX_TICK_ALARM;
						p_task_system_dta->state = ST_NORMAL_01_ALARM;

						//da un mensaje al usuario de estado de alarma ? esto validar
						//displayCharPositionWrite(0,1);
						//displayStringWrite("UNUSUAL TEMPERATURE");
						break;

					case EV_NORMAL_01_SWITCH_MOTOR: // si entra aca desde el st_monitor es porque se termino el tiempo cambio de motores
						//resetea el clock de cambio de motor
						if (HAL_GPIO_ReadPin(LED_B_PORT, LED_B_PIN) == LED_B_ON ){ //si el aire B esta prendido

							put_event_task_actuator(EV_LED_XX_OFF,ID_LED_B);
							put_event_task_actuator(EV_LED_XX_ON,ID_LED_A);

							put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_B);
							put_event_task_actuator(EV_LED_XX_BLINK,ID_AIRE_A);
						}
						else{
							put_event_task_actuator(EV_LED_XX_ON,ID_LED_B);
							put_event_task_actuator(EV_LED_XX_OFF,ID_LED_A);

							put_event_task_actuator(EV_LED_XX_BLINK,ID_AIRE_B);
							put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_A);
						}
						break;

					case EV_NORMAL_01_MONITOR:
						displayCharPositionWrite(0, 0);
						displayStringWrite("Time: 0:5:23 Tmicro: 23°C"); /// aca levantar la temperatura y el clock

						displayCharPositionWrite(0,1);
						displayStringWrite("Tamb: 0:5:23 Tset: 23°C");
						//clock_tick ++ en el caso de setear un clock a mano
						if (HAL_GPIO_ReadPin(LED_B_PORT, LED_B_PIN) == LED_B_ON ){ //si el aire B esta prendido
							put_event_task_actuator(EV_LED_XX_BLINK,ID_AIRE_B); //hago parpadear la luz testigo de B

						}
						else{
							put_event_task_actuator(EV_LED_XX_BLINK,ID_AIRE_A);
						}
						break;

					default:

						break;
					}
			}
			break;

		case ST_NORMAL_01_ALARM: // se detecto un rise de temperatura y se evalua si se mantiene
			if (true == p_task_system_dta->flag){ // verifico que haya una task de system

				switch(p_task_system_dta->event){

					case EV_NORMAL_01_NEX_ACTIVE: //en estado de alarma si se quieren cambiar los parametros de operacion
					case EV_NORMAL_01_ENT_ACTIVE:
						// Titilo las luces de usuario y apago todo lo demas //podemos comentar todo y decidir que el sistema ignore el modo manual en estado de alarma
						put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_A);
						put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_B);

						put_event_task_actuator(EV_LED_XX_OFF,ID_BUZZER_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_B);
						//cambio a estado standby
						p_task_system_dta->state = ST_NORMAL_01_STANDBY;
						break;

					case EV_NORMAL_01_FAILURE:
						break;

					case EV_NORMAL_01_ALARM_MONITOR:
						break;

					case EV_NORMAL_01_MONITOR: // baja la temperatura y se vuelve al estado normal de funcionamiento
						p_task_system_dta->state = ST_NORMAL_01_MONITOR;
						break;

					case EV_NORMAL_01_SWITCH_MOTOR://aca no deberia entrar en estado de alarma
						break;

					default:

						break;
					}
			}
			break;
		case ST_NORMAL_01_FAILURE:
			if (true == p_task_system_dta->flag){ // verifico que haya una task de system

				switch(p_task_system_dta->event){

					case EV_NORMAL_01_NEX_ACTIVE:
						break;

					case EV_NORMAL_01_ENT_ACTIVE:
						break;

					case EV_NORMAL_01_FAILURE:
						break;

					case EV_NORMAL_01_ALARM_MONITOR:
						break;

					case EV_NORMAL_01_MONITOR:
						break;

					case EV_NORMAL_01_SWITCH_MOTOR:
						break;

					default:

					break;
				}
			}
			break;
			/*********************************************************************************************/
		case ST_NORMAL_01_STANDBY:
			if (true == p_task_system_dta->flag){ // verifico que haya una task de system

				switch(p_task_system_dta->event){

					case EV_NORMAL_01_NEX_ACTIVE:
						break;

					case EV_NORMAL_01_ENT_ACTIVE:
						break;

					case EV_NORMAL_01_FAILURE:
						break;

					case EV_NORMAL_01_ALARM_MONITOR:
						break;

					case EV_NORMAL_01_MONITOR:
						break;

					case EV_NORMAL_01_SWITCH_MOTOR:
						break;

					default:

					break;
				}
			}
			default:

				break;
		}
	}
}

/********************** end of file ******************************************/
