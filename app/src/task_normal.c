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

#include "task_normal_attribute.h"
#include "task_normal_interface.h"

#include "task_actuator_attribute.h"
#include "task_actuator_interface.h"

#include "task_menu_interface.h"
#include "task_menu_attribute.h"

/********************** macros and definitions *******************************/
#define G_TASK_SYS_CNT_INI			0ul
#define G_TASK_SYS_TICK_CNT_INI		0ul

#define DEL_SYS_XX_MIN				0ul
#define DEL_SYS_XX_MED				50ul
#define DEL_SYS_XX_MAX				500ul

////////////////////////// Estos son placeholders los tiene que levantar del set up menu ////////////////
#define MAX_TICK_ALARM				2000ul
#define MAX_TICK_SWITCH				10000ul
/********************** internal data declaration ****************************/

uint32_t tempAmb = 0;
uint32_t counter_tick= 0;

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

extern task_menu_set_up_dta_t task_menu_set_up;

task_menu_set_up_dta_t   *p_set_up_dta = &task_menu_set_up;

/********************** external functions definition ************************/
void task_normal_init(void *parameters)
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

void task_normal_update(void *parameters)
{
	task_system_dta_t *p_task_system_dta;
	bool b_time_update_required = false;

	uint32_t lm35_temp;
    char display_str[16];
    uint32_t temp_amb;

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
			// verifico que haya una task de system
			//if (true == p_task_system_dta->flag)
			{

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

					case EV_NORMAL_01_FAILURE: //Entra en estado de alarma
						 //comienza el tick de alarma y pasa al estado de alarma
						//guardo el tick anterior;
						counter_tick = p_task_system_dta->tick;
						p_task_system_dta->tick = p_set_up_dta->tiempo_reporta_falla;
						p_task_system_dta->state = ST_NORMAL_01_FAILURE;

						//da un mensaje al usuario de estado de alarma ? esto validar
						//displayCharPositionWrite(0,1);
						//displayStringWrite("UNUSUAL TEMPERATURE");
						break;
				/*
				 ////////////////////// Aca si tenemos el clock que pueda levantar este evento, si no se hace con ticks /////////////////////
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
					*/

					case EV_NORMAL_01_MONITOR:

						///////////////////////////////////// MANEJO DE DISPLAY ////////////////////////////////////////
						displayCharPositionWrite(0, 0);
						displayStringWrite("Ent/Nxt [*C]    ");

						if ( true == any_value_task_adc()){
							temp_amb=get_value_task_adc();}

						lm35_temp = (3.30 * 100 * temp_amb)/(4096);

						 displayCharPositionWrite(0,1);
						 snprintf(display_str, sizeof(display_str),"Tamb:%lu Tset:%lu ",lm35_temp,p_set_up_dta->set_point_temperatura);

						 displayStringWrite(display_str);

						//////////////////////// BLINK de leds y check counter //////////////
						if (p_task_system_dta->tick > 0){// si el tiempo de operacion no termino
							if (HAL_GPIO_ReadPin(LED_B_PORT, LED_B_PIN) == LED_B_ON){ //si el aire B esta prendido
								put_event_task_actuator(EV_LED_XX_BLINK,ID_AIRE_B); //hago parpadear la luz testigo de B

							}
							else{
								put_event_task_actuator(EV_LED_XX_BLINK,ID_AIRE_A);
							}
							p_task_system_dta->tick --;
						}
						else {
							//hago el switch de motores y reinicio el tiempo al seteado por el usuario
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
							p_task_system_dta->tick = p_set_up_dta->tiempo_conmuta;
						}

						break;
					case EV_NORMAL_O1_OFF:
						//apago todo y entro a estado de off;
						// Titilo las luces de usuario y apago todo lo demas //podemos comentar todo y decidir que el sistema ignore el modo manual en estado de alarma
						put_event_task_actuator(EV_LED_XX_OFF,ID_LED_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_LED_B);

						put_event_task_actuator(EV_LED_XX_OFF,ID_BUZZER_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_B);
						//cambio a estado standby
						p_task_system_dta->state = ST_NORMAL_01_OFF;
						break;

					default:

						break;
					}
			}
			break;

		case ST_NORMAL_01_FAILURE: // se detecto un rise de temperatura y se evalua si se mantiene
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
						if(p_task_system_dta->tick == 0){
							// prendo el buzzer
							put_event_task_actuator(EV_LED_XX_PULSE,ID_BUZZER_A);
							//switch de motores
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
							p_task_system_dta->tick = p_set_up_dta->tiempo_conmuta;
							p_task_system_dta->state = ST_NORMAL_01_MONITOR;

						}
						else{
							p_task_system_dta->tick --;
						}
						break;


					case EV_NORMAL_01_MONITOR: // baja la temperatura y se vuelve al estado normal de funcionamiento
						counter_tick -= (p_set_up_dta->tiempo_reporta_falla - p_task_system_dta->tick);
						p_task_system_dta->tick = counter_tick; //vuelvo a dejar el counter donde estaba antes de entrar a falla
						p_task_system_dta->state = ST_NORMAL_01_MONITOR;
						break;

					case EV_NORMAL_01_SWITCH_MOTOR://aca no deberia entrar en estado de alarma
						break;

					case EV_NORMAL_O1_OFF:
						//apago todo y entro a estado de off;
						// Titilo las luces de usuario y apago todo lo demas //podemos comentar todo y decidir que el sistema ignore el modo manual en estado de alarma
						put_event_task_actuator(EV_LED_XX_OFF,ID_LED_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_LED_B);

						put_event_task_actuator(EV_LED_XX_OFF,ID_BUZZER_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_B);
						//cambio a estado standby
						p_task_system_dta->state = ST_NORMAL_01_OFF;
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

					case EV_NORMAL_01_NEX_ACTIVE: // sigue en estado de setup
					case EV_NORMAL_01_ENT_ACTIVE:
						put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_A);
						put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_B);
						break;

					case EV_NORMAL_01_FAILURE: // no hago nada ya que se esta configurando los parametros de failure
						put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_A);
						put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_B);
						break;

					case EV_NORMAL_01_MONITOR: //esto puede ser es evento ON tambien hay que chequear si se puede obviar
						//reinicia el programa prendo el motor A y las luces correspondientes
						put_event_task_actuator(EV_LED_XX_ON,ID_LED_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_LED_B);

						put_event_task_actuator(EV_LED_XX_OFF,ID_BUZZER_A);

						put_event_task_actuator(EV_LED_XX_BLINK,ID_AIRE_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_B);
						//cambio a estado standby
						p_task_system_dta->tick = p_set_up_dta->tiempo_conmuta;
						p_task_system_dta->state = ST_NORMAL_01_MONITOR;

						break;

					/*
					case EV_NORMAL_01_SWITCH_MOTOR:
						break;
					*/

					case EV_NORMAL_O1_OFF:
						//apago todo y entro a estado de off;
						// Titilo las luces de usuario y apago todo lo demas //podemos comentar todo y decidir que el sistema ignore el modo manual en estado de alarma
						put_event_task_actuator(EV_LED_XX_OFF,ID_LED_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_LED_B);

						put_event_task_actuator(EV_LED_XX_OFF,ID_BUZZER_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_A);
						put_event_task_actuator(EV_LED_XX_OFF,ID_AIRE_B);
						//cambio a estado standby
						p_task_system_dta->state = ST_NORMAL_01_OFF;
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
