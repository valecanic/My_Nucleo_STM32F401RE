/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t time2;

uint8_t stato;
unsigned int cont = 0;

uint16_t tempo;
uint16_t tempo_iniziale;
uint16_t tempo_finale;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern uint8_t full;

extern TIM_HandleTypeDef huart2;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void)
{
	/* USER CODE BEGIN TIM2_IRQn 0 */
	if ((stato == 3) || (stato == 4)) {
		// se lo stato era 3 o 4 ed è stato segnalato il TIM2 interrupt
		// passa allo stato 5
		stato = 5;
		// avviso Time Out del timer e stampa dello stato del LED
		HAL_UART_Transmit(&huart2, " \n\r TIME OUT! \n\r ", 21, 1000);
//		HAL_UART_Transmit(&huart2, " STATO : 4 \n\r ", 16, 1000);
		//		HAL_UART_Transmit(&huart2, "Premere RESET", 13, 1000);

	}
	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
	/* USER CODE BEGIN TIM2_IRQn 1 */

	/* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void)
{
	/* USER CODE BEGIN TIM3_IRQn 0 */
	if(cont < 10){
		HAL_UART_Transmit(&huart2, " \n\r LED blinking... \n\r ", 27, 1000);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		cont++;
	}
	else {
		// la variabile cont sarà 10 solo dopo i 5 lampeggi del LED
		// quindi si imposta il pin acceso e si ferma il TIM3

		// stop TIM3
		HAL_TIM_Base_Stop_IT(&htim3);

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_SET);
		HAL_UART_Transmit(&huart2, " \n\r LED ON \n\r ", 18, 1000);
		// si reimposta cont a 0, per poterlo riutilizzare al ritorno nello stato 2
		cont = 0;


		// TIM2 ON, Countdown 7s

		// avvio TIM2
		HAL_TIM_Base_Start_IT(&htim2);
		// clear del flag
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		// impostazione del valore del contatore a 0
		__HAL_TIM_SetCounter(&htim2, 0);

		// Avviso su UART dell'avvio del Timer, con avviso dei 7s
		HAL_UART_Transmit(&huart2, " \n\r Timer 2 Start \n\r ", 25, 1000);
		HAL_UART_Transmit(&huart2, " Premere il bottone per almeno 1,5s entro 7s \n\r ", 50, 1000);

		// quindi si imposta lo stato a 3
		stato = 3;
	}

	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */

	/* USER CODE END TIM3_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI15_10_IRQn 0 */

	if (stato == 0)
	{
		// Se lo stato era 0 (in cui il bottone non è premuto) e c'è stato un interrupt,
		// significa che il bottone è stato premuto e si entra nello stato 1.

		// si resetta l'eventuale full che indica timer pieno
		full = 0;

		// TIM2 viene resettato e avviato alla pressione del pulsante
		HAL_TIM_Base_Start_IT(&htim2);
		// clear del flag
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		// impostazione del valore del contatore a 0
		__HAL_TIM_SetCounter(&htim2, 0);

		// Avviso su UART dell'avvio del Timer
		HAL_UART_Transmit(&huart2, " \n\r Timer 2 Start \n\r ", 25, 1000);

		HAL_UART_Transmit(&huart2, " Pressione pulsante... \n\r ", 28, 1000);
		// stampa dello stato del LED
//		HAL_UART_Transmit(&huart2, " STATO : 1 \n\r ", 16, 1000);
		stato = 1;
	}
	else if (stato == 1)
	{
		// Se lo stato era 1 (dove il bottone è premuto) e c'è stato un interrupt,
		// significa che il bottone è stato rilasciato.

		// Il TIM2 viene fermato, misurando il tempo di pressione

		// stop TIM2 al rilascio del pulsante
		HAL_TIM_Base_Stop_IT(&htim2);
		// lettura del valore del timer
		time2=__HAL_TIM_GET_COUNTER(&htim2);

		// Avviso su UART dello stop del Timer
		HAL_UART_Transmit(&huart2, " \n\r Timer 2 Stop \n\r ", 24, 1000);

		// stampa su UART del risultato tramite buffer
		uint8_t buffertx[50]="";
		// calcolo del tempo di pressione in millisecondi
		tempo = time2*119/1000;	// 119 si ottiene moltiplicando il Prescaler (9999) per il periodo di CLK (1/84000000)
		sprintf(buffertx, "Tempo di pressione: %lu ms\n\r", (tempo));
		HAL_UART_Transmit(&huart2, buffertx, sizeof(buffertx), 1000);

		// setting prossimo stato
		if ((tempo >= 2000) || (full == 1)) {
			// se il tempo di pressione, misurato da TIM2, è > 2s
			// (o se il TIM è stato riempito, comunque > 2s)
			// resetta il full entra nello stato 2
			full = 0;
			HAL_UART_Transmit(&huart2, "Pressione superiore a 2 s. \n\r", 31, 1000);
			// stampa dello stato del LED
//			HAL_UART_Transmit(&huart2, "STATO : 2 \n\r", 14, 1000);

			// TIM3 viene resettato e avviato
			HAL_TIM_Base_Start_IT(&htim3);
			// clear del flag
			__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
			// impostazione del valore del contatore a 0
			__HAL_TIM_SetCounter(&htim3, 0);

			stato = 2;
		}
		else {
			HAL_UART_Transmit(&huart2, "Pressione inferiore a 2 s. \n\r", 31, 1000);
			// stampa dello stato del LED
//			HAL_UART_Transmit(&huart2, "STATO : 0 \n\r", 14, 1000);
			// reset full e impostazione stato a 0
			full = 0;
			stato = 0;
		}
	}
	else if (stato == 3) {
		// Se lo stato era 3 (dove il bottone non è premuto) e c'è stato un interrupt,
		// significa che il bottone è stato premuto.

		// si tiene conto del momento in cui è stato premuto
		// in riferimento al TIM2
		tempo_iniziale = __HAL_TIM_GET_COUNTER(&htim2);

		HAL_UART_Transmit(&huart2, "\n\r Pressione pulsante... \n\r", 31, 1000);
		// stampa dello stato del LED
//		HAL_UART_Transmit(&huart2, "STATO : 4 \n\r", 14, 1000);

		// setting stato a 4: il bottone è premuto durante i 7s di countdown.
		stato = 4;

	}
	else if (stato == 4) {
		// Se lo stato era 4 (dove il bottone è premuto) e c'è stato un interrupt,
		// significa che il bottone è stato rilasciato.

		// si tiene conto del momento di rilascio del bottone
		// in riferimento al TIM2
		tempo_finale = __HAL_TIM_GET_COUNTER(&htim2);
		// quindi si calcola il tempo di pressione tramite differenza
		// per poi convertirlo in millisecondi
		tempo = (tempo_finale - tempo_iniziale)*119/1000;

		// stampa su UART del risultato tramite buffer
		uint8_t buffertx[50]="";
		sprintf(buffertx, "Tempo di pressione: %lu ms\n\r", (tempo));
		HAL_UART_Transmit(&huart2, buffertx, sizeof(buffertx), 1000);


		if (tempo >= 1500) {
			// se il tempo di pressione è superiore a 1,5s
			// ferma il timer
			HAL_TIM_Base_Stop_IT(&htim2);
			// Avviso su UART dello stop del Timer
			HAL_UART_Transmit(&huart2, " \n\r Timer 2 Stop \n\r ", 24, 1000);

			HAL_UART_Transmit(&huart2, " Pressione superiore a 1,5 s. \n\r ", 35, 1000);

			// per tornare allo stato 0, il LED va spento
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_RESET);
			HAL_UART_Transmit(&huart2, " \n\r LED OFF \n\r ", 19, 1000);

			// stampa dello stato del LED
//			HAL_UART_Transmit(&huart2, " STATO : 0 \n\r ", 16, 1000);
			// quindi torna allo stato iniziale
			stato = 0;
		}
		else if (tempo < 1500) {
			// se il tempo di pressione è inferiore a 1,5s
			HAL_UART_Transmit(&huart2, " \n\r Pressione inferiore a 1,5 s. \n\r ", 40, 1000);
			// stampa dello stato del LED
//			HAL_UART_Transmit(&huart2, " STATO 3 \n\r ", 14, 1000);

			// ritorna allo stato 3
			stato = 3;
		}
	}



	/* USER CODE END EXTI15_10_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
	/* USER CODE BEGIN EXTI15_10_IRQn 1 */

	/* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
