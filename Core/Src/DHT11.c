#include "DHT11.h"

extern TIM_HandleTypeDef htim1;

// us단위의 지연시간
void DelayUs(uint32_t us) {
	// 타이머 1번을 0으로 초기화
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	// 타이머 1번이 1MHz의 클럭을 입력 받아 증가할때 us보다 커지면 루프 중단
	while(__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void checkPinWithTimeoutUs(uint32_t us, int value) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while((__HAL_TIM_GET_COUNTER(&htim1) < us) && value);
	//while(value);
}

void DHT11_Start() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	// output mode
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, 0);
  DelayUs(18000);
  // input mode
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

uint8_t DHT11_CheckResponse() {
	uint8_t	Response = 0;
	DelayUs(40);
	if(!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)) {
		DelayUs(80);
		if(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)) Response = 1;
		else Response = 0;
	}
	//checkPinWithTimeoutUs(200, HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin));
	while(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin));
	return Response;
}

uint8_t DHT11_Read() {
	uint8_t i, j;
	for(j = 0; j < 8; j++) {
		//checkPinWithTimeoutUs(200, !HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin));
		while(!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin));
		DelayUs(40);
		if(!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)) {
			i &= ~(1 << (7 - j));
		}
		else
			i |= (1 << (7 - j));
		//checkPinWithTimeoutUs(200, HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin));
		while(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin));
	}
	return i;
}
