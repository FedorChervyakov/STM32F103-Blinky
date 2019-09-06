#include <stm32f10x.h>

__IO int enable = 1; // State variable, if set -> blink, else don't
__IO uint32_t sysTickCounter;


/* SysTick initialization */
void SysTick_Init(void) {
  // Configure SysTick Interrupt to happen every 1 ms
  while (SysTick_Config(SystemCoreClock / 1000) != 0) {};
}

/* SysTick Interrupt Handler */
void SysTick_Handler(void) {
  --sysTickCounter;
}


/* EXTI0 Interrupt handler */
void EXTI0_IRQHandler(void) {
  EXTI->PR |= EXTI_PR_PR0; // Clear Pending Register (Acknowledge interrupt)
  enable ^= 1; // Toggle state
}


/* Delay function */
void delay(unsigned int mSecond) {
  sysTickCounter = mSecond;
  while (sysTickCounter != 0) {};
}


int main(void) {
  
//  __enable_irq();
  
  SysTick_Init(); // Enable sysTick interrupt
  
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Enable GPIO Port C clock
  GPIOC->CRH |= GPIO_CRH_MODE13_1; // Configure PC13 as output, max speed 2MHz
  GPIOC->CRH &= ~(GPIO_CRH_CNF13); // Configure PC13 as push-pull output
  
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable GPIO Port A clock
  GPIOA->CRL &= ~GPIO_CRL_MODE0; // Configure PA0 as input
  GPIOA->CRL &= ~GPIO_CRL_CNF0_0; // Configure PA0 as 
  GPIOA->CRL |= GPIO_CRL_CNF0_1;  // input with pull-up/down
  GPIOA->ODR |= GPIO_ODR_ODR0; // Enable pull-up resistor on PA0
  
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable clock for AFIO
  EXTI->IMR |= EXTI_IMR_MR0; // Unmask EXTI0 interrupt (Enable)
  EXTI->FTSR |= EXTI_FTSR_TR0; // Configure trigger on falling edge
  AFIO->EXTICR[1] |= AFIO_EXTICR1_EXTI0_PA; // Select PA to be a source input for EXTI0
  
  uint32_t prioritygroup = NVIC_GetPriorityGrouping(); 
  uint32_t priority = NVIC_EncodePriority(prioritygroup, 0, 0); // Define preempt and sub priorities to 0
  NVIC_SetPriority(EXTI0_IRQn, priority); // Set EXTI0 priority
  NVIC_EnableIRQ(EXTI0_IRQn); // Enable interrupt in NVIC

  
  while (1) { // Super-Loop
    if (enable) {
      GPIOC->BSRR = GPIO_BSRR_BR13; // Reset PC13, turn led on
      delay(500);
      GPIOC->BSRR = GPIO_BSRR_BS13; // Set PC13, turn led off
      delay(500);
    }
  }
}
