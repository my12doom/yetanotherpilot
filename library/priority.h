// call NVIC_PriorityGroupConfig only once, in main().
// any new module copied from 3rd sources must check for NVIC_PriorityGroupConfig() calls and remove them
// use NVIC_PriorityGroup_3, which means 0~7 pre-emption priority and 0~1 subpriority
// the following priority is described as (pre-emption,sub)
// time - critical interrupts use priority (0~1, 0~1): 
//    PPM recieving(0,0)
//    sonar recieving (1,1)
//    TIM5 for timing (0,1)
//

// non-time-critical ISR use priority (2~3, 0~1)
//     SDIO: (2,0~1)
//     USB: (3,0~1)
//     USART1: (3,1)
//     NRF24L01: (3,1)

// main loop(TIM7) use priority (4,0)

// logging task(TIM12) use priority (6,1)