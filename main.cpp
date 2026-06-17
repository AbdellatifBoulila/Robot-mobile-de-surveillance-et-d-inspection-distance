#include "stm32f10x.h"

#define THRESHOLD_CM 7       // Critical proximity threshold to force hard turns/reverse
#define SIDE_AVOID_CM 4      // Safety boundary margin for tight side walls
#define ALERT_ZONE_CM 9     // Look-ahead buffer to initiate smooth predictive veering

// Operational Modes
#define MODE_AUTO   0
#define MODE_MANUAL 1

// ================= PROTOTYPES =================
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void GPIO_Init_Custom(void);
void USART1_Init(void);
void USART1_SendChar(char c);
uint8_t USART1_Available(void);
char USART1_ReadChar(void);
uint8_t IR_Left(void);
uint8_t IR_Front(void);
uint8_t IR_Right(void);
uint8_t Cliff_Any(void);
uint32_t Get_Distance(uint8_t trig, uint8_t echo);
void Right_F(void);
void Right_B(void);
void Left_F(void);
void Left_B(void);
void Motors_Stop(void);
void Steer_Verve_Right(void);
void Steer_Verve_Left(void);
void Rotate_Left(void);
void Rotate_Right(void);
void Micro_Adjust_Left(void);
void Micro_Adjust_Right(void);
void Cliff_Escape(void);

// ================= DELAY =================
void delay_us(uint32_t us){
    for(volatile uint32_t i = 0; i < us * 8; i++) {
        __NOP();
    }
}

void delay_ms(uint32_t ms){
    for(uint32_t i = 0; i < ms; i++)
        delay_us(1200);
}

// ================= GPIO CONFIGURATION =================
void GPIO_Init_Custom(){
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

    // MOTOR PINS
    GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12 |
                    GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOB->CRH |= GPIO_CRH_MODE12_1 | GPIO_CRH_MODE12_0 |
                  GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0;

    GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8 |
                    GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
    GPIOA->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0 |
                  GPIO_CRH_MODE11_1 | GPIO_CRH_MODE11_0;

    GPIOA->BRR = (1<<8) | (1<<11);
    GPIOB->BRR = (1<<12) | (1<<13);

    // Configure PC13 as Output Push-Pull for onboard LED
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOC->CRH |= GPIO_CRH_MODE13_1;
    GPIOC->BSRR = (1<<13); // Default OFF

    // IR PINS
    GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 |
                    GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOA->CRL |= GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0;

    GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    GPIOB->CRL |= GPIO_CRL_CNF0_0;

    // ULTRASONIC PINS
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
                    GPIO_CRL_MODE2 | GPIO_CRL_CNF2 |
                    GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
    GPIOA->CRL |= GPIO_CRL_MODE0_1 | GPIO_CRL_MODE0_0 |
                  GPIO_CRL_MODE2_1 | GPIO_CRL_MODE2_0 |
                  GPIO_CRL_MODE4_1 | GPIO_CRL_MODE4_0;

    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1 |
                    GPIO_CRL_MODE3 | GPIO_CRL_CNF3 |
                    GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
    GPIOA->CRL |= GPIO_CRL_CNF1_0 | GPIO_CRL_CNF3_0 | GPIO_CRL_CNF5_0;
}

// ================= USART1 =================
void USART1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9 |
                    GPIO_CRH_MODE10 | GPIO_CRH_CNF10);

    GPIOA->CRH |= (GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0) | GPIO_CRH_CNF9_1;
    GPIOA->CRH |= GPIO_CRH_CNF10_0;

    USART1->BRR = 0x0271; // 115200 Baudrate at 72MHz
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void USART1_SendChar(char c)
{
    while(!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}

uint8_t USART1_Available(void)
{
    return (USART1->SR & USART_SR_RXNE) ? 1 : 0;
}

char USART1_ReadChar(void)
{
    return (char)USART1->DR;
}

// ================= IR SENSORS =================
uint8_t IR_Left() { return (GPIOA->IDR & (1<<6)) ? 1 : 0; }
uint8_t IR_Front(){ return (GPIOA->IDR & (1<<7)) ? 1 : 0; }
uint8_t IR_Right(){ return (GPIOB->IDR & (1<<0)) ? 1 : 0; }

uint8_t Cliff_Any(){
    return (IR_Left() || IR_Front() || IR_Right());
}

// ================= ULTRASONIC SENSORS =================
uint32_t Get_Distance(uint8_t trig, uint8_t echo)
{
    uint32_t t = 0;
    volatile uint32_t timeout;

    GPIOA->BRR = (1<<trig);
    delay_us(2);
    GPIOA->BSRR = (1<<trig);
    delay_us(10);
    GPIOA->BRR = (1<<trig);

    timeout = 40000; 
    while(!(GPIOA->IDR & (1<<echo)))
    {
        if(--timeout == 0)
            return 0;
    }

    timeout = 40000;
    while(GPIOA->IDR & (1<<echo))
    {
        t++;
        delay_us(1);
        if(--timeout == 0)
            break;
    }

    uint32_t cm = t / 58;
    if(cm == 0 || cm > 200)
        return 0;

    return cm;
}

// ================= MOTOR DRIVERS =================
void Right_F(){ GPIOB->BSRR = (1<<12); GPIOA->BRR = (1<<11); }
void Right_B(){ GPIOB->BRR = (1<<12); GPIOA->BSRR = (1<<11); }
void Left_F(){ GPIOA->BSRR = (1<<8); GPIOB->BRR = (1<<13); }
void Left_B(){ GPIOA->BRR = (1<<8); GPIOB->BSRR = (1<<13); }

void Motors_Stop(){
    GPIOB->BRR = (1<<12) | (1<<13);
    GPIOA->BRR = (1<<8) | (1<<11);
}

void Steer_Verve_Right()
{
    GPIOB->BRR = (1<<12); GPIOA->BRR = (1<<11);
    GPIOA->BSRR = (1<<8); GPIOB->BRR = (1<<13);
}

void Steer_Verve_Left()
{
    GPIOB->BSRR = (1<<12); GPIOA->BRR = (1<<11);
    GPIOB->BRR = (1<<8); GPIOB->BRR = (1<<13);
}

// ================= ROTATION EXECUTION =================
void Rotate_Left()
{
    Right_F();
    Left_B();
    delay_ms(180);
    Motors_Stop();
}

void Rotate_Right()
{
    Right_B();
    Left_F();
    delay_ms(180);
    Motors_Stop();
}

void Micro_Adjust_Left()
{
    Right_B();
    Left_F();
    delay_ms(40);
    Motors_Stop();
}

void Micro_Adjust_Right()
{
    Right_F();
    Left_B();
    delay_ms(40);
    Motors_Stop();
}

// ================= CRITICAL HAZARD MITIGATION =================
void Cliff_Escape(void)
{
    // INSTANT REACTENCE: Drop forward power and slam directly into reverse
    Right_B();
    Left_B();
    GPIOC->BRR = (1<<13); // Status LED alert indicator ON instantly

    delay_ms(350);        // Keep moving backwards to clear the danger zone
    Motors_Stop();
    delay_ms(100);

    if (IR_Left() && !IR_Right()) {
        Rotate_Right();
    } else {
        Rotate_Left();
    }
    
    Motors_Stop();
    GPIOC->BSRR = (1<<13); // Status LED reset OFF
    
    while(USART1_Available()) { (void)USART1_ReadChar(); }
    USART1_SendChar('0'); // RESTORED: Wakes up ESPCam after recovering from cliff edge
}

// ================= MAIN EXECUTION LOOP =================
int main()
{
    GPIO_Init_Custom();
    USART1_Init();
    Motors_Stop();

    delay_ms(1200);
    uint8_t current_mode = MODE_AUTO;

    while(1)
    {
        // ===== LEVEL 1: CRITICAL SAFETY CHECK =====
        if(Cliff_Any())
        {
            Cliff_Escape();
            continue;
        }

        // ===== LEVEL 2: UART PARSING ENGINE =====
        if(USART1_Available())
        {
            char rxChar = USART1_ReadChar();

            if(rxChar == 'A')
            {
                current_mode = MODE_AUTO;
                Motors_Stop();
            }
            else if(rxChar == 'M')
            {
                current_mode = MODE_MANUAL;
                Motors_Stop();
            }
            else if(current_mode == MODE_MANUAL)
            {
                switch(rxChar)
                {
                    case 'F':
                        Right_F(); Left_F();
                        delay_ms(30);
                        Motors_Stop();
                        break;
                    case 'B':
                        Right_B(); Left_B();
                        delay_ms(30);
                        Motors_Stop();
                        break;
                    case 'L':
                        Right_F(); Left_B();
                        delay_ms(30);
                        Motors_Stop();
                        break;
                    case 'R':
                        Right_B(); Left_F();
                        delay_ms(30);
                        Motors_Stop();
                        break;
                    case 'S':
                    default:
                        Motors_Stop();
                        break;
                }
            }
        }

        // ===== LEVEL 3: SMOOTH REACTIVE AUTOMATIC OBSTACLE AVOIDANCE =====
        if(current_mode == MODE_AUTO)
        {
            // 1. Fetch distances from sensors
            uint32_t front_raw = Get_Distance(0,1);
            delay_ms(15); // Added acoustic space delay to eliminate micro-stutter ghost waves
            uint32_t left_raw  = Get_Distance(2,3);
            delay_ms(15);
            uint32_t right_raw = Get_Distance(4,5);
            delay_ms(15);

            // 2. Treat 0 cm (timeout error) as a completely clear path (999 cm)
            uint32_t logic_front = (front_raw == 0) ? 999 : front_raw;
            uint32_t logic_left  = (left_raw  == 0) ? 999 : left_raw;
            uint32_t logic_right = (right_raw == 0) ? 999 : right_raw;

            // --- STRATEGY 1: BOXED-IN / DEAD END ---
            if((logic_front <= THRESHOLD_CM) && 
               (logic_left  <= ALERT_ZONE_CM) && 
               (logic_right <= ALERT_ZONE_CM))
            {
                Motors_Stop();
                delay_ms(100);
                
                Right_B(); Left_B();
                delay_ms(300); 
                Motors_Stop();
                delay_ms(100);

                if(logic_left >= logic_right) {
                    Rotate_Left();
                } else {
                    Rotate_Right();
                }
                continue; 
            }

            // --- STRATEGY 2: CRITICAL FRONT WALL ---
            else if(logic_front <= THRESHOLD_CM)
            {
                Motors_Stop();
                delay_ms(50);
                
                Right_B(); Left_B();
                delay_ms(150); 
                Motors_Stop();
                delay_ms(50);
                
                if(logic_left >= logic_right) {
                    Rotate_Left();
                } else {
                    Rotate_Right();
                }
                continue;
            }

            // --- STRATEGY 3: LEFT SIDE PROXIMITY ALERT (Active Counter-Steer Right) ---
            // Left wall encroaching: Left motor actively pushes Forward, Right motor actively forces Backward.
            // Generates true rotational physics to slide a heavy chassis around corners without stalling.
            else if(logic_left <= ALERT_ZONE_CM)
            {
                Left_F();
                Right_B();
            }

            // --- STRATEGY 4: RIGHT SIDE PROXIMITY ALERT (Active Counter-Steer Left) ---
            // Right wall encroaching: Right motor actively pushes Forward, Left motor actively forces Backward.
            else if(logic_right <= ALERT_ZONE_CM)
            {
                Right_F();
                Left_B();
            }

            // --- STRATEGY 5: NOMINAL FULL SPEED CRUISE ---
            // Clear sky conditions. Drive forward continuously without internal artificial loops.
            else
            {
                Right_F(); 
                Left_F();
            }
        }
        else
        {
            delay_ms(1); // Small power-saving anchor when running in Manual mode
        }
    }
}