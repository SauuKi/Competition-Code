#include "KEY.h"

// 按键状态枚举
typedef enum {
	KEY_STATE_IDLE = 0,       // 空闲状态
	KEY_STATE_DEBOUNCE,       // 消抖中
	KEY_STATE_PRESSED,        // 已按下
	KEY_STATE_WAIT_RELEASE    // 等待释放
} KeyState_t;

// 按键对象结构体
typedef struct {
	GPIO_TypeDef* port;       // GPIO端口
	uint16_t pin;             // GPIO引脚
	KeyState_t state;         // 当前状态
	uint32_t debounce_time;   // 消抖开始时间
	uint8_t key_code;         // 按键返回码
} Key_t;

// 三个按键的配置
static Key_t keys[5] = {
	{GPIOG, GPIO_PIN_0, KEY_STATE_IDLE, 0, 1},
	{GPIOG, GPIO_PIN_1, KEY_STATE_IDLE, 0, 2},
	{GPIOE, GPIO_PIN_7, KEY_STATE_IDLE, 0, 3},
	{GPIOE, GPIO_PIN_8, KEY_STATE_IDLE, 0, 4},
	{GPIOE, GPIO_PIN_9, KEY_STATE_IDLE, 0, 5}
};

uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;
	
	for(uint8_t i = 0; i < 5; i++) {
		Key_t* key = &keys[i];
		GPIO_PinState pin_state = HAL_GPIO_ReadPin(key->port, key->pin);
		
		switch(key->state) {
			case KEY_STATE_IDLE:
				if(pin_state == GPIO_PIN_RESET) {
					key->debounce_time = HAL_GetTick();
					key->state = KEY_STATE_DEBOUNCE;
				}
				break;
				
			case KEY_STATE_DEBOUNCE:
				if(pin_state == GPIO_PIN_RESET) {
					if((HAL_GetTick() - key->debounce_time) >= KEY_DELAY_MS) {
						KeyNum = key->key_code;
						key->state = KEY_STATE_WAIT_RELEASE;
					}
				} else {
					key->state = KEY_STATE_IDLE;
				}
				break;
				
			case KEY_STATE_WAIT_RELEASE:
				if(pin_state == GPIO_PIN_SET) {
					key->state = KEY_STATE_IDLE;
				}
				break;
				
			default:
				key->state = KEY_STATE_IDLE;
				break;
		}
		
		if(KeyNum != 0) {
			break; // 只返回一个按键
		}
	}
	
	return KeyNum;
}

//typedef enum {
//    KEY_IDLE = 0,
//    KEY_DOWN,        // 检测到按下
//    KEY_WAIT_RELEASE // 等待松手
//} KeyState_t;

//typedef struct {
//    uint16_t  gpio_pin;
//    GPIO_TypeDef *port;
//    KeyState_t state;
//    uint8_t  key_code;
//    uint32_t down_time;   // 记录第一次按下时刻
//} KeyObj_t;

//static KeyObj_t keys[] = {
//    { GPIO_PIN_0, GPIOG, KEY_IDLE, 1, 0 },
//    { GPIO_PIN_1, GPIOG, KEY_IDLE, 2, 0 },
//    { GPIO_PIN_9, GPIOE, KEY_IDLE, 3, 0 },
//};

//uint8_t Key_ScanTask(void)      // 每 10 ms 被调度器调用一次
//{
////    static uint32_t last_tick = 0;
//    uint32_t now = sys_tick;   // 来自 1 ms 时基
//    uint8_t  key_ret = 0;

//    for (uint8_t i = 0; i < 3; ++i) {
//        KeyObj_t *k = &keys[i];
//        uint8_t  level = HAL_GPIO_ReadPin(k->port, k->gpio_pin);

//        switch (k->state) {
//        case KEY_IDLE:
//            if (level == 0) {              // 首次检测到按下
//                k->down_time = now;
//                k->state     = KEY_DOWN;
//            }
//            break;

//        case KEY_DOWN:
//            if (level != 0) {              // 抖动弹回
//                k->state = KEY_IDLE;
//            } else if ((now - k->down_time) >= KEY_DELAY_MS) {
//                // 持续 20 ms 仍按下，确认有效
//                key_ret      = k->key_code; // 只返回一次
//                k->state     = KEY_WAIT_RELEASE;
//            }
//            break;

//        case KEY_WAIT_RELEASE:
//            if (level != 0)                // 松手
//                k->state = KEY_IDLE;
//            break;
//        }
//    }
//    return key_ret;   // 0 表示无键，1/2/3 表示对应键被确认一次
//}
