
#define BUTTON_DEBOUNCE_DELAY_MS 250

struct onoff_state {
	u8_t current;
	u8_t previous;
	u8_t led_gpio_pin;
	struct device *led_device;
};
typedef struct sw {
	u8_t sw_num;
	u8_t onoff_state;
	struct k_work button_work;
	struct k_timer button_timer;
}SW;

extern struct onoff_state onoff_state[1];

int pheriphs_init(void);
void get_dht22_data(u16_t *temp,u16_t *humi);
