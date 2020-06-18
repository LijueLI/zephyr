#include <sys/printk.h>
#include <settings/settings.h>
#include <nrf.h>
#include <sys/byteorder.h>
#include <device.h>
#include <zephyr.h>

#include <drivers/gpio.h>
#include <stdio.h>
#include <logging/log.h>
#include <drivers/sensor.h>
#include <string.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/mesh.h>

#include "mesh.h"
#include "pherphs.h"

static struct device *dht22,*sw_device;
static u8_t button_press_cnt;
static struct sw sw;

static struct gpio_callback button_cb;

static u32_t time, last_time;

 struct onoff_state onoff_state[1];

/*
 * Map GPIO pins to button number
 * Change to select different GPIO input pins
 */

static uint8_t pin_to_sw(uint32_t pin_pos)
{
	switch (pin_pos) {
	case BIT(DT_ALIAS_SW0_GPIOS_PIN): return 0;
	}
	return 0;
}

static void button_pressed(struct device *dev, struct gpio_callback *cb,
			   u32_t pin_pos)
{
	/*
	 * One button press within a 1 second interval sends an on message
	 * More than one button press sends an off message
	 */

	time = k_uptime_get_32();

	/* debounce the switch */
	if (time < last_time + BUTTON_DEBOUNCE_DELAY_MS) {
		last_time = time;
		return;
	}

	if (button_press_cnt == 0U) {
		k_timer_start(&sw.button_timer, K_SECONDS(1), K_NO_WAIT);
	}

	printk("button_press_cnt 0x%02x\n", button_press_cnt);
	button_press_cnt++;

	/* The variable pin_pos is the pin position in the GPIO register,
	 * not the pin number. It's assumed that only one bit is set.
	 */

	sw.sw_num = pin_to_sw(pin_pos);
	last_time = time;
}

/*
 * Button Count Timer Worker
 */

static void button_cnt_timer(struct k_timer *work)
{
	struct sw *button_sw = CONTAINER_OF(work, struct sw, button_timer);

	button_sw->onoff_state = button_press_cnt == 1U ? 1 : 0;
	printk("button_press_cnt 0x%02x onoff_state 0x%02x\n",
	       button_press_cnt, button_sw->onoff_state);
	button_press_cnt = 0U;
	k_work_submit(&sw.button_work);
}

/*
 * Button Pressed Worker Task
 */

static void button_pressed_worker(struct k_work *work)
{
	SW *sw = CONTAINER_OF(work, SW, button_work);
    send_unack(sw->onoff_state);
}

void init_led(u8_t dev, const char *port, u32_t pin_num, gpio_flags_t flags)
{
    onoff_state[dev].led_gpio_pin = DT_ALIAS_LED0_GPIOS_PIN;
	onoff_state[dev].led_device = device_get_binding(port);
	gpio_pin_configure(onoff_state[dev].led_device, pin_num,
			   flags | GPIO_OUTPUT_INACTIVE);
}

void get_dht22_data(u16_t *temp, u16_t *humi){
    struct  sensor_value temperature, humidity;
    sensor_sample_fetch(dht22);
    sensor_channel_get(dht22, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
    sensor_channel_get(dht22,SENSOR_CHAN_HUMIDITY, &humidity);
    k_busy_wait(500);
    temp[0] = temperature.val1;
    temp[1] = temperature.val2/100000;
    humi[0] = humidity.val1;
    humi[1] = humidity.val2/100000;
    printk("%d %d %d %d\n", temp[0] , temp[1] , humi[0] , humi[1] );
}

int pheriphs_init(){

    const char *const label = DT_INST_0_AOSONG_DHT_LABEL;
	dht22 = device_get_binding(label);

    if(!dht22){
        printk("FAILED to get device binding\n");
        return -1;
    }
    /* Initialize the button debouncer */
	last_time = k_uptime_get_32();

	/* Initialize button worker task*/
	k_work_init(&sw.button_work, button_pressed_worker);

	/* Initialize button count timer */
	k_timer_init(&sw.button_timer, button_cnt_timer, NULL);

    sw_device = device_get_binding(DT_ALIAS_SW0_GPIOS_CONTROLLER);
	gpio_pin_configure(sw_device, DT_ALIAS_SW0_GPIOS_PIN,
			    GPIO_INPUT | DT_ALIAS_SW0_GPIOS_FLAGS);
     gpio_pin_interrupt_configure(sw_device, DT_ALIAS_SW0_GPIOS_PIN,
			     GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb, button_pressed,
			   BIT(DT_ALIAS_SW0_GPIOS_PIN) | BIT(DT_ALIAS_SW1_GPIOS_PIN) |
			   BIT(DT_ALIAS_SW2_GPIOS_PIN) | BIT(DT_ALIAS_SW3_GPIOS_PIN));
    gpio_add_callback(sw_device, &button_cb);
    init_led(0, DT_ALIAS_LED0_GPIOS_CONTROLLER, DT_ALIAS_LED0_GPIOS_PIN,
		 DT_ALIAS_LED0_GPIOS_FLAGS);
    return 0;
}