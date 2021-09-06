/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <linux/gpio.h>
#include <linux/gpio/machine.h>

#include <linux/i2c.h>
#include <linux/platform_data/pca953x.h>
#include <linux/platform_data/i2c-pxa.h>

#include <plat/una10.h>


/* On board MBM1.0 we have such GPIO blocks 

--w-------    1 root  root   4096 Jan  1  1970 export
lrwxrwxrwx    1 root  root      0 Jan  1  1970 gpiochip24 -> ../../devices/platform/soc/20200000.gpio/gpio/gpiochip24
lrwxrwxrwx    1 root  root      0 Jan  1  1970 gpiochip56 -> ../../devices/platform/soc/20250000.i2c0/i2c-0/0-0050/gpio/gpiochip56
--w-------    1 root  root   4096 Jan  1  1970 unexport

If we do not have some another GPIO chips on UNA1.0 board ,we can set for it the gpio_base param for subsequent GPIO blocks,
starting at a value of 88 = 56(gpio_base parameter for gpiochip56) + 32 or 56 in case when no GPIO units are connected
to I2C bus with number 0.
*/

/******************************************************************************
 * LED
 ******************************************************************************/
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
static struct gpio_led una10_leds[] = {
  {
    .name               = "status:red",
    .default_trigger    = "none",
    .gpio               = 0,
    .active_low         = 1,
  },
  {
    .name               = "status:green",
    .default_trigger    = "none",
    .gpio               = 1,
    .active_low         = 1,
  },
  {
    .name               = "status:yellow",
    .default_trigger    = "none",
    .gpio               = 2,
    .active_low         = 1,
  },
  { 
    .name               = "1G-combo:port_selector_1",
    .default_trigger    = "heartbeat",
    .gpio               = 15,
    .active_low         = 1,
  },
  { 
    .name               = "1G-combo:port_selector_2",
    .default_trigger    = "heartbeat",
    .gpio               = 16,
    .active_low         = 1,
  },
};

static struct gpio_led_platform_data una10_gpio_led_pdata = {
  .num_leds = ARRAY_SIZE(una10_leds),
  .leds = una10_leds,
};

static struct platform_device una10_led_device = {
	.name		= "una10-leds-gpio",
	.id		= -1,
	.dev		= {
	  .platform_data = &una10_gpio_led_pdata,
	},
};
static void __init una10_init_leds(void) {
	//if (system_rev < 130) cm_x300_leds[0].gpio = 79;
	//else cm_x300_leds[0].gpio = 76;
	platform_device_register(&una10_led_device);
}

#else

static inline void una10_init_leds(void) {}

#endif

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

static struct pca953x_platform_data una10_gpio_ext_pdata_0 = {
  .gpio_base = 88,
  .irq_base = -1,
};

static struct pca953x_platform_data una10_gpio_ext_pdata_1 = {
  .gpio_base = -1,
  .irq_base = -1,
};

static struct pca953x_platform_data una10_i2c_ext_pdata_1 = {
  .gpio_base = -1,
  .irq_base = -1,
};

static struct pca953x_platform_data una10_abrtcmc_ext_pdata_1 = {
  .gpio_base = -1,
  .irq_base = -1,
};

static struct pca953x_platform_data una10_at24cs04_ext_pdata_1 = {
  .gpio_base = -1,
  .irq_base = -1,
};

static struct i2c_board_info una10_gpio_ext_info1[] = {
  [0] = {
    I2C_BOARD_INFO("pca9555", PCA9555_ADDR01),
    .platform_data = &una10_gpio_ext_pdata_0,
  },

  [1] = {
    I2C_BOARD_INFO("pca9555", PCA9555_ADDR02),
    .platform_data = &una10_gpio_ext_pdata_1,
  },

  [2] = {
    I2C_BOARD_INFO("pca9546", PCA9546_ADDR01),
    .platform_data = &una10_i2c_ext_pdata_1,
  },

  [3] = {  
    I2C_BOARD_INFO("ab-rtcmc", AB_RTCMC_ADDR01),
    .platform_data = &una10_abrtcmc_ext_pdata_1,          
  },
};

static struct i2c_board_info una10_gpio_ext_info2[] = {
  [0] = {
    I2C_BOARD_INFO("at24cs04", AT24CS04_ADDR01),
    .platform_data = &una10_at24cs04_ext_pdata_1,
  },
};

static void __init una10_init_i2c(void)
{

  pxa_set_i2c_info(NULL);

  i2c_register_board_info(1, una10_gpio_ext_info1, ARRAY_SIZE(una10_gpio_ext_info1));
  i2c_register_board_info(2, una10_gpio_ext_info2, ARRAY_SIZE(una10_gpio_ext_info2));

}
#else
static inline void una10_init_i2c(void)
{
}
#endif

static inline void una10_init_bl() {}
 
static void __init una10_init(void)
{
  una10_init_leds();
  una10_init_i2c();
  una10_init_bl();
}

MACHINE_START(UNA10, "UNA10 module")
	//.atag_offset	= 0x100,
        .reboot_mode	= REBOOT_SOFT,
	//.map_io		= pxa3xx_map_io,
	//.nr_irqs	= PXA_NR_IRQS,
	//.init_irq	= pxa3xx_init_irq,
	//.handle_irq	= pxa3xx_handle_irq,
	//.init_time	= pxa_timer_init,
	.init_machine	= una10_init,               //cm_x300_init,
	//.restart	= una10_restart,
MACHINE_END

