/*
 * Copyright (c) 2020 LSI-TEC - Caninos Loucos
 * Author: Edgar Bernardi Righi <edgar.righi@lsitec.org.br>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <asm/system_misc.h>
#include <linux/mfd/core.h>
#include <linux/mfd/atc260x/atc260x.h>

#define PMU_SYS_CTL0_USB_WK_EN                BIT(15)
#define PMU_SYS_CTL0_WALL_WK_EN               BIT(14)
#define PMU_SYS_CTL0_ONOFF_LONG_WK_EN         BIT(13)
#define PMU_SYS_CTL0_ONOFF_SHORT_WK_EN        BIT(12)
#define PMU_SYS_CTL0_SGPIOIRQ_WK_EN           BIT(11)
#define PMU_SYS_CTL0_RESTART_EN               BIT(10)
#define PMU_SYS_CTL0_REM_CON_WK_EN            BIT(9)
#define PMU_SYS_CTL0_ALARM_WK_EN              BIT(8)
#define PMU_SYS_CTL0_HDSW_WK_EN               BIT(7)
#define PMU_SYS_CTL0_RESET_WK_EN              BIT(6)
#define PMU_SYS_CTL0_IR_WK_EN                 BIT(5)
#define PMU_SYS_CTL0_VBUS_WK_TH(x)            (((x) & 0x3) << 3)
#define PMU_SYS_CTL0_WALL_WK_TH(x)            (((x) & 0x3) << 1)
#define PMU_SYS_CTL0_ONOFF_MUXKEY_EN          BIT(0)

#define PMU_SYS_CTL0_WK_TH_VAL_405 (0x0)
#define PMU_SYS_CTL0_WK_TH_VAL_420 (0x1)
#define PMU_SYS_CTL0_WK_TH_VAL_435 (0x2)
#define PMU_SYS_CTL0_WK_TH_VAL_450 (0x3)

#define PMU_SYS_CTL1_USB_WK_FLAG              BIT(15)
#define PMU_SYS_CTL1_WALL_WK_FLAG             BIT(14)
#define PMU_SYS_CTL1_ONOFF_LONG_WK_FLAG       BIT(13)
#define PMU_SYS_CTL1_ONOFF_SHORT_WK_FLAG      BIT(12)
#define PMU_SYS_CTL1_SGPIOIRQ_WK_FLAG         BIT(11)
#define PMU_SYS_CTL1_ONOFF_PRESS_RST_IRQ_PD   BIT(10)
#define PMU_SYS_CTL1_REM_CON_WK_FLAG          BIT(9)
#define PMU_SYS_CTL1_ALARM_WK_FLAG            BIT(8)
#define PMU_SYS_CTL1_HDSW_WK_FLAG             BIT(7)
#define PMU_SYS_CTL1_RESET_WK_FLAG            BIT(6)
#define PMU_SYS_CTL1_IR_WK_FLAG               BIT(5)
#define PMU_SYS_CTL1_LOW_BAT_S4(x)            (((x) & 0x3) << 3)
#define PMU_SYS_CTL1_LOW_BAT_S4_EN            BIT(2)
#define PMU_SYS_CTL1_ENRTCOSC                 BIT(1)
#define PMU_SYS_CTL1_EN_S1                    BIT(0)

#define PMU_SYS_CTL1_LOW_BAT_VAL_29 (0x0)
#define PMU_SYS_CTL1_LOW_BAT_VAL_30 (0x1)
#define PMU_SYS_CTL1_LOW_BAT_VAL_31 (0x2)
#define PMU_SYS_CTL1_LOW_BAT_VAL_33 (0x3)

#define PMU_SYS_CTL2_ONOFF_PRESS              BIT(15)
#define PMU_SYS_CTL2_ONOFF_SHORT_PRESS        BIT(14)
#define PMU_SYS_CTL2_ONOFF_LONG_PRESS         BIT(13)
#define PMU_SYS_CTL2_ONOFF_INT_EN             BIT(12)
#define PMU_SYS_CTL2_ONOFF_PRESS_TIME(x)      (((x) & 0x3) << 10)
#define PMU_SYS_CTL2_ONOFF_PRESS_RST_EN       BIT(9)
#define PMU_SYS_CTL2_ONOFF_RESET_TIME_SEL(x)  (((x) & 0x3) << 7)
#define PMU_SYS_CTL2_S2_TIMER_EN              BIT(6)
#define PMU_SYS_CTL2_S2_TIMER(x)              (((x) & 0x7) << 3)
#define PMU_SYS_CTL2_ONOFF_PRESS_PD           BIT(2)
#define PMU_SYS_CTL2_ONOFF_PRESS_INT_EN       BIT(1)
#define PMU_SYS_CTL2_PMU_A_EN                 BIT(0)

static struct atc260x_dev *pmic = NULL;

static void atc260x_restart(enum reboot_mode reboot_mode, const char *cmd)
{
	atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL0, 0xE04B | PMU_SYS_CTL0_RESTART_EN);
	atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL1, 0xE);
	
	for(;;) { /* must never return */
		cpu_relax();
	}
}

static void atc260x_poweroff(void)
{
	atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL1, 0xE);
	
	for(;;) { /* must never return */
		cpu_relax();
	}
}

static int atc260x_system_control_setup(void)
{
	atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL0, 0xE04B);
	atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL1, 0xE | PMU_SYS_CTL1_EN_S1);
	atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL2, 0x680);
	atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL3, 0x80);
	atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL4, 0x80);
	atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL5, 0x180);
	return 0;
}

static int atc2603c_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	
	pmic = dev_get_drvdata(dev->parent);
	
	if (!pmic) {
		return -EINVAL;
	}
	
	atc260x_system_control_setup();
	
	pm_power_off = atc260x_poweroff;
	arm_pm_restart = atc260x_restart;
	
	dev_info(dev, "probe finished\n");
	return 0;
}

static const struct of_device_id atc2603c_poweroff_of_match[] = {
	{.compatible = "actions,atc2603c-poweroff",},
	{}
};
MODULE_DEVICE_TABLE(of, atc2603c_poweroff_of_match);

static struct platform_driver atc2603c_platform_driver = {
	.probe = atc2603c_platform_probe,
	.driver = {
		.name = "atc2603c-poweroff",
		.owner = THIS_MODULE,
		.of_match_table = atc2603c_poweroff_of_match,
	},
};

module_platform_driver(atc2603c_platform_driver);

MODULE_AUTHOR("Edgar Bernardi Righi <edgar.righi@lsitec.org.br>");
MODULE_DESCRIPTION("ATC2603C poweroff driver");
MODULE_LICENSE("GPL");

