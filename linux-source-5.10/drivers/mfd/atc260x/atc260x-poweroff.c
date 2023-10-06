/*
 * Copyright (c) 2023 LSI-TEC - Caninos Loucos
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

static struct atc260x_dev *pmic = NULL;

static void atc260x_restart(enum reboot_mode reboot_mode, const char *cmd)
{
	int ret;
	
	/* set bit 10 to set restart flag */
	ret = atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL0, 0xE04B | BIT(10));
	
	if (ret < 0) {
		pr_err("unable to set restart flag on PMU_SYS_CTL0 register\n");
	}
	else {
		ret = atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL1, 0xE);
	}
	
	if (ret < 0) {
		pr_err("restart failed\n");
	}
	for(;;) { /* must never return */
		msleep(100);
	}
}

static void atc260x_poweroff(void)
{
	if (atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL1, 0xE) < 0) {
		pr_err("poweroff failed\n");
	}
	for(;;) { /* must never return */
		msleep(100);
	}
}

static int atc260x_system_control_setup(struct device *dev)
{
	int ret;
	
	ret = atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL0, 0xE04B);
	
	if (ret < 0) {
		dev_err(dev, "unable to write to PMU_SYS_CTL0 register\n");
		return ret;
	}
	
	/* set bit 0 to enable s1 power state */
	ret = atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL1, 0xE | BIT(0));
	
	if (ret < 0) {
		dev_err(dev, "unable to write to PMU_SYS_CTL1 register\n");
		return ret;
	}
	
	ret = atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL2, 0x680);
	
	if (ret < 0) {
		dev_err(dev, "unable to write to PMU_SYS_CTL2 register\n");
		return ret;
	}
	
	ret = atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL3, 0x80);
	
	if (ret < 0) {
		dev_err(dev, "unable to write to PMU_SYS_CTL3 register\n");
		return ret;
	}
	
	ret = atc260x_reg_write(pmic, ATC2603C_PMU_SYS_CTL5, 0x180);
	
	if (ret < 0) {
		dev_err(dev, "unable to write to PMU_SYS_CTL5 register\n");
		return ret;
	}
	
	return 0;
}

static int atc2603c_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;
	
	pmic = dev_get_drvdata(dev->parent);
	
	if (!pmic) {
		return -EINVAL;
	}
	
	ret = atc260x_system_control_setup(dev);
	
	if (ret < 0) {
		return ret;
	}
	
	pm_power_off = atc260x_poweroff;
	arm_pm_restart = atc260x_restart;
	
	dev_info(dev, "probe finished\n");
	return 0;
}

static int atc2603c_platform_remove(struct platform_device *pdev)
{
	pmic = NULL;
	return 0;
}

static const struct of_device_id atc2603c_poweroff_of_match[] = {
	{.compatible = "actions,atc2603c-poweroff",},
	{}
};
MODULE_DEVICE_TABLE(of, atc2603c_poweroff_of_match);

static struct platform_driver atc2603c_platform_driver = {
	.probe = atc2603c_platform_probe,
	.remove = atc2603c_platform_remove,
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

