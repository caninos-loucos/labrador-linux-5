// SPDX-License-Identifier: GPL-2.0
/*
 * Caninos Labrador DWMAC specific glue layer
 * Copyright (c) 2019-2020 LSI-TEC - Caninos Loucos
 * Edgar Bernardi Righi <edgar.righi@lsitec.org.br>
 * Igor Ruschi Andrade E Lima <igor.lima@lsitec.org.br>
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

#include <linux/stmmac.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/of_net.h>
#include <linux/of_gpio.h>

#include "stmmac_platform.h"

struct caninos_priv_data {
	
	struct plat_stmmacenet_data *plat_dat;
	struct device *dev;
	struct clk *ref_clk, *eth_clk;
	int power_gpio;
	int reset_gpio;
};

static int caninos_gmac_get_gpios(struct caninos_priv_data *gmac)
{
	struct device *dev = gmac->dev;
	int ret;
	
	gmac->reset_gpio = of_get_named_gpio(dev->of_node, "phy-reset-gpio", 0);
	
	if (!gpio_is_valid(gmac->reset_gpio)) {
		dev_err(dev, "unable to get reset gpio\n");
		return -ENODEV;
	}
	
	gmac->power_gpio = of_get_named_gpio(dev->of_node, "phy-power-gpio", 0);
	
	if (!gpio_is_valid(gmac->power_gpio)) {
		dev_err(dev, "unable to get power gpio\n");
		return -ENODEV;
	}
	
	ret = devm_gpio_request(dev, gmac->reset_gpio, "phy_reset");
	
	if (ret) {
		dev_err(dev, "unable to request reset gpio\n");
		return ret;
	}
	
	ret = devm_gpio_request(dev, gmac->power_gpio, "phy_power");
	
	if (ret) {
		dev_err(dev, "unable to request power gpio\n");
		return ret;
	}
	
	return 0;
}

static void caninos_gmac_fix_speed(void *priv, unsigned int speed)
{
	struct caninos_priv_data *gmac = priv;
	
	if (speed == 1000U) {
		clk_set_rate(gmac->ref_clk, 125000000);
	}
	else {
		clk_set_rate(gmac->ref_clk, 50000000);
	}
}

static void caninos_gmac_interface_fini(struct caninos_priv_data *gmac)
{
	gpio_set_value(gmac->reset_gpio, 0);
	gpio_set_value(gmac->power_gpio, 0); /* power off the phy */
}

static int caninos_gmac_interface_init(struct caninos_priv_data *gmac)
{
	struct device *dev = gmac->dev;
	void __iomem *addr;
	int ret = 0;
	
	addr = ioremap(0xe024c0a0, 4);
	
	if (!addr) {
		dev_err(dev, "unable to map interface config reg\n");
		return -ENOMEM;
	}
	
	switch (gmac->plat_dat->phy_interface)
	{
	case PHY_INTERFACE_MODE_RGMII:
		writel(0x1, addr);
		break;
		
	case PHY_INTERFACE_MODE_RMII:
		writel(0x4, addr);
		break;
		
	default:
		dev_err(dev, "invalid phy interface\n");
		iounmap(addr);
		return -EINVAL;
	}
	
	/* power up the phy */
	gpio_direction_output(gmac->reset_gpio, 1);
	gpio_direction_output(gmac->power_gpio, 1);
	msleep(50); /* time for power up */
	
	/* reset the phy */
	gpio_set_value(gmac->reset_gpio, 0);
	usleep_range(12000, 15000); /* time for reset */
	gpio_set_value(gmac->reset_gpio, 1);
	msleep(150); /* time required to access registers */
	
	iounmap(addr);
	return ret;
}

static void caninos_gmac_enable_clocks(struct caninos_priv_data *gmac)
{
	clk_prepare_enable(gmac->eth_clk);
	clk_prepare_enable(gmac->ref_clk);
	clk_set_rate(gmac->ref_clk, 50000000);
}

static void caninos_gmac_disable_clocks(struct caninos_priv_data *gmac)
{
	clk_disable_unprepare(gmac->ref_clk);
	clk_disable_unprepare(gmac->eth_clk);
}

static int caninos_gmac_probe(struct platform_device *pdev)
{
	struct stmmac_resources stmmac_res;
	struct device *dev = &pdev->dev;
	struct caninos_priv_data *gmac;
	int ret;
	
	gmac = devm_kzalloc(dev, sizeof(*gmac), GFP_KERNEL);
	
	if (!gmac) {
		return -ENOMEM;
	}
	
	gmac->dev = dev;
	
	gmac->ref_clk = devm_clk_get(dev, "rmii");
	
	if (IS_ERR(gmac->ref_clk))
	{
		dev_err(dev, "unable to get ref clock\n");
		return PTR_ERR(gmac->ref_clk);
	}
	
	gmac->eth_clk = devm_clk_get(dev, "eth");
	
	if (IS_ERR(gmac->eth_clk))
	{
		dev_err(dev, "unable to get eth clock\n");
		return PTR_ERR(gmac->eth_clk);
	}
	
	ret = caninos_gmac_get_gpios(gmac);
	
	if (ret) {
		return ret;
	}
	
	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	
	if (ret) {
		dev_err(dev, "stmmac_get_platform_resources routine failed\n");
		return ret;
	}
	
	caninos_gmac_enable_clocks(gmac);
	
	gmac->plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	
	if (IS_ERR(gmac->plat_dat)) {
		dev_err(dev, "stmmac_probe_config_dt routine failed\n");
		caninos_gmac_disable_clocks(gmac);
		return PTR_ERR(gmac->plat_dat);
	}
	
	gmac->plat_dat->tx_coe = 0;
	gmac->plat_dat->riwt_off = 1; /* disable Rx Watchdog */
	gmac->plat_dat->bsp_priv = gmac;
	gmac->plat_dat->fix_mac_speed = caninos_gmac_fix_speed;
	gmac->plat_dat->maxmtu = 1518;
	gmac->plat_dat->has_gmac = 1;
	gmac->plat_dat->pmt = 1;
	gmac->plat_dat->force_thresh_dma_mode = 1;
	gmac->plat_dat->tx_fifo_size = 16384;
	gmac->plat_dat->rx_fifo_size = 16384;
	gmac->plat_dat->clk_csr = 0x4;
	
	ret = caninos_gmac_interface_init(gmac);
	
	if (ret) {
		stmmac_remove_config_dt(pdev, gmac->plat_dat);
		caninos_gmac_disable_clocks(gmac);
		return ret;
	}
	
	ret = stmmac_dvr_probe(dev, gmac->plat_dat, &stmmac_res);
	
	if (ret) {
		caninos_gmac_interface_fini(gmac);
		stmmac_remove_config_dt(pdev, gmac->plat_dat);
		caninos_gmac_disable_clocks(gmac);
	}
	return ret;
}

static int caninos_gmac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct plat_stmmacenet_data *plat = priv->plat;
	int ret;
	
	ret = stmmac_dvr_remove(&pdev->dev);
	caninos_gmac_interface_fini(plat->bsp_priv);
	stmmac_remove_config_dt(pdev, plat);
	caninos_gmac_disable_clocks(plat->bsp_priv);
	return ret;
}

static const struct of_device_id caninos_dwmac_match[] = {
	{.compatible = "caninos,k7-gmac" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, caninos_dwmac_match);

static struct platform_driver caninos_dwmac_driver = {
	.probe  = caninos_gmac_probe,
	.remove = caninos_gmac_remove,
	.driver = {
		.name = "caninos-dwmac",
		.pm = &stmmac_pltfr_pm_ops,
		.of_match_table = caninos_dwmac_match,
	},
};

module_platform_driver(caninos_dwmac_driver);

MODULE_AUTHOR("LSI-TEC - Caninos Loucos");
MODULE_DESCRIPTION("Caninos Labrador DWMAC specific glue layer");
MODULE_LICENSE("GPL v2");

