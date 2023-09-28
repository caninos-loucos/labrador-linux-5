// SPDX-License-Identifier: GPL-2.0
/*
 * Divider clock implementation for Caninos Labrador
 *
 * Copyright (c) 2022-2023 ITEX - LSITEC - Caninos Loucos
 * Author: Edgar Bernardi Righi <edgar.righi@lsitec.org.br>
 *
 * Copyright (c) 2018-2020 LSI-TEC - Caninos Loucos
 * Author: Edgar Bernardi Righi <edgar.righi@lsitec.org.br>
 *
 * Copyright (c) 2014 Actions Semi Inc.
 * Author: David Liu <liuwei@actions-semi.com>
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

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include "clk-caninos.h"

struct clk *__init
caninos_register_divider_table(const struct caninos_div_clock *info,
                               struct device *dev, void __iomem *reg,
                               spinlock_t *lock)
{
	struct clk_div_table *table = NULL;
	struct clk *clk;
	int count, len;
	
	if (info->table)
	{
		/* calculate the number of elements in div_table */
		for (count = 0; info->table[count].div > 0UL; count++);
		
		if (!count) {
			return ERR_PTR(-EINVAL);
		}
		
		len = (count + 1) * sizeof(*table);
		
		/* allocate space for a copy of div_table */
		table = kmalloc(len, GFP_KERNEL);
		
		if (!table) {
			return ERR_PTR(-ENOMEM);
		}
		
		/* copy div_table to allow __initconst*/
		memcpy(table, info->table, len);
	}
	
	clk = clk_register_divider_table(dev, info->name, info->parent_name,
	                                 info->flags, reg + info->offset,
	                                 info->shift, info->width, info->div_flags,
	                                 table, lock);
	
	if (IS_ERR(clk) && table) {
		kfree(table);
	}
	return clk;
}

