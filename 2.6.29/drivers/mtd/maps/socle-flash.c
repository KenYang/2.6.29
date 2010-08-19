/*======================================================================

    drivers/mtd/maps/socle-flash.c: Socle flash map driver

    Copyright (C) 2007 Socle Tech. Corp.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

======================================================================*/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/init.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/flash.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/system.h>
#include <mach/platform.h>

#define PFX "socle-flash: "

struct socle_flash_info {
	struct mtd_info		*mtd;
	struct map_info		 map;
	struct mtd_partition	*partitions;
	struct resource		*area;
};

//#ifdef CONFIG_MTD_PARTITIONS
//static const char *probes[] = { "u-boot", "kernel", "ramdisk", "data", NULL };
//#endif

static int socle_flash_remove(struct platform_device *pdev)
{
	struct socle_flash_info *info = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	del_mtd_partitions(info->mtd);
	map_destroy(info->mtd);

	platform_set_drvdata(pdev, NULL);
	iounmap(info->map.virt);

	release_mem_region(res->start, (res->end - res->start) + 1);
	kfree(info);

	return 0;
}

static int socle_flash_probe(struct platform_device *pdev)
{
	struct socle_flash_info *info;
	struct flash_platform_data *pdata = pdev->dev.platform_data;
	struct resource *res = NULL;

	int ret;
	static int no_partitions;
	info = kzalloc(sizeof(struct socle_flash_info), GFP_KERNEL);
	if (info == NULL) {
		printk(KERN_ERR PFX "no memory for flash info\n");
		return -ENOMEM;
	}

	/* request register map resource & check */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!res) {
                dev_err(&pdev->dev, "register resources unusable\n");
                ret = -ENXIO;
                goto free_dev;
        }

	if (!request_mem_region(res->start, res->end - res->start + 1, pdev->name)) {
		ret = -EBUSY;
		goto free_something_1;
	}

	info->map.virt = ioremap(res->start, (res->end - res->start) + 1);
	if (!info->map.virt) {
		dev_err(&pdev->dev, "cannot map socle_flash_info registers\n");
		ret = -ENOMEM;
		goto release_mem;
	}	
	info->map.phys = res->start;
	info->map.size = res->end - res->start + 1;
	info->map.name = pdev->dev.bus_id;
#ifdef CONFIG_MTD_MAP_BANK_WIDTH_1
		info->map.bankwidth = 1;
#endif
#ifdef CONFIG_MTD_MAP_BANK_WIDTH_2
		info->map.bankwidth = 2;
#endif	
	platform_set_drvdata(pdev, info);

//	printk("%s %x, %x, %s \n",__FUNCTION__,res->start,res->end,pdev->dev.bus_id);

//	printk("%s: area %08lx, size %lx\n", __FUNCTION__, info->map.phys, info->map.size);

//	printk("%s: virt at %08x, res->start %x \n", __FUNCTION__, (int)info->map.virt,res->start);

	info->partitions = pdata->parts;

	simple_map_init(&info->map);

	/* probe for the device(s) */

	info->mtd = do_map_probe(pdata->map_name, &info->map);
	if (!info->mtd) {
		ret = -EIO;
		goto reset_drvdata;
	}

	/* mark ourselves as the owner */
	info->mtd->owner = THIS_MODULE;

	no_partitions = pdata->nr_parts;//ARRAY_SIZE(nor_partitions);
	ret = add_mtd_partitions(info->mtd, info->partitions, no_partitions);
	if (ret){
		printk(KERN_ERR PFX "cannot add/parse partitions\n");
		goto free_something_2;
	}

	return 0;

	/* fall through to exit error */

free_something_2:
	del_mtd_partitions(info->mtd);
	map_destroy(info->mtd);
reset_drvdata:
        //kfree(info->partitions);
	platform_set_drvdata(pdev, NULL);
//unmap_regs:
	iounmap(info->map.virt);
release_mem:
        release_mem_region(res->start, (res->end - res->start) + 1);
free_something_1:

free_dev:
	kfree(info);

	return ret;
}

#ifdef CONFIG_PM
static int
socle_flash_suspend(struct platform_device *pdev, pm_message_t msg)
{
	pr_debug("socle_flash_suspend\n");

        return 0;
}

static int 
socle_flash_resume(struct platform_device *pdev)
{	
	pr_debug("socle_flash_resume\n");
	
  	return 0;
}
#else
#define socle_flash_suspend NULL
#define socle_flash_resume NULL
#endif
	
static struct platform_driver socle_flash_driver = {
	.probe		= socle_flash_probe,
	.remove		= socle_flash_remove,
	.suspend = socle_flash_suspend,
	.resume = socle_flash_resume,
	.driver		= {
		.name	= "socle-flash",
		.owner	= THIS_MODULE,
	},
};

static int __init socle_flash_init(void)
{
	printk("SOCLE NOR-Flash Driver, (c) 2007 Socle Tech \n");
	return platform_driver_register(&socle_flash_driver);
}

static void __exit socle_flash_exit(void)
{
	platform_driver_unregister(&socle_flash_driver);
}

module_init(socle_flash_init);
module_exit(socle_flash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ryan Chen <ryanchen@socle-tech.com.tw>");
MODULE_DESCRIPTION("SOCLE MTD Map driver");

