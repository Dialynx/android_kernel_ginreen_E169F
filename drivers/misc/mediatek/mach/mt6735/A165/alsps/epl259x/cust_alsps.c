//liqiang 20150720 begin
#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <epl259x_cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
	.polling_mode_ps = 0,		          /* not work, define in epl8882.c */
	.polling_mode_als = 1,		          /* not work, define in epl8882.c */
	.power_id   = MT65XX_POWER_NONE,      /* LDO is not used */
	.power_vol  = VOL_DEFAULT,            /* LDO is not used */
	.i2c_addr   = {0x92, 0x48, 0x78, 0x00},
	.als_level	= {0, 40, 100, 180, 270,  650, 1200, 1800, 3000,  4000,  5000, 7000, 9000, 15000, 20000},
	.als_value	= {0,0,150, 340, 430, 570,  700,  800,  950,  1500,  3000,  5000, 7000, 9000, 10240, 10240},
	.ps_threshold_low = 500,
	.ps_threshold_high = 800,
	.als_threshold_low = 1000,
	.als_threshold_high = 1500,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}
//liqiang 20150720 end
