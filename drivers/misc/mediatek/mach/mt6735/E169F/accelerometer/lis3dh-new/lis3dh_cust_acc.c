//lichengmin begin
#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>


/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = 2,
//tuwenzan modify gsensor direction at 20150715 begin
    .direction = 3,//0 7  6  4  2 3  1
//tuwenzan modify gsensor direction at 20150715 end
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 6, //old value 16 tuwenzan modify at 20150923 /*!< don't enable low pass fileter */
};
/*---------------------------------------------------------------------------*/
struct acc_hw* lis3dh_get_cust_acc_hw(void) 
{
    return &cust_acc_hw;
}
//lichengmin end