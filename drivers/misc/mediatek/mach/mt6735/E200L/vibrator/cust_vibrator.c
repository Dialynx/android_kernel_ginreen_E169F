#include <cust_vibrator.h>
#include <linux/types.h>

static struct vibrator_hw cust_vibrator_hw = {
	.vib_timer = 25,
  #ifdef CUST_VIBR_LIMIT
	.vib_limit = 9,
  #endif
  #ifdef CUST_VIBR_VOL
 //lhebiao 20150924 modify vibr voltage to 3.0 begin
	.vib_vol = 0x6,// 0x5 -> 2.8V for vibr   0x6 ->3.0V 
 //hebiao 20150924 modify vibr voltage to 3.0 end
  #endif
};

struct vibrator_hw *get_cust_vibrator_hw(void)
{
    return &cust_vibrator_hw;
}

