#ifdef CONFIG_OFM_MOUSE
#ifndef __OFM_MOUSE__
#define __OFM_MOUSE__

enum ofm_pins {
		OFM_POWER_DN = 0,
		OFM_MOTION_DETECT,
		OFM_LEFT_BUTTON,
};

struct ofm_pin{
		int 	pin;
    	int 	pin_setting;
    	int 	irq;
        char	*name;
};
#endif
#endif
