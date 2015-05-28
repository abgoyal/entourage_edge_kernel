#ifndef __ASM_MACH_PXA910_H
#define __ASM_MACH_PXA910_H

#include <mach/devices.h>
#include <plat/i2c.h>

/*TODO: add devices here for pxa910*/

static inline int pxa910_add_uart(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &pxa910_device_uart1; break;
	case 2: d = &pxa910_device_uart2; break;
	case 3: d = &pxa910_device_uart3; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline void pxa910_add_acipc(void)
{
	int ret;
	ret = platform_device_register(&pxa910_device_acipc);
	if (ret)
		dev_err(&pxa910_device_acipc.dev,
			"unable to register device: %d\n", ret);
}

static inline int pxa910_add_ire(void)
{
	return pxa_register_device(&pxa910_device_ire, NULL, 0);
}

static inline int pxa910_add_ssp(int id)
{
        struct pxa_device_desc *d = NULL;

        switch (id) {
	        case 0: d = &pxa910_device_ssp0; break;
	        case 1: d = &pxa910_device_ssp1; break;
	        case 2: d = &pxa910_device_ssp2; break;
	        default:
	                return -EINVAL;
	        }

        return pxa_register_device(d, NULL, 0);
}

static inline void pxa910_add_imm(void)
{
        int ret;
        ret = platform_device_register(&pxa910_device_imm);
	if (ret)
		dev_err(&pxa910_device_imm.dev,
			"unable to register device: %d\n", ret);
}

static inline void pxa910_add_rtc(void)
{
	int ret;
	ret = platform_device_register(&pxa910_device_rtc);
	if (ret)
		dev_err(&pxa910_device_acipc.dev,
			"unable to register device: %d\n", ret);
}

static inline int pxa910_add_twsi(int id, struct i2c_pxa_platform_data *data,
				  struct i2c_board_info *info, unsigned size)
{
	struct pxa_device_desc *d = NULL;
	int ret;

	switch (id) {
	case 0: d = &pxa910_device_twsi0; break;
	case 1: d = &pxa910_device_twsi1; break;
	default:
		return -EINVAL;
	}

	ret = i2c_register_board_info(id, info, size);
	if (ret)
		return ret;

	return pxa_register_device(d, data, sizeof(*data));
}


#endif /* __ASM_MACH_PXA910_H */
