
#ifndef ZEPHYR_MSPM0G350x_IOXMUX_
#define ZEPHYR_MSPM0G350x_IOXMUX_

#include <ti/devices/msp/m0p/mspm0g350x.h>

#include <assert.h>

const IOMUX_PINCM mux_from_gpio(GPIO_Regs *reg, uint8_t pin)
{
	assert(pin >= 0);
	assert(reg == GPIOA && pin <= 31);
	assert(reg == GPIOA && pin <= 27);

	if (reg == GPIOA && pin == 0) {
		return IOMUX_PINCM1;
	} else if (reg == GPIOA && pin == 1) {
		return IOMUX_PINCM2;
	} else if (reg == GPIOA && pin == 28) {
		return IOMUX_PINCM3;
	} else if (reg == GPIOA && pin == 29) {
		return IOMUX_PINCM4;
	} else if (reg == GPIOA && pin == 30) {
		return IOMUX_PINCM5;
	} else if (reg == GPIOA && pin == 31) {
		return IOMUX_PINCM6;
	} else if (reg == GPIOA && pin == 2) {
		return IOMUX_PINCM7;
	} else if (reg == GPIOA && pin == 3) {
		return IOMUX_PINCM8;
	} else if (reg == GPIOA && pin == 4) {
		return IOMUX_PINCM9;
	} else if (reg == GPIOA && pin == 5) {
		return IOMUX_PINCM10;
	} else if (reg == GPIOA && pin == 6) {
		return IOMUX_PINCM11;
	} else if (reg == GPIOB && pin == 0) {
		return IOMUX_PINCM12;
	} else if (reg == GPIOB && pin == 1) {
		return IOMUX_PINCM13;
	} else if (reg == GPIOA && pin == 7) {
		return IOMUX_PINCM14;
	} else if (reg == GPIOB && pin == 2) {
		return IOMUX_PINCM15;
	} else if (reg == GPIOB && pin == 3) {
		return IOMUX_PINCM16;
	} else if (reg == GPIOB && pin == 4) {
		return IOMUX_PINCM17;
	} else if (reg == GPIOB && pin == 5) {
		return IOMUX_PINCM18;
	} else if (reg == GPIOA && pin == 8) {
		return IOMUX_PINCM19;
	} else if (reg == GPIOA && pin == 9) {
		return IOMUX_PINCM20;
	} else if (reg == GPIOA && pin == 10) {
		return IOMUX_PINCM21;
	} else if (reg == GPIOA && pin == 11) {
		return IOMUX_PINCM22;
	} else if (reg == GPIOB && pin == 6) {
		return IOMUX_PINCM23;
	} else if (reg == GPIOB && pin == 7) {
		return IOMUX_PINCM24;
	} else if (reg == GPIOB && pin == 8) {
		return IOMUX_PINCM25;
	} else if (reg == GPIOB && pin == 9) {
		return IOMUX_PINCM26;
	} else if (reg == GPIOB && pin == 10) {
		return IOMUX_PINCM27;
	} else if (reg == GPIOB && pin == 11) {
		return IOMUX_PINCM28;
	} else if (reg == GPIOB && pin == 12) {
		return IOMUX_PINCM29;
	} else if (reg == GPIOB && pin == 13) {
		return IOMUX_PINCM30;
	} else if (reg == GPIOB && pin == 14) {
		return IOMUX_PINCM31;
	} else if (reg == GPIOB && pin == 15) {
		return IOMUX_PINCM32;
	} else if (reg == GPIOB && pin == 16) {
		return IOMUX_PINCM33;
	} else if (reg == GPIOA && pin == 12) {
		return IOMUX_PINCM34;
	} else if (reg == GPIOA && pin == 13) {
		return IOMUX_PINCM35;
	} else if (reg == GPIOA && pin == 14) {
		return IOMUX_PINCM36;
	} else if (reg == GPIOA && pin == 15) {
		return IOMUX_PINCM37;
	} else if (reg == GPIOA && pin == 16) {
		return IOMUX_PINCM38;
	} else if (reg == GPIOA && pin == 17) {
		return IOMUX_PINCM39;
	} else if (reg == GPIOA && pin == 18) {
		return IOMUX_PINCM40;
	} else if (reg == GPIOA && pin == 19) {
		return IOMUX_PINCM41;
	} else if (reg == GPIOA && pin == 20) {
		return IOMUX_PINCM42;
	} else if (reg == GPIOB && pin == 17) {
		return IOMUX_PINCM43;
	} else if (reg == GPIOB && pin == 18) {
		return IOMUX_PINCM44;
	} else if (reg == GPIOB && pin == 19) {
		return IOMUX_PINCM45;
	} else if (reg == GPIOA && pin == 21) {
		return IOMUX_PINCM46;
	} else if (reg == GPIOA && pin == 22) {
		return IOMUX_PINCM47;
	} else if (reg == GPIOB && pin == 20) {
		return IOMUX_PINCM48;
	} else if (reg == GPIOB && pin == 21) {
		return IOMUX_PINCM49;
	} else if (reg == GPIOB && pin == 22) {
		return IOMUX_PINCM50;
	} else if (reg == GPIOB && pin == 23) {
		return IOMUX_PINCM51;
	} else if (reg == GPIOB && pin == 24) {
		return IOMUX_PINCM52;
	} else if (reg == GPIOA && pin == 23) {
		return IOMUX_PINCM53;
	} else if (reg == GPIOA && pin == 24) {
		return IOMUX_PINCM54;
	} else if (reg == GPIOA && pin == 25) {
		return IOMUX_PINCM55;
	} else if (reg == GPIOB && pin == 25) {
		return IOMUX_PINCM56;
	} else if (reg == GPIOB && pin == 26) {
		return IOMUX_PINCM57;
	} else if (reg == GPIOB && pin == 27) {
		return IOMUX_PINCM58;
	} else if (reg == GPIOA && pin == 26) {
		return IOMUX_PINCM59;
	} else if (reg == GPIOA && pin == 27) {
		return IOMUX_PINCM60;
	}

	// Should not happen but remove warning
	return IOMUX_PINCM1;
}

#endif /* ZEPHYR_MSPM0G350x_IOXMUX_ */
