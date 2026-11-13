/*
 * The realme 8i's battery thermistor on the MT6366's BAT_TEMP (AUXADC channel 3), the way Android reads it on this board
 * (realme's GPL kernel, ODM_SELECT_BAT_NTC_SUPPORT 21690 at PCB version 4 and later; the phone logged version 11):
 *  - 12 bits over 1.8 V, halved before the converter (the DT's resistance-ratio 2/1): counts x 3600 / 4096 mV.
 *    Android logged 0x507 = 1131 mV.
 *  - The cell's current moves it by 1 mV a 100 mA (COM_FG_METER_RESISTANCE 100 + COM_R_FG_VALUE 0, over 10000): taken
 *    off while charging, added while discharging.
 *  - Pulled up by 12 kOhm (RBAT_PULL_UP_R_new) to 2.8 V (RBAT_PULL_UP_VOLT, fixed for this board, not VBIF28 read back);
 *    2 V and over is no thermistor.
 *  - Ohms to degrees by Fg_Temperature_Table_new_temp (mt6785 mtk_battery_table.h, which this board builds with),
 *    a straight line between its points; outside the table there is no reading.
 * tools/mt6358_ntc_test.c checks it against Android's log.
 */
#ifndef _MT6358_NTC_H_
#define _MT6358_NTC_H_

#include <stdbool.h>
#include <stdint.h>

static const struct {
	int32_t c, ohm;
} mt6358_ntc[] = {
	{ -40, 21437 }, { -35, 20750 }, { -30, 19956 }, { -25, 19062 }, { -20, 18082 }, { -15, 17039 }, { -10, 15956 },
	{ -5, 14856 }, { 0, 13772 }, { 5, 12732 }, { 10, 11751 }, { 15, 10841 }, { 20, 10012 }, { 25, 9268 },
	{ 30, 8604 }, { 35, 8021 }, { 40, 7511 }, { 45, 7067 }, { 50, 6682 }, { 55, 6350 }, { 60, 6064 },
	{ 65, 5821 }, { 70, 5613 }, { 75, 5434 }, { 80, 5275 }, { 85, 5146 }, { 90, 5032 }, { 95, 4933 },
};

static inline bool
mt6358_ntc_decic(int32_t ohm, int32_t *decic)
{
	unsigned i;

	if (ohm > mt6358_ntc[0].ohm || ohm < mt6358_ntc[sizeof(mt6358_ntc) / sizeof(mt6358_ntc[0]) - 1].ohm) {
		return false;
	}
	for (i = 1; ohm < mt6358_ntc[i].ohm; i++) {
	}
	*decic = ((ohm - mt6358_ntc[i].ohm) * mt6358_ntc[i - 1].c + (mt6358_ntc[i - 1].ohm - ohm) * mt6358_ntc[i].c) * 10 /
	    (mt6358_ntc[i - 1].ohm - mt6358_ntc[i].ohm);
	return true;
}

/* BAT_TEMP's counts, the cell's current (mA, more than 0 into it) to tenths of a degree */
static inline bool
mt6358_bat_temp_decic(uint32_t counts, int32_t ma, int32_t *decic)
{
	int32_t mv = (int32_t)(counts * 3600 / 4096) - ma / 100;

	if (mv >= 2000) {
		return false;
	}
	return mt6358_ntc_decic((int32_t)((int64_t)12000 * mv / (2800 - mv)), decic);
}

#endif
