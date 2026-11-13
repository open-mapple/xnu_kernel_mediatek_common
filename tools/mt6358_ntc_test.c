/*
 * bsd/dev/mt6358_ntc.h on this machine: the table's own points, its ends, and Android's log of the realme 8i.
 *   cc -Wall tools/mt6358_ntc_test.c -o ntc_test && ./ntc_test
 */
#include <stdio.h>
#include "../bsd/dev/mt6358_ntc.h"

static int failures;

static void
expect(const char *what, bool ok, int32_t got, bool want_ok, int32_t want)
{
	if (ok != want_ok || (ok && got != want)) {
		printf("FAILED: %s: %s %d, wanted %s %d\n", what, ok ? "got" : "no reading", got, want_ok ? "" : "no reading", want);
		failures++;
	}
}

int
main(void)
{
	/* Android's log: BAT_TEMP's counts and the tenths it made of them; the current is not in the log, and 400..499 mA in
	 * (the phone on a USB port) gives all four to the tenth */
	static const struct { uint32_t counts; int32_t decic; } android[] = {
		{ 0x507, 344 }, { 0x500, 350 }, { 0x4f7, 360 }, { 0x4f1, 367 },
	};
	int32_t t = 0;
	bool ok;

	for (unsigned i = 0; i < sizeof(mt6358_ntc) / sizeof(mt6358_ntc[0]); i++) {
		ok = mt6358_ntc_decic(mt6358_ntc[i].ohm, &t);
		expect("a table point", ok, t, true, mt6358_ntc[i].c * 10);
	}
	ok = mt6358_ntc_decic(21438, &t);
	expect("colder than -40", ok, t, false, 0);
	ok = mt6358_ntc_decic(4932, &t);
	expect("hotter than 95", ok, t, false, 0);
	ok = mt6358_ntc_decic((8604 + 8021) / 2, &t);
	expect("halfway 30..35", ok, t, true, 325);
	for (unsigned i = 0; i < sizeof(android) / sizeof(android[0]); i++) {
		ok = mt6358_bat_temp_decic(android[i].counts, 450, &t);
		expect("Android's log", ok, t, true, android[i].decic);
		ok = mt6358_bat_temp_decic(android[i].counts, 0, &t);
		if (!ok || t < android[i].decic - 6 || t > android[i].decic) {
			printf("FAILED: Android's log without the current: %d against %d\n", t, android[i].decic);
			failures++;
		}
	}
	ok = mt6358_bat_temp_decic(0xfff, 0, &t);
	expect("no thermistor (open)", ok, t, false, 0);
	ok = mt6358_bat_temp_decic(0, 0, &t);
	expect("shorted", ok, t, false, 0);
	ok = mt6358_bat_temp_decic(0, 1000, &t);
	expect("shorted, charging", ok, t, false, 0);
	(void)mt6358_bat_temp_decic(0x507, 450, &t);
	printf("%s: Android's 34.4 C is counts 0x507 = %d tenths at 450 mA in\n", failures ? "FAILED" : "ok", t);
	return failures != 0;
}
