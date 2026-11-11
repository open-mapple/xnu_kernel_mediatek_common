/*
 * Open Mapple: the WiFi station (bsd/dev/mt6631_wifi.c: CONNSYS, MLME, WPA2) and its Apple80211 face for iOS
 * (bsd/dev/mt6631_wlan.c: en0, SIOCGA80211/SIOCSA80211, kern_event, iokit/Kernel/arm/MT6631WLAN.cpp: registry).
 */
#ifndef _MT6631_WLAN_H_
#define _MT6631_WLAN_H_

#include <sys/kpi_mbuf.h>

#define VWL_IE_MAX      512

struct vwl_bss {                        /* one BSS from the last scans */
	uint8_t  bssid[6], ssid[32], ssid_len, ch;
	int8_t   rssi;
	uint8_t  rsvd;
	uint16_t capinfo, bintval;
	uint16_t ie_len;
	uint8_t  ie[VWL_IE_MAX];        /* beacon / probe response body after the fixed fields */
	uint32_t age_ms;
};

struct vwl_scan {
	uint8_t  ssid[32], ssid_len;    /* 0 = wildcard */
	uint8_t  passive;
	uint8_t  nch;                   /* 0 = all */
	uint8_t  ch[64];
};

struct vwl_join {
	uint8_t  ssid[32], ssid_len;
	uint8_t  bssid[6];              /* zero: the strongest BSS with that SSID */
	uint8_t  has_pmk;               /* WPA2-PSK; otherwise open */
	uint8_t  pmk[32];
	uint8_t  rsnie[64], rsnie_len;  /* RSN IE for the association request (0: built by the driver) */
};

struct vwl_status {
	uint8_t  power;                 /* radio on */
	uint8_t  state;                 /* APPLE80211_S_*: 0 init, 1 scan, 2 auth, 3 assoc, 4 run */
	uint8_t  link;                  /* associated and keys installed: data flows */
	uint8_t  result;                /* APPLE80211 ASSOCIATE_RESULT of the last join (0 = running) */
	uint8_t  ssid[32], ssid_len, ch;
	uint8_t  bssid[6], mac[6];
	uint8_t  rsn;                   /* the current BSS uses WPA2-PSK */
	int8_t   rssi;
	uint16_t rate;                  /* Mbit/s */
	uint32_t txFrames, rxFrames, txDrops;
};

/* mt6631_wifi.c (station) */
int  mt6631_wifi_set_power(bool on);                      /* blocks until the radio is up (or down) */
int  mt6631_wifi_scan(const struct vwl_scan *s);          /* async: mt6631_wlan_scan_done() */
int  mt6631_wifi_scan_result(uint32_t i, struct vwl_bss *out);    /* ENOENT past the end */
int  mt6631_wifi_current_bss(struct vwl_bss *out);
int  mt6631_wifi_join(const struct vwl_join *j);          /* async: mt6631_wlan_assoc_done() */
void mt6631_wifi_leave(void);
void mt6631_wifi_get_status(struct vwl_status *s);
void mt6631_wifi_output(mbuf_t m);                        /* an 802.3 frame; the mbuf is consumed */
void mt6631_wifi_mac(uint8_t mac[6]);

/* mt6631_wlan.c (Apple80211 / BSD side), called by the station without its locks held */
void mt6631_wlan_attach(void);                            /* en0 (MT6631WLAN::start) */
void mt6631_wlan_scan_done(int status);
void mt6631_wlan_assoc_done(int result);                  /* APPLE80211 ASSOCIATE_RESULT, 1 = success */
void mt6631_wlan_link(bool up, uint32_t reason);
void mt6631_wlan_power(bool on);
void mt6631_wlan_input(mbuf_t m);                         /* a received 802.3 frame */
size_t mt6631_wlan_stats(char *out, size_t len);            /* USB link "wifi sta" */

/* MT6631WLAN.cpp */
void mt6631_wlan_registry_link(bool up);

#endif
