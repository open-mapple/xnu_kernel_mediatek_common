/* Open Mapple: WPA2-PSK supplicant (bsd/dev/mt6631_wpa.c), used by bsd/dev/mt6631_wifi.c. */
#ifndef _MT6631_WPA_H_
#define _MT6631_WPA_H_

#define MT6631_WPA_SEND   1       /* w->out / w->out_len: EAPOL frame to send to the AP */
#define MT6631_WPA_PTK    2       /* install w->ptk + 32 (TK, 16 bytes), pairwise, key id 0 */
#define MT6631_WPA_GTK    4       /* install w->gtk (gtk_len) as group key gtk_idx, RX PN from gtk_rsc */

struct mt6631_wpa {
	uint8_t  pmk[32];
	uint8_t  aa[6], spa[6];         /* AP (authenticator) and our address */
	uint8_t  anonce[32], snonce[32];
	uint8_t  ptk[48];               /* KCK | KEK | TK */
	uint8_t  replay[8];
	uint8_t  rsnie[64], rsnie_len;  /* our RSN IE exactly as in the association request */
	uint8_t  gtk[32], gtk_len, gtk_idx, gtk_rsc[8];
	uint8_t  have_ptk, replay_ok, state;     /* state 0 idle, 1 sent message 2, 2 keys installed */
	uint32_t drops, mic_fail;
	uint8_t  out[256];
	uint32_t out_len;
};

void mt6631_wpa_pmk(const char *pass, const uint8_t *ssid, size_t ssid_len, uint8_t pmk[32]);
void mt6631_wpa_start(struct mt6631_wpa *w, const uint8_t pmk[32], const uint8_t aa[6], const uint8_t spa[6],
    const uint8_t *rsnie, size_t rsnie_len);
int  mt6631_wpa_rx(struct mt6631_wpa *w, const uint8_t *frame, size_t len);

#endif
