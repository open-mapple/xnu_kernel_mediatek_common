/*
 * The passcode record of the built-in AppleKeyStore. There is no Secure
 * Enclave: the verifier is a PBKDF2-HMAC-SHA256 of the passcode with a random
 * salt, kept in a file only root can read, so a copy of that file can be
 * attacked offline. It locks the UI; it does not protect data.
 *
 * The file holds two slots; each write goes to the older one with the next
 * sequence number, so a torn write leaves the other intact.
 */
#ifndef _MAPPLE_KEYSTORE_PASSCODE_H
#define _MAPPLE_KEYSTORE_PASSCODE_H

__BEGIN_DECLS
#include <corecrypto/cchmac.h>
#include <corecrypto/ccsha2.h>
#include <corecrypto/cc.h>
__END_DECLS

#define MAPPLE_AKS_REC_MAGIC    0x314b504du     /* "MPK1" */
#define MAPPLE_AKS_SALT_LEN     16
#define MAPPLE_AKS_HASH_LEN     32
#define MAPPLE_AKS_ITERATIONS   20000
#define MAPPLE_AKS_SECRET_MAX   128

struct mapple_aks_rec {
	uint32_t magic;
	uint32_t seq;
	uint32_t has;           /* a passcode is set */
	uint32_t iter;
	uint8_t  salt[MAPPLE_AKS_SALT_LEN];
	uint8_t  hash[MAPPLE_AKS_HASH_LEN];
	uint32_t failed;        /* wrong attempts since the last right one */
	uint32_t reserved;
	uint64_t until;         /* no attempt before this calendar second */
	uint8_t  sum[MAPPLE_AKS_HASH_LEN];      /* HMAC-SHA256, empty key, of everything above */
};

enum {
	MAPPLE_AKS_OK = 0,
	MAPPLE_AKS_WRONG,
	MAPPLE_AKS_WAIT,
};

/* iOS's delays after the nth wrong passcode (Apple Platform Security). */
static inline uint64_t
mapple_aks_delay(uint32_t failed)
{
	if (failed < 5) {
		return 0;
	}
	if (failed == 5) {
		return 60;
	}
	if (failed == 6) {
		return 5 * 60;
	}
	if (failed <= 8) {
		return 15 * 60;
	}
	return 60 * 60;
}

/* PBKDF2-HMAC-SHA256, one 32-byte block. */
static inline void
mapple_aks_derive(const void *pw, size_t pwlen, const uint8_t *salt, size_t saltlen, uint32_t iter, uint8_t out[32])
{
	const struct ccdigest_info *di = ccsha256_di();
	uint8_t blk[64 + 4], u[32], v[32];

	if (saltlen > 64) {
		saltlen = 64;
	}
	memcpy(blk, salt, saltlen);
	blk[saltlen] = 0;
	blk[saltlen + 1] = 0;
	blk[saltlen + 2] = 0;
	blk[saltlen + 3] = 1;
	cchmac(di, pwlen, pw, saltlen + 4, blk, u);
	memcpy(out, u, 32);
	for (uint32_t i = 1; i < iter; i++) {
		cchmac(di, pwlen, pw, 32, u, v);
		memcpy(u, v, 32);
		for (int j = 0; j < 32; j++) {
			out[j] ^= u[j];
		}
	}
	cc_clear(sizeof(u), u);
	cc_clear(sizeof(v), v);
}

static inline void
mapple_aks_rec_sum(const struct mapple_aks_rec *r, uint8_t sum[32])
{
	cchmac(ccsha256_di(), 0, "", offsetof(struct mapple_aks_rec, sum), r, sum);
}

static inline bool
mapple_aks_rec_valid(const struct mapple_aks_rec *r)
{
	uint8_t sum[32];

	if (r->magic != MAPPLE_AKS_REC_MAGIC) {
		return false;
	}
	mapple_aks_rec_sum(r, sum);
	return cc_cmp_safe(sizeof(sum), sum, r->sum) == 0;
}

/* The newer valid slot, or NULL when neither is. */
static inline const struct mapple_aks_rec *
mapple_aks_rec_pick(const struct mapple_aks_rec slot[2])
{
	bool a = mapple_aks_rec_valid(&slot[0]), b = mapple_aks_rec_valid(&slot[1]);

	if (a && b) {
		return (int32_t)(slot[1].seq - slot[0].seq) > 0 ? &slot[1] : &slot[0];
	}
	return a ? &slot[0] : b ? &slot[1] : NULL;
}

/*
 * Check a passcode at calendar second now. A wrong one (not an empty one)
 * counts and starts the next delay; nothing is tried while a delay runs (*wait: seconds left). A
 * clock set back cannot stretch a delay past its length.
 */
static inline int
mapple_aks_check(struct mapple_aks_rec *r, const void *pw, size_t len, uint64_t now, uint64_t *wait)
{
	uint8_t h[32];
	uint64_t d;

	*wait = 0;
	if (!r->has) {
		return MAPPLE_AKS_OK;
	}
	d = mapple_aks_delay(r->failed);
	if (r->until > now + d) {
		r->until = now + d;
	}
	if (now < r->until) {
		*wait = r->until - now;
		return MAPPLE_AKS_WAIT;
	}
	if (len == 0) {
		return MAPPLE_AKS_WRONG;        /* an empty one is never a try */
	}
	mapple_aks_derive(pw, len, r->salt, sizeof(r->salt), r->iter, h);
	if (cc_cmp_safe(sizeof(h), h, r->hash) == 0) {
		cc_clear(sizeof(h), h);
		r->failed = 0;
		r->until = 0;
		return MAPPLE_AKS_OK;
	}
	cc_clear(sizeof(h), h);
	r->failed++;
	r->until = now + mapple_aks_delay(r->failed);
	*wait = r->until - now;
	return MAPPLE_AKS_WRONG;
}

/* Set (newlen > 0) or remove (newlen == 0) the passcode after checking the old one. */
static inline int
mapple_aks_change(struct mapple_aks_rec *r, const void *old, size_t oldlen, const void *pw, size_t newlen,
    const uint8_t salt[MAPPLE_AKS_SALT_LEN], uint64_t now, uint64_t *wait)
{
	int ret = mapple_aks_check(r, old, oldlen, now, wait);

	if (ret != MAPPLE_AKS_OK) {
		return ret;
	}
	r->has = newlen > 0;
	r->iter = MAPPLE_AKS_ITERATIONS;
	if (r->has) {
		memcpy(r->salt, salt, sizeof(r->salt));
		mapple_aks_derive(pw, newlen, r->salt, sizeof(r->salt), r->iter, r->hash);
	} else {
		cc_clear(sizeof(r->salt), r->salt);
		cc_clear(sizeof(r->hash), r->hash);
	}
	return MAPPLE_AKS_OK;
}

/* The first item of aks_pack_data's buffer: u32 count, then u32 length + bytes padded to 4, per item. */
static inline bool
mapple_aks_first_item(const uint8_t *buf, size_t len, const uint8_t **item, uint32_t *itemlen)
{
	uint32_t count, n;

	if (len < 8) {
		return false;
	}
	memcpy(&count, buf, 4);
	memcpy(&n, buf + 4, 4);
	if (count < 1 || n > len - 8) {
		return false;
	}
	*item = buf + 8;
	*itemlen = n;
	return true;
}

#endif /* _MAPPLE_KEYSTORE_PASSCODE_H */
