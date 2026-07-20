/* mac_hostname.c - MAC <-> hostname codec (v3, fully bijective).
 *
 * This file turns a network device's 6-byte MAC address (its
 * hardware serial number, e.g. AA:BB:CC:DD:EE:FF) into a friendly,
 * pronounceable hostname like "noble-abbot-marc-hark", and back
 * again. It's self-contained: the 4-round Feistel permutation
 * (explained below) lives here and takes its keys from name_data.c
 * (ROUND_KEYS). No other component is needed.
 *
 *   48 bits = PREFIX 8 | TITLE 7 | GIVEN 17 | SURNAME 16   (MSB -> LSB)
 *
 * A MAC address is 48 bits. We treat those 48 bits as one big number,
 * scramble it (see the Feistel section below), then chop the result
 * into 4 chunks of 8/7/17/16 bits. Each chunk is an index into one of
 * the word tables in name_data.c, which is how we get 4 words out of
 * 48 bits of binary.
 *
 * GIVEN and SURNAME tokens are stem+ending compositions (e.g. stem
 * "marc" + ending "" -> "marc", or stem "a" + ending "ren" ->
 * "aren") - this lets a small table of stems and a small table of
 * endings combine into far more distinct words than either table
 * could hold alone. The stem tables are PREFIX-FREE, meaning no stem
 * is the beginning of another stem, so:
 *   stem1+end1 == stem2+end2  =>  stem1 == stem2 and end1 == end2
 * In plain terms: given a finished word, there's only ever one way
 * to split it back into (stem, ending), so decoding is unambiguous -
 * a simple left-to-right scan for a matching stem always finds the
 * right one. Combined with the Feistel step being reversible, every
 * one of the 2^48 possible MACs gets its own distinct hostname, and
 * every hostname this scheme produces can be decoded back to exactly
 * that MAC (this "no two inputs share an output, and every output
 * can be reversed" property is what "bijective" means).
 *
 * Layer index convention (see name_data.c):
 *   pro = idx >> deu_bits,  deu = idx & ((1 << deu_bits) - 1)
 * Layers with deu_count == 1 (prefix, title) have an empty ending -
 * they're single whole words, not stem+ending compositions.
 */
#include "mac_hostname.h"
#include "name_data.h"
#include <string.h>

/* Compile-time sanity checks: catch a mismatch between this file's
 * assumptions and name_data.h/mac_hostname.h at build time, instead
 * of failing (or worse, silently corrupting hostnames) at runtime. */
_Static_assert(MAC_HOSTNAME_N_TOKENS == N_LAYERS, "layer count mismatch");
_Static_assert(MAC_HOSTNAME_BUF_LEN == 32, "31-character hostname plus NUL required");

/* ------------------------------------------------------------------ */
/* 48-bit Feistel permutation (bijective)                             */
/* ------------------------------------------------------------------ */
/* A Feistel cipher is a classic, simple way to build a reversible
 * "shuffle" of a number: split it into a left and right half, run
 * the right half through a scrambling function together with a
 * round key, XOR that into the left half, then swap the halves and
 * repeat for a few rounds. It's reversible by design - running the
 * same rounds backwards undoes the shuffle exactly - which is why it
 * fits this use case: without it, MAC addresses that only differ in
 * their last byte (very common - e.g. sequential devices from the
 * same manufacturer) would produce hostnames that only differ in
 * their last word too, leaking a pattern to anyone comparing names.
 * The keys used here (ROUND_KEYS, in name_data.c) aren't secret -
 * they just need to mix the bits well, not hide anything. */
#define MASK24 0xFFFFFFu   /* keeps a value within 24 bits (our Feistel half-width) */

/* The per-round scrambling function: mixes a 24-bit half (r) with a
 * round key (k) using multiplication and XOR-shifts, so a small
 * change in the input flips many bits in the output. */
static uint32_t feistel_f(uint32_t r, uint32_t k)
{
    r ^= k;
    r = (r * k) & MASK24;  r ^= r >> 13;
    r = (r * k) & MASK24;  r ^= r >> 11;
    return r & MASK24;
}

/* Scrambles a 48-bit MAC value (x) into a 48-bit shuffled value.
 * Splits x into two 24-bit halves and runs N_ROUNDS Feistel rounds
 * over them. Used when generating a hostname from a MAC. */
static uint64_t feistel_forward(uint64_t x)
{
    uint32_t left  = (uint32_t)((x >> 24) & MASK24);
    uint32_t right = (uint32_t)( x        & MASK24);
    for (int i = 0; i < N_ROUNDS; ++i) {
        uint32_t t = right;
        right = left ^ feistel_f(right, ROUND_KEYS[i]);
        left  = t;
    }
    return ((uint64_t)left << 24) | right;
}

/* Undoes feistel_forward(): runs the same rounds in reverse order to
 * recover the original 48-bit MAC value from a shuffled one. Used
 * when parsing a hostname back into a MAC. */
static uint64_t feistel_inverse(uint64_t y)
{
    uint32_t left  = (uint32_t)((y >> 24) & MASK24);
    uint32_t right = (uint32_t)( y        & MASK24);
    for (int i = N_ROUNDS - 1; i >= 0; --i) {
        uint32_t t = left;
        left  = right ^ feistel_f(left, ROUND_KEYS[i]);
        right = t;
    }
    return ((uint64_t)left << 24) | right;
}

/* ------------------------------------------------------------------ */
/* table access helpers                                               */
/* ------------------------------------------------------------------ */
/* Recall from name_data.h: each word table is a single NUL-separated
 * string blob plus an offsets array that marks where each word
 * starts. These helpers do the actual "look up word #N" work; the
 * rest of the file never touches the tables directly. */

/* Look up stem word `index` in `layer` (e.g. one of the PREFIX_PRO_*
 * tables). Returns a pointer to the word's text (not NUL-terminated
 * on its own - always use it together with *len) and writes the
 * word's length in bytes to *len. */
static const char *pro_word(const name_layer_t *layer, uint32_t index,
                            size_t *len)
{
    *len = (size_t)(layer->pro_offsets[index + 1] -
                    layer->pro_offsets[index]) - 1u;
    return layer->pro_data + layer->pro_offsets[index];
}

/* Same idea as pro_word(), but for the "ending" table (e.g. the
 * *_DEU_* tables) that gets glued onto a stem to finish a GIVEN or
 * SURNAME token. For PREFIX/TITLE layers this is only ever called
 * with index 0, returning the single empty-string entry. */
static const char *deu_word(const name_layer_t *layer, uint32_t index,
                            size_t *len)
{
    *len = (size_t)(layer->deu_offsets[index + 1] -
                    layer->deu_offsets[index]) - 1u;
    return layer->deu_data + layer->deu_offsets[index];
}

/* Like strlen(), but never reads past `limit` bytes - protects
 * against scanning off the end of a hostname buffer that might not
 * be NUL-terminated within the expected size (e.g. bad/attacker-
 * supplied input). */
static size_t bounded_strlen(const char *text, size_t limit)
{
    size_t n = 0;
    while (n < limit && text[n] != '\0') ++n;
    return n;
}

/* Converts one ASCII uppercase letter to lowercase; leaves anything
 * else (lowercase letters, digits, punctuation) unchanged. Used so
 * hostname lookups work regardless of how the caller capitalized
 * the string (hostnames are conventionally treated case-insensitively). */
static unsigned char ascii_lower(unsigned char c)
{
    return (c >= 'A' && c <= 'Z') ? (unsigned char)(c + ('a' - 'A')) : c;
}

/* case-insensitive equality of a token slice against a table word */
static int slice_equals_ci(const char *slice, size_t slice_len,
                           const char *word, size_t word_len)
{
    if (slice_len != word_len) return 0;
    for (size_t i = 0; i < slice_len; ++i)
        if (ascii_lower((unsigned char)slice[i]) != (unsigned char)word[i])
            return 0;
    return 1;
}

/* case-insensitive prefix test of a table word against a token */
static int slice_starts_ci(const char *slice, size_t slice_len,
                           const char *word, size_t word_len)
{
    if (word_len > slice_len) return 0;
    for (size_t i = 0; i < word_len; ++i)
        if (ascii_lower((unsigned char)slice[i]) != (unsigned char)word[i])
            return 0;
    return 1;
}

/* ------------------------------------------------------------------ */
/* token <-> layer index                                              */
/* ------------------------------------------------------------------ */

/* Decode one dashed token (e.g. the "abbot" in "noble-abbot-marc-hark")
 * back into the numeric index it came from within `layer`.
 * Prefix/title (deu_count == 1): the token is a whole word, so it's
 * just a plain search through the stem list for an exact match.
 * Given/surname: the token is stem+ending, so first find which stem
 * the token starts with, then match the remaining characters against
 * the ending table. Because the stem table is prefix-free (see the
 * file header comment), at most one stem can ever match, so the
 * first one found is guaranteed to be the right one - no backtracking
 * needed. Returns 0 and sets *index on success, -1 if the token
 * doesn't match anything in this layer's tables. */
static int find_token(const name_layer_t *layer, const char *token,
                      size_t token_len, uint32_t *index)
{
    if (!token || !index || token_len == 0) return -1;

    for (uint32_t p = 0; p < layer->pro_count; ++p) {
        size_t pl;
        const char *pw = pro_word(layer, p, &pl);
        if (layer->deu_count == 1) {
            if (slice_equals_ci(token, token_len, pw, pl)) {
                *index = p;
                return 0;
            }
            continue;
        }
        if (!slice_starts_ci(token, token_len, pw, pl))
            continue;
        /* prefix-free stems: this is the only stem that can match */
        for (uint32_t d = 0; d < layer->deu_count; ++d) {
            size_t dl;
            const char *dw = deu_word(layer, d, &dl);
            if (slice_equals_ci(token + pl, token_len - pl, dw, dl)) {
                *index = (p << layer->deu_bits) | d;
                return 0;
            }
        }
        return -1;    /* stem matched but ending is not in the table */
    }
    return -1;
}

/* The reverse of find_token(): given a numeric layer index, writes
 * out the corresponding word (stem, then ending if any) as raw
 * characters at `dst` (no separators, no NUL terminator - the caller
 * adds those). Returns how many characters were written, so the
 * caller knows where to continue writing. `index` is split back into
 * its stem/ending halves the same way name_data.c packed them:
 * high bits = stem index, low deu_bits bits = ending index.
 * Never writes more than `cap` bytes: if the token would not fit,
 * nothing is written and 0 is returned (a table word that long means
 * the generated tables violate the 31-char hostname invariant). */
static size_t emit_token(const name_layer_t *layer, uint32_t index,
                         char *dst, size_t cap)
{
    uint32_t deu = index & ((1u << layer->deu_bits) - 1u);
    uint32_t pro = index >> layer->deu_bits;
    size_t pl, dl;
    const char *pw = pro_word(layer, pro, &pl);
    const char *dw = deu_word(layer, deu, &dl);
    if (pl + dl > cap)
        return 0;
    memcpy(dst, pw, pl);
    memcpy(dst + pl, dw, dl);
    return pl + dl;
}

/* ------------------------------------------------------------------ */
/* public API                                                         */
/* ------------------------------------------------------------------ */

/* Splits a dashed hostname string into its 4 word tokens, without
 * copying any characters - `tokens[i]` just points at the start of
 * each word inside the original `hostname` string, and
 * `token_lens[i]` says how many characters long it is. This also
 * doubles as basic input validation: it rejects anything that isn't
 * exactly 4 dash-separated runs of ASCII letters (so it's safe to
 * call on untrusted/unknown strings before trying to decode them). */
int mac_hostname_split(const char *hostname,
                       const char *tokens[MAC_HOSTNAME_N_TOKENS],
                       size_t token_lens[MAC_HOSTNAME_N_TOKENS])
{
    if (!hostname || !tokens || !token_lens || hostname[0] == '\0') return -1;
    size_t total_len = bounded_strlen(hostname, MAC_HOSTNAME_BUF_LEN);
    if (total_len == 0 || total_len > MAC_HOSTNAME_MAX_LEN) return -1;

    unsigned count = 0;
    const char *start = hostname;
    /* Walk the string once, character by character. Every '-' or the
     * final '\0' closes off the token that started at `start`. Any
     * character that isn't a letter or a separator is an immediate
     * error - digits, punctuation etc. never appear in a hostname
     * this scheme generates. */
    for (const char *p = hostname;; ++p) {
        unsigned char c = (unsigned char)*p;
        if (c == '-' || c == '\0') {
            if (p == start || count >= MAC_HOSTNAME_N_TOKENS) return -1;
            tokens[count] = start;
            token_lens[count] = (size_t)(p - start);
            ++count;
            if (c == '\0') break;
            start = p + 1;
        } else if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'))) {
            return -1;
        }
    }
    return count == MAC_HOSTNAME_N_TOKENS ? (int)count : -1;
}

/* Reformats a hostname like "noble-abbot-marc-hark" into two shorter
 * lines ("noble-abbot" / "marc-hark") for small displays that can't
 * fit the whole name on one row. Internally this is just
 * mac_hostname_split() followed by copying the first pair of tokens
 * into line1 and the second pair into line2, with a size check to
 * make sure the caller's buffers are big enough first. */
int mac_hostname_display_lines(const char *hostname,
                               char *line1, size_t line1_len,
                               char *line2, size_t line2_len)
{
    if (!hostname || !line1 || !line2 || line1_len == 0 || line2_len == 0) {
        return -1;
    }

    /* Always leave valid empty strings on failure. */
    line1[0] = '\0';
    line2[0] = '\0';

    const char *tokens[MAC_HOSTNAME_N_TOKENS];
    size_t lengths[MAC_HOSTNAME_N_TOKENS];

    if (mac_hostname_split(hostname, tokens, lengths) !=
        MAC_HOSTNAME_N_TOKENS) {
        return -1;
    }

    /*
     * Line 1: prefix-title   (worst case 6+1+6  = 13 chars + NUL = 14)
     * Line 2: first-last     (worst case 8+1+8  = 17 chars + NUL = 18)
     */
    const size_t line1_required = lengths[0] + 1u + lengths[1];
    const size_t line2_required = lengths[2] + 1u + lengths[3];

    if (line1_required + 1u > line1_len ||
        line2_required + 1u > line2_len) {
        return -1;
    }

    memcpy(line1, tokens[0], lengths[0]);
    line1[lengths[0]] = '-';
    memcpy(line1 + lengths[0] + 1u, tokens[1], lengths[1]);
    line1[line1_required] = '\0';

    memcpy(line2, tokens[2], lengths[2]);
    line2[lengths[2]] = '-';
    memcpy(line2 + lengths[2] + 1u, tokens[3], lengths[3]);
    line2[line2_required] = '\0';

    return 0;
}

/* MAC -> hostname. Overall flow:
 *   1. Pack the 6 MAC bytes into one 48-bit number.
 *   2. Scramble that number with the Feistel permutation (so similar
 *      MACs don't produce similar-looking names - see the Feistel
 *      section at the top of this file).
 *   3. Slice the scrambled number into 4 chunks (one per token: 8,
 *      7, 17, 16 bits) using each layer's `bits` field.
 *   4. Turn each chunk into a word via emit_token(), joining them
 *      with dashes, to build the final hostname string.
 */
int mac_hostname_from_mac(const uint8_t mac[6], char *out, size_t out_len,
                          mac_hostname_format_t format)
{
    if (!mac || !out || out_len == 0 || format != MAC_HOSTNAME_DASHED) return -1;
    out[0] = '\0';

    /* Step 1: 6 bytes -> one 48-bit integer, most-significant byte first
     * (this is the normal way a MAC address like AA:BB:CC:DD:EE:FF is
     * read as a number). */
    uint64_t value = 0;
    for (unsigned i = 0; i < 6; ++i) value = (value << 8) | mac[i];
    /* Step 2: scramble it. */
    value = feistel_forward(value);

    /* Step 3: peel chunks off the low (least-significant) end first,
     * so layer N-1 (surname) - the last token in the hostname - comes
     * from the lowest bits, and layer 0 (prefix) - the first token -
     * ends up with whatever's left in the highest bits. */
    uint32_t indexes[N_LAYERS];
    for (int i = N_LAYERS - 1; i >= 0; --i) {
        const name_layer_t *layer = &NAME_LAYERS[i];
        indexes[i] = (uint32_t)(value & ((1u << layer->bits) - 1u));
        value >>= layer->bits;
    }
    if (value != 0) return -2;   /* layer bit widths must sum to 48 */

    /* Step 4: render each chunk as a word and join with dashes. Every
     * write is bounds-checked BEFORE it happens (the previous version
     * wrote first and length-checked after — safe with the current
     * generated tables, whose worst case is exactly 31 chars, but a
     * latent stack overflow if the tables were ever regenerated with
     * longer words). One byte is always reserved for the NUL. */
    char buf[MAC_HOSTNAME_MAX_LEN + 1];
    size_t written = 0;
    for (unsigned i = 0; i < N_LAYERS; ++i) {
        if (i) {
            if (written + 2 > sizeof(buf)) return -2; /* '-' plus the NUL */
            buf[written++] = '-';
        }
        size_t n = emit_token(&NAME_LAYERS[i], indexes[i], buf + written,
                              sizeof(buf) - 1 - written);
        if (n == 0) return -2;   /* token would not fit: corrupt tables */
        written += n;
    }
    buf[written] = '\0';

    /* Only copy into the caller's buffer once we know the whole
     * result (including the NUL terminator) actually fits. */
    if (written > MAC_HOSTNAME_MAX_LEN || written + 1u > out_len) {
        return -1;
    }
    memcpy(out, buf, written + 1u);
    return (int)written;
}

/* hostname -> MAC. This is mac_hostname_from_mac() run backwards:
 *   1. Split the hostname into its 4 word tokens.
 *   2. Look each token up (find_token()) to recover the numeric
 *      chunk it represents, and reassemble the 4 chunks into one
 *      48-bit number (mirroring how mac_hostname_from_mac() split it).
 *   3. Undo the Feistel scramble to get back the original MAC value.
 *   4. Unpack that 48-bit number into 6 individual MAC bytes.
 */
int mac_hostname_to_mac(const char *hostname, uint8_t mac[6])
{
    if (!hostname || !mac) return -1;
    const char *tokens[N_LAYERS];
    size_t lengths[N_LAYERS];
    if (mac_hostname_split(hostname, tokens, lengths) != N_LAYERS) return -1;

    /* Step 2: pack MSB-first: layer 0 (prefix) into the high bits,
     * the reverse order of how mac_hostname_from_mac() unpacked them. */
    uint64_t value = 0;
    for (unsigned i = 0; i < N_LAYERS; ++i) {
        uint32_t index = 0;
        if (find_token(&NAME_LAYERS[i], tokens[i], lengths[i], &index) != 0) return -1;
        value = (value << NAME_LAYERS[i].bits) | index;
    }

    /* Step 3: unscramble. */
    value = feistel_inverse(value);
    /* Step 4: 48-bit integer -> 6 bytes, most-significant byte first,
     * undoing the packing done at the start of mac_hostname_from_mac(). */
    for (int i = 5; i >= 0; --i) {
        mac[i] = (uint8_t)(value & UINT64_C(0xFF));
        value >>= 8;
    }
    return 0;
}
