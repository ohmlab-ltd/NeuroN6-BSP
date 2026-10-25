/* Host test of the link key's SHA-256 and HMAC-SHA256
 * (Middlewares/NeuroN6/MayaW476/maya_w4_board/maya_w4_sha256.c) against the FIPS
 * 180-4 and RFC 4231 vectors. From this folder:
 *
 *   MSVC (x64 Native Tools prompt):
 *     cl /nologo /W3 /std:c11 /I..\..\..\Middlewares\NeuroN6\MayaW476\maya_w4_board test_sha256.c ..\..\..\Middlewares\NeuroN6\MayaW476\maya_w4_board\maya_w4_sha256.c /Fe:%TEMP%\test_sha256.exe
 *   gcc:
 *     gcc -std=c11 -Wall -I../../../Middlewares/NeuroN6/MayaW476/maya_w4_board test_sha256.c ../../../Middlewares/NeuroN6/MayaW476/maya_w4_board/maya_w4_sha256.c -o /tmp/test_sha256
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <string.h>
#include "maya_w4_sha256.h"

static int fails, checks;

static void hex(const uint8_t *p, size_t n, char *out)
{
  for (size_t i = 0; i < n; i++) snprintf(out + 2 * i, 3, "%02x", p[i]);
}

static void expect(const char *name, const uint8_t d[32], const char *want)
{
  char got[65];
  hex(d, 32, got);
  checks++;
  if (strcmp(got, want) != 0) {
    fails++;
    printf("FAIL %s\n  got  %s\n  want %s\n", name, got, want);
  }
}

static void sha(const char *name, const void *m, size_t n, const char *want)
{
  uint8_t d[32];
  maya_w4_sha256_t c;
  maya_w4_sha256_init(&c);
  maya_w4_sha256_update(&c, m, n);
  maya_w4_sha256_final(&c, d);
  expect(name, d, want);
}

static void hmac(const char *name, const uint8_t *k, size_t kn, const void *m, size_t n, const char *want)
{
  uint8_t d[32];
  maya_w4_hmac_sha256(k, kn, m, n, NULL, 0, d);
  expect(name, d, want);
  /* The same message split in two parts. */
  maya_w4_hmac_sha256(k, kn, m, n / 2, (const uint8_t *)m + n / 2, n - n / 2, d);
  expect(name, d, want);
}

int main(void)
{
  static uint8_t million[1000000];
  uint8_t k[131];

  sha("sha256 empty", "", 0, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  sha("sha256 abc", "abc", 3, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  sha("sha256 two blocks", "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq", 56,
      "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  memset(million, 'a', sizeof(million));
  sha("sha256 a million a", million, sizeof(million),
      "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");

  memset(k, 0x0b, 20);
  hmac("rfc4231 case 1", k, 20, "Hi There", 8,
       "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7");
  hmac("rfc4231 case 2", (const uint8_t *)"Jefe", 4, "what do ya want for nothing?", 28,
       "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843");
  memset(k, 0xaa, 20);
  {
    uint8_t m[50];
    memset(m, 0xdd, sizeof(m));
    hmac("rfc4231 case 3", k, 20, m, sizeof(m),
         "773ea91e36800e46854db8ebd09181a72959098b3ef8c122d9635514ced565fe");
  }
  memset(k, 0xaa, 131);
  hmac("rfc4231 case 6", k, 131, "Test Using Larger Than Block-Size Key - Hash Key First", 54,
       "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54");
  {
    static const char m[] = "This is a test using a larger than block-size key and a larger than "
                            "block-size data. The key needs to be hashed before being used by the "
                            "HMAC algorithm.";
    hmac("rfc4231 case 7", k, 131, m, sizeof(m) - 1,
         "9b09ffa71b942fcb27635fbcd5b0e944bfdc63644f0713938a7f51535c3a35e2");
  }

  printf("%d checks, %d failed\n", checks, fails);
  return fails != 0;
}
