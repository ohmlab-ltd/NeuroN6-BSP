/*
 * The Lepton video parser, LEPTON_VoSPI_Parse in Components/lepton/lepton.c,
 * fed synthetic streams on the PC. Each pixel of a synthetic frame is a
 * function of the frame and the pixel's place, so a delivered frame either is
 * one whole frame of the stream or is wrong in a way that names its pieces.
 *
 * What must hold:
 *   - At the camera's own cadence, one frame and then eight segments of
 *     filler, with discard packets anywhere, every frame is delivered, whole,
 *     in order, however the stream is cut into blocks.
 *   - With frames back to back, so that the end of one and the start of the
 *     next fall in one block, no delivered frame is part one frame and part
 *     the next. Frames may be lost; none is torn.
 *   - One damaged packet - dropped, repeated, renumbered, a header with bits
 *     the protocol never sets, a line held low, a line held high - costs the
 *     frame it is in and no other.
 *   - A frame whose segments arrive in two different buffers is not delivered.
 *
 *   gcc -std=c11 -Wall -I../../../Components/lepton test_vospi.c ../../../Components/lepton/lepton.c
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lepton.h"

#define PKT    LEPTON_PACKET_BYTES
#define PPS    LEPTON_PACKETS_PER_SEGMENT
#define W      LEPTON_WIDTH
#define PIX    LEPTON_FRAME_PIXELS

static int checks = 0;
static int failures = 0;

#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; printf("FAIL  " __VA_ARGS__); printf("\n"); } } while (0)

/* Streams. */

static uint32_t rng;

static uint32_t rnd(void)
{
  rng ^= rng << 13;
  rng ^= rng >> 17;
  rng ^= rng << 5;
  return rng;
}

/* The value pixel i of frame f carries. */
static uint16_t value(uint32_t f, uint32_t i)
{
  uint32_t h = (f + 1U) * 2654435761U ^ (i * 40503U);
  h ^= h >> 15;
  return (uint16_t)(h * 2246822519U >> 16);
}

static uint8_t *stream;
static size_t   packets;
static size_t   capacity;

static uint8_t *next_packet(void)
{
  if (packets == capacity)
  {
    capacity = (capacity * 2U) + 256U;
    stream = realloc(stream, capacity * PKT);
    if (stream == NULL) { exit(2); }
  }
  return &stream[(packets++) * PKT];
}

static void discards(uint32_t per_thousand)
{
  while ((rnd() % 1000U) < per_thousand)
  {
    uint8_t *p = next_packet();
    for (size_t b = 0; b < PKT; b++) { p[b] = (uint8_t)rnd(); }
    p[0] = (uint8_t)(p[0] | 0x0FU);   /* a discard packet: ID nibble 0xF */
  }
}

/* A segment's packets. id is 1 to 4 for frame f's segment, 0 for filler. */
static void segment(uint32_t f, uint32_t id, uint32_t discard_rate)
{
  for (uint32_t n = 0U; n < PPS; n++)
  {
    uint16_t header = (uint16_t)n;
    uint8_t *p;

    discards(discard_rate);
    if (n == LEPTON_SEGMENT_ID_PACKET) { header |= (uint16_t)(id << 12); }
    p = next_packet();
    p[0] = (uint8_t)(header >> 8);
    p[1] = (uint8_t)header;
    p[2] = (uint8_t)rnd();
    p[3] = (uint8_t)rnd();
    for (uint32_t x = 0U; x < LEPTON_PACKET_PIXELS; x++)
    {
      /* Two packets a row: a segment is 30 rows of the frame. */
      uint32_t row = ((id - 1U) * LEPTON_SEGMENT_ROWS) + (n / 2U);
      uint32_t col = ((n % 2U) * LEPTON_PACKET_PIXELS) + x;
      uint16_t v = (id == 0U) ? (uint16_t)rnd() : value(f, (row * W) + col);

      p[4U + (2U * x)] = (uint8_t)(v >> 8);
      p[5U + (2U * x)] = (uint8_t)v;
    }
  }
}

/* Which frame this is, whole: its number, -1 for none, -2 for torn. */
static int32_t which(const uint16_t *frame, uint32_t frames)
{
  for (uint32_t f = 0U; f < frames; f++)
  {
    if (frame[0] == value(f, 0U))
    {
      for (uint32_t i = 0U; i < PIX; i++)
      {
        if (frame[i] != value(f, i)) { return -2; }
      }
      return (int32_t)f;
    }
  }
  return -1;
}

/* Parse the stream in blocks of `block` packets into one buffer, recording
 * what each completed frame turned out to be. Returns how many completed. */
static uint32_t parse(uint32_t block, uint32_t frames, int32_t *got, uint32_t got_max)
{
  static uint16_t frame[PIX];
  LEPTON_VoSPI_t parser;
  uint32_t n = 0U;

  (void)LEPTON_VoSPI_Init(&parser);
  for (size_t k = 0U; k < packets; k += block)
  {
    uint32_t count = (uint32_t)(((packets - k) < block) ? (packets - k) : block);
    uint32_t recognised = 0U;

    if (LEPTON_VoSPI_Parse(&parser, &stream[k * PKT], count, frame, &recognised) == 1)
    {
      if (n < got_max) { got[n] = which(frame, frames); }
      n++;
    }
  }
  return n;
}

/* The cases. */

static void cadence(int cases)
{
  int32_t got[64];

  for (int c = 0; c < cases; c++)
  {
    uint32_t frames = 6U + (rnd() % 7U);
    uint32_t rate = rnd() % 400U;
    uint32_t block = 4U * (1U + (rnd() % 49U));
    uint32_t delivered;
    int good = 1;

    packets = 0U;
    for (uint32_t n = rnd() % PPS; n < PPS; n++) { (void)next_packet(); stream[(packets - 1U) * PKT] = 0x0F; }
    for (uint32_t f = 0U; f < frames; f++)
    {
      for (uint32_t s = 1U; s <= LEPTON_SEGMENTS; s++) { segment(f, s, rate); }
      for (uint32_t z = 0U; z < 8U; z++) { segment(0U, 0U, rate); }
    }
    delivered = parse(block, frames, got, 64U);
    for (uint32_t i = 0U; (i < delivered) && (i < 64U); i++) { good &= (got[i] == (int32_t)i); }
    CHECK((delivered == frames) && good,
          "cadence case %d: %u of %u frames, in order and whole: %s (blocks of %u packets, discards %u/1000)",
          c, delivered, frames, good ? "yes" : "no", block, rate);
  }
}

static void back_to_back(int cases)
{
  int32_t got[64];
  uint32_t torn_cases = 0U;
  uint32_t lost = 0U;
  uint32_t total = 0U;

  for (int c = 0; c < cases; c++)
  {
    uint32_t frames = 8U + (rnd() % 8U);
    uint32_t rate = rnd() % 100U;
    uint32_t block = 4U * (16U + (rnd() % 34U));   /* 64 to 196 packets */
    uint32_t delivered;
    uint32_t torn = 0U;
    int32_t  last = -1;
    int      ordered = 1;

    packets = 0U;
    for (uint32_t f = 0U; f < frames; f++)
    {
      for (uint32_t s = 1U; s <= LEPTON_SEGMENTS; s++) { segment(f, s, rate); }
    }
    delivered = parse(block, frames, got, 64U);
    for (uint32_t i = 0U; (i < delivered) && (i < 64U); i++)
    {
      if (got[i] < 0) { torn++; }
      else { ordered &= (got[i] > last); last = got[i]; }
    }
    torn_cases += (torn != 0U) ? 1U : 0U;
    total += frames;
    lost += frames - delivered;
    CHECK((torn == 0U) && ordered,
          "back to back, case %d: %u frames delivered, %u of them torn (blocks of %u packets)",
          c, delivered, torn, block);
  }
  printf("      back to back: %u frames sent, %u lost, %u cases with a torn frame\n",
         total, lost, torn_cases);
}

enum { DROP, REPEAT, RENUMBER, HIGH_BITS, HELD_LOW, HELD_HIGH, DAMAGE_KINDS };
static const char *const damage_name[DAMAGE_KINDS] =
  { "dropped", "repeated", "renumbered", "ID bits 11:8 set", "line held low", "line held high" };

static void damage(int cases)
{
  int32_t got[64];

  for (int c = 0; c < cases; c++)
  {
    const int kind = (int)(rnd() % DAMAGE_KINDS);
    const uint32_t frames = 7U;
    const uint32_t hit = 2U + (rnd() % 3U);           /* the damaged frame */
    const uint32_t seg = 1U + (rnd() % LEPTON_SEGMENTS);
    /* Two exceptions, neither a fault: a repeat of a segment's last packet
     * comes after the segment is complete and is ignored, and a line held low
     * for exactly packet 0 reads as a packet 0. Telling that from the real one
     * would take the packet's CRC, which the parser does not check. */
    const uint32_t pkt = (kind == REPEAT)   ? (rnd() % (PPS - 1U))
                       : (kind == HELD_LOW) ? (1U + (rnd() % (PPS - 1U)))
                       : (rnd() % PPS);
    const uint32_t block = 4U * (1U + (rnd() % 49U));
    uint32_t delivered;
    int good = 1;

    packets = 0U;
    for (uint32_t f = 0U; f < frames; f++)
    {
      for (uint32_t s = 1U; s <= LEPTON_SEGMENTS; s++)
      {
        size_t first = packets;

        segment(f, s, 0U);
        if ((f == hit) && (s == seg))
        {
          uint8_t *p = &stream[(first + pkt) * PKT];

          switch (kind)
          {
            case DROP:
              memmove(p, p + PKT, (packets - first - pkt - 1U) * PKT);
              packets--;
              break;
            case REPEAT:
            {
              size_t tail = packets - first - pkt;
              (void)next_packet();
              p = &stream[(first + pkt) * PKT];
              memmove(p + PKT, p, tail * PKT);
              break;
            }
            case RENUMBER:
              p[1] = (uint8_t)((pkt + 1U + (rnd() % (PPS - 1U))) % PPS);
              break;
            case HIGH_BITS:
              p[0] = (uint8_t)((p[0] & 0xF0U) | (1U + (rnd() % 14U)));
              break;
            case HELD_LOW:
              memset(p, 0x00, PKT);
              break;
            default:
              memset(p, 0xFF, PKT);
              break;
          }
        }
      }
      for (uint32_t z = 0U; z < 8U; z++) { segment(0U, 0U, 0U); }
    }
    delivered = parse(block, frames, got, 64U);
    for (uint32_t i = 0U, f = 0U; i < delivered; i++, f++)
    {
      if (f == hit) { f++; }
      good &= (got[i] == (int32_t)f);
    }
    CHECK((delivered == frames - 1U) && good,
          "damage case %d, packet %u of segment %u of frame %u %s: %u of %u frames delivered, the others whole: %s",
          c, pkt, seg, hit, damage_name[kind], delivered, frames, good ? "yes" : "no");
  }
}

static void split(void)
{
  static uint16_t a[PIX];
  static uint16_t b[PIX];
  LEPTON_VoSPI_t parser;
  uint32_t half;
  int r1, r2, r3;

  packets = 0U;
  for (uint32_t f = 0U; f < 2U; f++)
  {
    for (uint32_t s = 1U; s <= LEPTON_SEGMENTS; s++) { segment(f, s, 0U); }
    for (uint32_t z = 0U; z < 8U; z++) { segment(0U, 0U, 0U); }
  }
  half = 2U * PPS;   /* frame 0's first two segments */
  (void)LEPTON_VoSPI_Init(&parser);
  r1 = (int)LEPTON_VoSPI_Parse(&parser, stream, half, a, NULL);
  r2 = (int)LEPTON_VoSPI_Parse(&parser, &stream[half * PKT], (uint32_t)(12U * PPS) - half, b, NULL);
  r3 = (int)LEPTON_VoSPI_Parse(&parser, &stream[12U * PPS * PKT], (uint32_t)(packets - (12U * PPS)), b, NULL);
  CHECK((r1 == 0) && (r2 == 0), "a frame split across two buffers is not delivered (%d %d)", r1, r2);
  CHECK((r3 == 1) && (which(b, 2U) == 1), "the next frame, all in one buffer, is (%d, frame %ld)",
        r3, (long)which(b, 2U));
}

/* A line held low reads as packet 0 over and over: none of it is well
 * formed. A clean stream counts every packet, its segments' first packets
 * included. */
static void held_low(void)
{
  static uint8_t  zeros[196U * PKT];
  static uint16_t out[PIX];
  LEPTON_VoSPI_t parser;
  uint32_t recognised = 0U;

  (void)LEPTON_VoSPI_Init(&parser);
  (void)LEPTON_VoSPI_Parse(&parser, zeros, 196U, out, &recognised);
  CHECK(recognised == 0U, "a line held low counts %lu of 196 packets well formed",
        (unsigned long)recognised);

  packets = 0U;
  for (uint32_t s = 1U; s <= LEPTON_SEGMENTS; s++) { segment(0U, s, 0U); }
  (void)LEPTON_VoSPI_Init(&parser);
  recognised = 0U;
  (void)LEPTON_VoSPI_Parse(&parser, stream, (uint32_t)packets, out, &recognised);
  CHECK(recognised == (uint32_t)packets, "a clean stream counts %lu of %lu packets",
        (unsigned long)recognised, (unsigned long)packets);
}

int main(int argc, char **argv)
{
  int cases = (argc > 1) ? atoi(argv[1]) : 300;

  rng = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 0) : 20261003U;
  if (rng == 0U) { rng = 1U; }

  cadence(cases);
  back_to_back(cases);
  damage(cases);
  split();
  held_low();

  printf("%d checks, %d failed (the Lepton video parser)\n", checks, failures);
  free(stream);
  return (failures == 0) ? 0 : 1;
}
