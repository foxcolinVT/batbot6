/*
 * Author: Ben Westcott
 * Date created: 8/10/22
 */


#define SINE_LEN 256

uint16_t SINE_WAVE_TBL[SINE_LEN] = {
      0x400,0x419,0x432,0x44b,0x464,0x47d,0x496,0x4af,
      0x4c8,0x4e0,0x4f9,0x511,0x529,0x541,0x559,0x571,
      0x588,0x59f,0x5b6,0x5cc,0x5e3,0x5f9,0x60e,0x624,
      0x639,0x64e,0x662,0x676,0x68a,0x69d,0x6b0,0x6c2,
      0x6d4,0x6e6,0x6f7,0x707,0x718,0x727,0x736,0x745,
      0x753,0x761,0x76e,0x77b,0x787,0x793,0x79e,0x7a8,
      0x7b2,0x7bb,0x7c4,0x7cc,0x7d4,0x7db,0x7e1,0x7e7,
      0x7ec,0x7f1,0x7f5,0x7f8,0x7fb,0x7fd,0x7ff,0x800,
      0x800,0x800,0x7ff,0x7fd,0x7fb,0x7f8,0x7f5,0x7f1,
      0x7ec,0x7e7,0x7e1,0x7db,0x7d4,0x7cc,0x7c4,0x7bb,
      0x7b2,0x7a8,0x79e,0x793,0x787,0x77b,0x76e,0x761,
      0x753,0x745,0x736,0x727,0x718,0x707,0x6f7,0x6e6,
      0x6d4,0x6c2,0x6b0,0x69d,0x68a,0x676,0x662,0x64e,
      0x639,0x624,0x60e,0x5f9,0x5e3,0x5cc,0x5b6,0x59f,
      0x588,0x571,0x559,0x541,0x529,0x511,0x4f9,0x4e0,
      0x4c8,0x4af,0x496,0x47d,0x464,0x44b,0x432,0x419,
      0x400,0x3e7,0x3ce,0x3b5,0x39c,0x383,0x36a,0x351,
      0x338,0x320,0x307,0x2ef,0x2d7,0x2bf,0x2a7,0x28f,
      0x278,0x261,0x24a,0x234,0x21d,0x207,0x1f2,0x1dc,
      0x1c7,0x1b2,0x19e,0x18a,0x176,0x163,0x150,0x13e,
      0x12c,0x11a,0x109,0xf9,0xe8,0xd9,0xca,0xbb,0xad,
      0x9f,0x92,0x85,0x79,0x6d,0x62,0x58,0x4e,0x45,0x3c,
      0x34,0x2c,0x25,0x1f,0x19,0x14,0xf,0xb,0x8,0x5,0x3,
      0x1,0x0,0x0,0x0,0x1,0x3,0x5,0x8,0xb,0xf,0x14,0x19,
      0x1f,0x25,0x2c,0x34,0x3c,0x45,0x4e,0x58,0x62,0x6d,
      0x79,0x85,0x92,0x9f,0xad,0xbb,0xca,0xd9,0xe8,0xf9,
      0x109,0x11a,0x12c,0x13e,0x150,0x163,0x176,0x18a,
      0x19e,0x1b2,0x1c7,0x1dc,0x1f2,0x207,0x21d,0x234,
      0x24a,0x261,0x278,0x28f,0x2a7,0x2bf,0x2d7,0x2ef,
      0x307,0x320,0x338,0x351,0x36a,0x383,0x39c,0x3b5,
      0x3ce,0x3e7
};

#define SINE_LEN_ALT 1024
#define SINE_ALT_MAX_AMPLITUDE 0xff

uint8_t SINE_WAVE_TBL_ALT[SINE_LEN_ALT]= {
        0x80,0x80,0x81,0x82,0x83,0x83,0x84,0x85,0x86,0x87,0x87,0x88,0x89,0x8a,0x8a,0x8b,
        0x8c,0x8d,0x8e,0x8e,0x8f,0x90,0x91,0x91,0x92,0x93,0x94,0x95,0x95,0x96,0x97,0x98,
        0x98,0x99,0x9a,0x9b,0x9b,0x9c,0x9d,0x9e,0x9f,0x9f,0xa0,0xa1,0xa2,0xa2,0xa3,0xa4,
        0xa5,0xa5,0xa6,0xa7,0xa8,0xa8,0xa9,0xaa,0xaa,0xab,0xac,0xad,0xad,0xae,0xaf,0xb0,
        0xb0,0xb1,0xb2,0xb2,0xb3,0xb4,0xb5,0xb5,0xb6,0xb7,0xb7,0xb8,0xb9,0xba,0xba,0xbb,
        0xbc,0xbc,0xbd,0xbe,0xbe,0xbf,0xc0,0xc0,0xc1,0xc2,0xc2,0xc3,0xc4,0xc4,0xc5,0xc6,
        0xc6,0xc7,0xc8,0xc8,0xc9,0xca,0xca,0xcb,0xcc,0xcc,0xcd,0xcd,0xce,0xcf,0xcf,0xd0,
        0xd0,0xd1,0xd2,0xd2,0xd3,0xd3,0xd4,0xd5,0xd5,0xd6,0xd6,0xd7,0xd7,0xd8,0xd9,0xd9,
        0xda,0xda,0xdb,0xdb,0xdc,0xdc,0xdd,0xde,0xde,0xdf,0xdf,0xe0,0xe0,0xe1,0xe1,0xe2,
        0xe2,0xe3,0xe3,0xe4,0xe4,0xe5,0xe5,0xe6,0xe6,0xe6,0xe7,0xe7,0xe8,0xe8,0xe9,0xe9,
        0xea,0xea,0xea,0xeb,0xeb,0xec,0xec,0xed,0xed,0xed,0xee,0xee,0xef,0xef,0xef,0xf0,
        0xf0,0xf0,0xf1,0xf1,0xf1,0xf2,0xf2,0xf2,0xf3,0xf3,0xf3,0xf4,0xf4,0xf4,0xf5,0xf5,
        0xf5,0xf6,0xf6,0xf6,0xf7,0xf7,0xf7,0xf7,0xf8,0xf8,0xf8,0xf8,0xf9,0xf9,0xf9,0xf9,
        0xfa,0xfa,0xfa,0xfa,0xfa,0xfb,0xfb,0xfb,0xfb,0xfb,0xfc,0xfc,0xfc,0xfc,0xfc,0xfc,
        0xfd,0xfd,0xfd,0xfd,0xfd,0xfd,0xfd,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,
        0xfe,0xfe,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
        0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe,
        0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfd,0xfd,0xfd,0xfd,0xfd,0xfd,0xfd,
        0xfd,0xfc,0xfc,0xfc,0xfc,0xfc,0xfb,0xfb,0xfb,0xfb,0xfb,0xfb,0xfa,0xfa,0xfa,0xfa,
        0xf9,0xf9,0xf9,0xf9,0xf8,0xf8,0xf8,0xf8,0xf7,0xf7,0xf7,0xf7,0xf6,0xf6,0xf6,0xf5,
        0xf5,0xf5,0xf5,0xf4,0xf4,0xf4,0xf3,0xf3,0xf3,0xf2,0xf2,0xf2,0xf1,0xf1,0xf1,0xf0,
        0xf0,0xef,0xef,0xef,0xee,0xee,0xee,0xed,0xed,0xec,0xec,0xeb,0xeb,0xeb,0xea,0xea,
        0xe9,0xe9,0xe8,0xe8,0xe8,0xe7,0xe7,0xe6,0xe6,0xe5,0xe5,0xe4,0xe4,0xe3,0xe3,0xe2,
        0xe2,0xe1,0xe1,0xe0,0xe0,0xdf,0xdf,0xde,0xde,0xdd,0xdd,0xdc,0xdc,0xdb,0xdb,0xda,
        0xd9,0xd9,0xd8,0xd8,0xd7,0xd7,0xd6,0xd5,0xd5,0xd4,0xd4,0xd3,0xd3,0xd2,0xd1,0xd1,
        0xd0,0xd0,0xcf,0xce,0xce,0xcd,0xcc,0xcc,0xcb,0xcb,0xca,0xc9,0xc9,0xc8,0xc7,0xc7,
        0xc6,0xc5,0xc5,0xc4,0xc3,0xc3,0xc2,0xc1,0xc1,0xc0,0xbf,0xbf,0xbe,0xbd,0xbd,0xbc,
        0xbb,0xbb,0xba,0xb9,0xb9,0xb8,0xb7,0xb6,0xb6,0xb5,0xb4,0xb4,0xb3,0xb2,0xb1,0xb1,
        0xb0,0xaf,0xaf,0xae,0xad,0xac,0xac,0xab,0xaa,0xa9,0xa9,0xa8,0xa7,0xa6,0xa6,0xa5,
        0xa4,0xa3,0xa3,0xa2,0xa1,0xa0,0xa0,0x9f,0x9e,0x9d,0x9d,0x9c,0x9b,0x9a,0x9a,0x99,
        0x98,0x97,0x96,0x96,0x95,0x94,0x93,0x93,0x92,0x91,0x90,0x90,0x8f,0x8e,0x8d,0x8c,
        0x8c,0x8b,0x8a,0x89,0x88,0x88,0x87,0x86,0x85,0x85,0x84,0x83,0x82,0x81,0x81,0x80,
        0x7f,0x7e,0x7e,0x7d,0x7c,0x7b,0x7a,0x7a,0x79,0x78,0x77,0x77,0x76,0x75,0x74,0x73,
        0x73,0x72,0x71,0x70,0x6f,0x6f,0x6e,0x6d,0x6c,0x6c,0x6b,0x6a,0x69,0x69,0x68,0x67,
        0x66,0x65,0x65,0x64,0x63,0x62,0x62,0x61,0x60,0x5f,0x5f,0x5e,0x5d,0x5c,0x5c,0x5b,
        0x5a,0x59,0x59,0x58,0x57,0x56,0x56,0x55,0x54,0x53,0x53,0x52,0x51,0x50,0x50,0x4f,
        0x4e,0x4e,0x4d,0x4c,0x4b,0x4b,0x4a,0x49,0x49,0x48,0x47,0x46,0x46,0x45,0x44,0x44,
        0x43,0x42,0x42,0x41,0x40,0x40,0x3f,0x3e,0x3e,0x3d,0x3c,0x3c,0x3b,0x3a,0x3a,0x39,
        0x38,0x38,0x37,0x36,0x36,0x35,0x34,0x34,0x33,0x33,0x32,0x31,0x31,0x30,0x2f,0x2f,
        0x2e,0x2e,0x2d,0x2c,0x2c,0x2b,0x2b,0x2a,0x2a,0x29,0x28,0x28,0x27,0x27,0x26,0x26,
        0x25,0x24,0x24,0x23,0x23,0x22,0x22,0x21,0x21,0x20,0x20,0x1f,0x1f,0x1e,0x1e,0x1d,
        0x1d,0x1c,0x1c,0x1b,0x1b,0x1a,0x1a,0x19,0x19,0x18,0x18,0x17,0x17,0x17,0x16,0x16,
        0x15,0x15,0x14,0x14,0x14,0x13,0x13,0x12,0x12,0x11,0x11,0x11,0x10,0x10,0x10,0x0f,
        0x0f,0x0e,0x0e,0x0e,0x0d,0x0d,0x0d,0x0c,0x0c,0x0c,0x0b,0x0b,0x0b,0x0a,0x0a,0x0a,
        0x0a,0x09,0x09,0x09,0x08,0x08,0x08,0x08,0x07,0x07,0x07,0x07,0x06,0x06,0x06,0x06,
        0x05,0x05,0x05,0x05,0x04,0x04,0x04,0x04,0x04,0x04,0x03,0x03,0x03,0x03,0x03,0x02,
        0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
        0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,
        0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x02,0x02,0x02,0x02,0x02,0x02,
        0x03,0x03,0x03,0x03,0x03,0x03,0x04,0x04,0x04,0x04,0x04,0x05,0x05,0x05,0x05,0x05,
        0x06,0x06,0x06,0x06,0x07,0x07,0x07,0x07,0x08,0x08,0x08,0x08,0x09,0x09,0x09,0x0a,
        0x0a,0x0a,0x0b,0x0b,0x0b,0x0c,0x0c,0x0c,0x0d,0x0d,0x0d,0x0e,0x0e,0x0e,0x0f,0x0f,
        0x0f,0x10,0x10,0x10,0x11,0x11,0x12,0x12,0x12,0x13,0x13,0x14,0x14,0x15,0x15,0x15,
        0x16,0x16,0x17,0x17,0x18,0x18,0x19,0x19,0x19,0x1a,0x1a,0x1b,0x1b,0x1c,0x1c,0x1d,
        0x1d,0x1e,0x1e,0x1f,0x1f,0x20,0x20,0x21,0x21,0x22,0x23,0x23,0x24,0x24,0x25,0x25,
        0x26,0x26,0x27,0x28,0x28,0x29,0x29,0x2a,0x2a,0x2b,0x2c,0x2c,0x2d,0x2d,0x2e,0x2f,
        0x2f,0x30,0x30,0x31,0x32,0x32,0x33,0x33,0x34,0x35,0x35,0x36,0x37,0x37,0x38,0x39,
        0x39,0x3a,0x3b,0x3b,0x3c,0x3d,0x3d,0x3e,0x3f,0x3f,0x40,0x41,0x41,0x42,0x43,0x43,
        0x44,0x45,0x45,0x46,0x47,0x48,0x48,0x49,0x4a,0x4a,0x4b,0x4c,0x4d,0x4d,0x4e,0x4f,
        0x4f,0x50,0x51,0x52,0x52,0x53,0x54,0x55,0x55,0x56,0x57,0x57,0x58,0x59,0x5a,0x5a,
        0x5b,0x5c,0x5d,0x5d,0x5e,0x5f,0x60,0x60,0x61,0x62,0x63,0x64,0x64,0x65,0x66,0x67,
        0x67,0x68,0x69,0x6a,0x6a,0x6b,0x6c,0x6d,0x6e,0x6e,0x6f,0x70,0x71,0x71,0x72,0x73,
        0x74,0x75,0x75,0x76,0x77,0x78,0x78,0x79,0x7a,0x7b,0x7c,0x7c,0x7d,0x7e,0x7f,0x80,
};
