#include "spdif.h"

#include "esphome/core/defines.h"
#include "esphome/core/log.h"

#include <esp_timer.h>
namespace esphome {
namespace spdif_audio {

static const char *const TAG = "spdif";

/*
 * 8bit PCM to 16bit BMC conversion table, LSb first, 1 end
 */
static const uint16_t BMC_TABLE[256] = {
    0x3333, 0xb333, 0xd333, 0x5333, 0xcb33, 0x4b33, 0x2b33, 0xab33, 0xcd33, 0x4d33, 0x2d33, 0xad33, 0x3533, 0xb533,
    0xd533, 0x5533, 0xccb3, 0x4cb3, 0x2cb3, 0xacb3, 0x34b3, 0xb4b3, 0xd4b3, 0x54b3, 0x32b3, 0xb2b3, 0xd2b3, 0x52b3,
    0xcab3, 0x4ab3, 0x2ab3, 0xaab3, 0xccd3, 0x4cd3, 0x2cd3, 0xacd3, 0x34d3, 0xb4d3, 0xd4d3, 0x54d3, 0x32d3, 0xb2d3,
    0xd2d3, 0x52d3, 0xcad3, 0x4ad3, 0x2ad3, 0xaad3, 0x3353, 0xb353, 0xd353, 0x5353, 0xcb53, 0x4b53, 0x2b53, 0xab53,
    0xcd53, 0x4d53, 0x2d53, 0xad53, 0x3553, 0xb553, 0xd553, 0x5553, 0xcccb, 0x4ccb, 0x2ccb, 0xaccb, 0x34cb, 0xb4cb,
    0xd4cb, 0x54cb, 0x32cb, 0xb2cb, 0xd2cb, 0x52cb, 0xcacb, 0x4acb, 0x2acb, 0xaacb, 0x334b, 0xb34b, 0xd34b, 0x534b,
    0xcb4b, 0x4b4b, 0x2b4b, 0xab4b, 0xcd4b, 0x4d4b, 0x2d4b, 0xad4b, 0x354b, 0xb54b, 0xd54b, 0x554b, 0x332b, 0xb32b,
    0xd32b, 0x532b, 0xcb2b, 0x4b2b, 0x2b2b, 0xab2b, 0xcd2b, 0x4d2b, 0x2d2b, 0xad2b, 0x352b, 0xb52b, 0xd52b, 0x552b,
    0xccab, 0x4cab, 0x2cab, 0xacab, 0x34ab, 0xb4ab, 0xd4ab, 0x54ab, 0x32ab, 0xb2ab, 0xd2ab, 0x52ab, 0xcaab, 0x4aab,
    0x2aab, 0xaaab, 0xcccd, 0x4ccd, 0x2ccd, 0xaccd, 0x34cd, 0xb4cd, 0xd4cd, 0x54cd, 0x32cd, 0xb2cd, 0xd2cd, 0x52cd,
    0xcacd, 0x4acd, 0x2acd, 0xaacd, 0x334d, 0xb34d, 0xd34d, 0x534d, 0xcb4d, 0x4b4d, 0x2b4d, 0xab4d, 0xcd4d, 0x4d4d,
    0x2d4d, 0xad4d, 0x354d, 0xb54d, 0xd54d, 0x554d, 0x332d, 0xb32d, 0xd32d, 0x532d, 0xcb2d, 0x4b2d, 0x2b2d, 0xab2d,
    0xcd2d, 0x4d2d, 0x2d2d, 0xad2d, 0x352d, 0xb52d, 0xd52d, 0x552d, 0xccad, 0x4cad, 0x2cad, 0xacad, 0x34ad, 0xb4ad,
    0xd4ad, 0x54ad, 0x32ad, 0xb2ad, 0xd2ad, 0x52ad, 0xcaad, 0x4aad, 0x2aad, 0xaaad, 0x3335, 0xb335, 0xd335, 0x5335,
    0xcb35, 0x4b35, 0x2b35, 0xab35, 0xcd35, 0x4d35, 0x2d35, 0xad35, 0x3535, 0xb535, 0xd535, 0x5535, 0xccb5, 0x4cb5,
    0x2cb5, 0xacb5, 0x34b5, 0xb4b5, 0xd4b5, 0x54b5, 0x32b5, 0xb2b5, 0xd2b5, 0x52b5, 0xcab5, 0x4ab5, 0x2ab5, 0xaab5,
    0xccd5, 0x4cd5, 0x2cd5, 0xacd5, 0x34d5, 0xb4d5, 0xd4d5, 0x54d5, 0x32d5, 0xb2d5, 0xd2d5, 0x52d5, 0xcad5, 0x4ad5,
    0x2ad5, 0xaad5, 0x3355, 0xb355, 0xd355, 0x5355, 0xcb55, 0x4b55, 0x2b55, 0xab55, 0xcd55, 0x4d55, 0x2d55, 0xad55,
    0x3555, 0xb555, 0xd555, 0x5555,
};

// BMC preamble
static const uint32_t BMC_B = 0x33173333;  // block start
static const uint32_t BMC_M = 0x331d3333;  // left ch
static const uint32_t BMC_W = 0x331b3333;  // right ch
static const uint32_t BMC_MW_DIF = (BMC_M ^ BMC_W);
static const uint8_t SYNC_OFFSET = 2;  // byte offset of SYNC
static const uint32_t SYNC_FLIP = ((BMC_B ^ BMC_M) >> (SYNC_OFFSET * 8));

// initialize S/PDIF buffer
void SPDIF::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SPDIF...");

  int i;
  uint32_t bmc_mw = BMC_W;

  for (i = 0; i < SPDIF_BLOCK_SIZE_U32; i += 2) {
    spdif_block_buf_[i] = bmc_mw ^= BMC_MW_DIF;
  }
  ESP_LOGD(TAG, "SPDIF buffer initialized to %zu bytes", sizeof(spdif_block_buf_));

  spdif_block_ptr_ = spdif_block_buf_;
}

esp_err_t SPDIF::write(const uint8_t *src, size_t size, TickType_t ticks_to_wait) {
  const uint8_t *p = reinterpret_cast<const uint8_t *>(src);

  while (p < (uint8_t *) src + size) {
    // convert PCM 16bit data to BMC 32bit pulse pattern (which is 64 i2s bits to emulate BMC)
    // We cast to int16_t to avoid sign extension issues when XOR-ing
    *(spdif_block_ptr_ + 1) =
        (uint32_t) (((static_cast<int16_t>(BMC_TABLE[*p]) << 16) ^ static_cast<int16_t>(BMC_TABLE[*(p + 1)])) << 1) >>
        1;

    p += 2;
    spdif_block_ptr_ += 2;  // advance to next audio data

    if (spdif_block_ptr_ >= &spdif_block_buf_[SPDIF_BLOCK_SIZE_U32]) {
      // set block start preamble
      ((uint8_t *) spdif_block_buf_)[SYNC_OFFSET] ^= SYNC_FLIP;

      esp_err_t err = block_complete_callback_(spdif_block_buf_, sizeof(spdif_block_buf_), ticks_to_wait);
      if (err != ESP_OK) {
        return err;
      }

      spdif_block_ptr_ = spdif_block_buf_;
    }
  }

  return ESP_OK;
}

}  // namespace spdif_audio
}  // namespace esphome
