#pragma once
#include "mdevice.hpp"
#include <array>
#include <algorithm>
#include <functional>
#include <cmath>

namespace mDev {

enum class Bitrate : uint32_t {
    DSHOT150  = 150'000,
    DSHOT300  = 300'000,
    DSHOT600  = 600'000,
    DSHOT1200 = 1'200'000,
};

class mDSHOT : public mDevice {
public:
    /* 构造时把定时器时钟（Hz）传进来 */
    explicit mDSHOT(const std::string& name, uint32_t tim_clk_hz)
        : mDevice(name),
          tim_clk_hz_(tim_clk_hz) {
        /* 使用浮点数计算：1 ns 需要多少个 timer tick */
        ns2tick_f_ = static_cast<double>(tim_clk_hz) / 1'000'000'000.0;
        recalcTiming();
    }

    /* 用户 API：油门 0-2047 */
    bool send(uint16_t throttle, bool telem = false) {
        throttle = std::min<uint16_t>(throttle, 2047);
        uint16_t frame = (throttle << 1) | (telem ? 1 : 0);
        uint8_t  c     = crc(frame);
        frame = (frame << 4) | c;
        fillDmaBuffer(frame);
        printf("throttle=%u telem=%d → 0x%04X  CRC=%01X\n", throttle, telem, frame, c);
        if (dmaTransferCompleteCb_ && !dmaTransferCompleteCb_()) return false;
        if (dmaTransferStartCb_) dmaTransferStartCb_(dma_buf_.data(), DMA_LEN);
        return true;
    }

    void registerDmaTransferCompleteCb(const std::function<bool()>& cb) {
        dmaTransferCompleteCb_ = cb;
    }
    void registerDmaTransferStartCb(const std::function<void(uint16_t*, uint32_t)>& cb) {
        dmaTransferStartCb_ = cb;
    }

    /* 换波特率/时钟后手动调用 */
    void recalcTiming() {
        const double bit_period_ns = 1'000'000'000.0 / static_cast<double>(bitrate_);
        t0h_ns_ = bit_period_ns * 0.3;  // 30% 高电平
        t1h_ns_ = bit_period_ns * 0.6;  // 60% 高电平
        
        // 使用浮点数计算tick数，然后四舍五入
        t0h_tick_f_ = t0h_ns_ * ns2tick_f_;
        t1h_tick_f_ = t1h_ns_ * ns2tick_f_;
        
        // 转换为整数，确保至少1个tick
        t0h_tick_ = std::max<uint16_t>(1, static_cast<uint16_t>(std::round(t0h_tick_f_)));
        t1h_tick_ = std::max<uint16_t>(1, static_cast<uint16_t>(std::round(t1h_tick_f_)));
        
        // 根据DShot频率和每ns的tick数精确计算timer_period
        // 一个完整的位周期需要的tick数
        const double bit_period_ticks = bit_period_ns * ns2tick_f_;
        
        // 确保timer_period能容纳整个位周期，加上10%的余量
        timer_period_ = static_cast<uint32_t>(std::round(bit_period_ticks));
        
        // 确保timer_period大于最大的高电平tick数
        uint16_t max_high_tick = std::max(t0h_tick_, t1h_tick_);
        if (timer_period_ <= max_high_tick) {
            timer_period_ = max_high_tick + 1;
        }
        
        printf("bitrate=%u Hz, bit_period=%.1f ns, bit_period_ticks=%.1f, timer_period_=%u\r\n", 
               static_cast<uint32_t>(bitrate_), bit_period_ns, bit_period_ticks, timer_period_);
        printf("t0h_ns_=%.1f, t1h_ns_=%.1f, t0h_tick_=%u, t1h_tick_=%u\r\n", 
               t0h_ns_, t1h_ns_, t0h_tick_, t1h_tick_);
    }

    void setBitrate(Bitrate b) { bitrate_ = b; recalcTiming(); }
    Bitrate bitrate() const { return bitrate_; }

private:
    static constexpr uint32_t FRAME_BITS = 16;
    static constexpr uint32_t DMA_LEN    = FRAME_BITS; // 加 1 个帧尾空闲位

    Bitrate bitrate_ = Bitrate::DSHOT600;
    const uint32_t tim_clk_hz_;
    uint32_t timer_period_;   // ARR 值（用于计算 PWM 低电平宽度），现在会自动计算
    
    // 浮数版本的关键参数
    double ns2tick_f_ = 0.0;        // 每 ns 多少 timer tick（浮点）
    double t0h_ns_ = 0.0, t1h_ns_ = 0.0;
    double t0h_tick_f_ = 0.0, t1h_tick_f_ = 0.0;
    
    // 整数版本（最终使用）
    uint16_t t0h_tick_ = 0, t1h_tick_ = 0;
    
    std::array<uint16_t, DMA_LEN> dma_buf_{};

    /* 官方 DShot CRC */
    static uint8_t crc(uint16_t v) {
        return (v ^ (v >> 4) ^ (v >> 8)) & 0x0F;
    }

    /* 填 DMA 缓冲区：MSB first，PWM 极性已翻转 */
    void fillDmaBuffer(uint16_t frame) {
        printf("frame = %d\r\n",frame);
        for (uint32_t i = 0; i < FRAME_BITS; ++i) {
            bool bit = (frame >> (15 - i)) & 1;
            uint16_t high_tick = bit ? t1h_tick_ : t0h_tick_;
            /* 输出低电平长度 = ARR - 高电平长度 */
            dma_buf_[i] = timer_period_ - high_tick;
            printf("%d ", dma_buf_[i]);
        }

        /* 帧尾空闲位：强制发 0（低电平） */
        //dma_buf_[FRAME_BITS] = timer_period_; // 高电平 0 tick
        //printf("%d ", dma_buf_[FRAME_BITS]);
                printf("\r\n");
    }

    std::function<bool()> dmaTransferCompleteCb_;
    std::function<void(uint16_t*, uint32_t)> dmaTransferStartCb_;
};

} // namespace mDev