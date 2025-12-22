
#pragma once
#include <cstdint>
#include <thread> // #include ""
#include <iostream>
#include <cmath>
#include <mutex>
namespace Hall
{

    class HallSensors
    {
    public:
        HallSensors();
        // 获取零点校正后的角度数组的const引用
        const int16_t *getValuesZero() const { return _values_zero; }
        
        uint16_t getValue(uint8_t num)
        {
            if (num < 16)
            {
                return _values_zero[num];
            }
            return 0;
        }
        float getFValue(uint8_t num)
        {
            if (num < 16)
            {
                // std::cout << _values_zero[num] << std::endl;

                return round((_values_zero[num] / 8192.0 * 180) * 100) / 100;
                ;
            }
            return 0;
        }
        // void init();
        // void start(uint8_t show = 0);
        void start(unsigned int show);
        void stop();
        void setZero();
        ~HallSensors();

    private:
        // std::atomic<int> sharedVar(0);
        // value mutex
        std::mutex _mutex;
        uint16_t _values[16] = {0};
        uint16_t _zero[16] = {0};
        int16_t _values_zero[16] = {0};
        // 读取线程
        std::thread _readThread;
        void read();
    };

} // namespace Hall