#include "CowAlphaNum.h"

#include <frc/I2C.h>

namespace CowLib
{
    static const uint16_t m_Table[] = {
        0b0000000000000001, 0b0000000000000010, 0b0000000000000100, 0b0000000000001000, 0b0000000000010000, 0b0000000000100000, 0b0000000001000000,
        0b0000000010000000, 0b0000000100000000, 0b0000001000000000, 0b0000010000000000, 0b0000100000000000, 0b0001000000000000, 0b0010000000000000,
        0b0100000000000000, 0b1000000000000000, 0b0000000000000000, 0b0000000000000000, 0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
        0b0000000000000000, 0b0000000000000000, 0b0000000000000000, 0b0001001011001001, 0b0001010111000000, 0b0001001011111001, 0b0000000011100011,
        0b0000010100110000, 0b0001001011001000, 0b0011101000000000, 0b0001011100000000,
        0b0000000000000000, //
        0b0000000000000110, // !
        0b0000001000100000, // "
        0b0001001011001110, // #
        0b0001001011101101, // $
        0b0000110000100100, // %
        0b0010001101011101, // &
        0b0000010000000000, // '
        0b0010010000000000, // (
        0b0000100100000000, // )
        0b0011111111000000, // *
        0b0001001011000000, // +
        0b0000100000000000, // ,
        0b0000000011000000, // -
        0b0000000000000000, // .
        0b0000110000000000, // /
        0b0000110000111111, // 0
        0b0000000000000110, // 1
        0b0000000011011011, // 2
        0b0000000010001111, // 3
        0b0000000011100110, // 4
        0b0010000001101001, // 5
        0b0000000011111101, // 6
        0b0000000000000111, // 7
        0b0000000011111111, // 8
        0b0000000011101111, // 9
        0b0001001000000000, // :
        0b0000101000000000, // ;
        0b0010010000000000, // <
        0b0000000011001000, // =
        0b0000100100000000, // >
        0b0001000010000011, // ?
        0b0000001010111011, // @
        0b0000000011110111, // A
        0b0001001010001111, // B
        0b0000000000111001, // C
        0b0001001000001111, // D
        0b0000000011111001, // E
        0b0000000001110001, // F
        0b0000000010111101, // G
        0b0000000011110110, // H
        0b0001001000000000, // I
        0b0000000000011110, // J
        0b0010010001110000, // K
        0b0000000000111000, // L
        0b0000010100110110, // M
        0b0010000100110110, // N
        0b0000000000111111, // O
        0b0000000011110011, // P
        0b0010000000111111, // Q
        0b0010000011110011, // R
        0b0000000011101101, // S
        0b0001001000000001, // T
        0b0000000000111110, // U
        0b0000110000110000, // V
        0b0010100000110110, // W
        0b0010110100000000, // X
        0b0001010100000000, // Y
        0b0000110000001001, // Z
        0b0000000000111001, // [
        0b0010000100000000, //
        0b0000000000001111, // ]
        0b0000110000000011, // ^
        0b0000000000001000, // _
        0b0000000100000000, // `
        0b0001000001011000, // a
        0b0010000001111000, // b
        0b0000000011011000, // c
        0b0000100010001110, // d
        0b0000100001011000, // e
        0b0000000001110001, // f
        0b0000010010001110, // g
        0b0001000001110000, // h
        0b0001000000000000, // i
        0b0000000000001110, // j
        0b0011011000000000, // k
        0b0000000000110000, // l
        0b0001000011010100, // m
        0b0001000001010000, // n
        0b0000000011011100, // o
        0b0000000101110000, // p
        0b0000010010000110, // q
        0b0000000001010000, // r
        0b0010000010001000, // s
        0b0000000001111000, // t
        0b0000000000011100, // u
        0b0010000000000100, // v
        0b0010100000010100, // w
        0b0010100011000000, // x
        0b0010000000001100, // y
        0b0000100001001000, // z
        0b0000100101001001, // {
        0b0001001000000000, // |
        0b0010010010001001, // }
        0b0000010100100000, // ~
        0b0011111111111111,
    };

    CowAlphaNum::CowAlphaNum(uint8_t address)
        : m_Address(0x70),
          m_I2C(new frc::I2C(frc::I2C::kMXP, 0x70)),
          m_Banner(0),
          m_BannerLength(0),
          m_BannerPosition(0)
    {
        memset(m_DisplayBuffer, 0, sizeof(m_DisplayBuffer));

        OscillatorOn();
        BlinkRate(HT16K33_BLINK_OFF);
        SetBrightness(15);
    }

    CowAlphaNum::~CowAlphaNum()
    {
        if (!m_I2C)
        {
            delete m_I2C;
        }
    }

    void CowAlphaNum::BlinkRate(uint8_t b)
    {
        if (b > 3)
            b = 0;
        b = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1);
        m_I2C->WriteBulk(&b, sizeof(b));
    }

    void CowAlphaNum::OscillatorOn()
    {
        uint8_t oscillator = HT16K33_OSCILLATOR;
        m_I2C->WriteBulk(&oscillator, sizeof(oscillator));
    }

    void CowAlphaNum::SetBrightness(uint8_t b)
    {
        if (b > 15)
            b = 15;
        b = HT16K33_CMD_BRIGHTNESS | b;
        m_I2C->WriteBulk(&b, sizeof(b));
    }

    void CowAlphaNum::WriteAscii(uint32_t n, uint8_t c, bool d)
    {
        if (n > 3)
        {
            return;
        }

        uint8_t l = m_Table[c] & 0xff;
        uint8_t u = (m_Table[c] >> 8) & 0xff;

        m_DisplayBuffer[(n * 2) + 1] = l;
        m_DisplayBuffer[(n * 2) + 2] = u;

        if (d)
        {
            m_DisplayBuffer[(n * 2) + 2] |= (1 << 6);
        }
    }

    void CowAlphaNum::WriteRaw(uint32_t n, uint16_t d)
    {
        if (n > (sizeof(m_DisplayBuffer) - 1))
        {
            return;
        }

        m_DisplayBuffer[(n * 2) + 1] = d & 0xff;
        m_DisplayBuffer[(n * 2) + 2] = (d >> 8) & 0xff;
    }

    void CowAlphaNum::Clear()
    {
        memset(m_DisplayBuffer, 0, sizeof(m_DisplayBuffer));
    }

    void CowAlphaNum::Display()
    {
        m_I2C->WriteBulk(m_DisplayBuffer, sizeof(m_DisplayBuffer));
        if (m_DisplayBuffer[sizeof(m_DisplayBuffer) - 1] == 0x81)
        {
            m_DisplayBuffer[sizeof(m_DisplayBuffer) - 1] = 0x00;
        }
        else
        {
            m_DisplayBuffer[sizeof(m_DisplayBuffer) - 1] = 0x81;
        }
    }

    void CowAlphaNum::SetBanner(std::string msg)
    {
        uint32_t rawMsgCount = 0;
        uint32_t firstIndex  = 0;
        uint32_t i, j;

        for (i = 0; i < msg.length(); ++i)
        {
            if (msg[i] != '.')
            {
                firstIndex = i;
                break;
            }
        }

        for (; i < msg.length(); ++i)
        {
            if (msg[i] != '.')
            {
                rawMsgCount++;
            }
        }

        if (m_Banner)
        {
            delete[] m_Banner;
        }

        m_Banner       = new uint16_t[rawMsgCount];
        m_BannerLength = rawMsgCount;

        j = firstIndex;
        for (i = 0; i < rawMsgCount; ++i)
        {
            m_Banner[i] = m_Table[static_cast<uint8_t>(msg[j])];
            if (((j + 1) < msg.length()) && (msg[j + 1] == '.'))
            {
                m_Banner[i] |= (1 << 14);
                j++;
            }
            j++;
        }
    }

    void CowAlphaNum::SetBannerPosition(uint32_t pos)
    {
        m_BannerPosition = pos;
    }

    void CowAlphaNum::DisplayBanner()
    {
        uint8_t i;
        for (i = 0; i < 4; ++i)
        {
            WriteRaw(i, m_Banner[(m_BannerPosition + i) % m_BannerLength]);
        }

        Display();
    }
} // namespace CowLib
