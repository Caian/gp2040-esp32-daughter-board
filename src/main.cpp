/*
 * Copyright (C) 2020 Caian Benedicto <caianbene@gmail.com>
 *
 * This file is part of gp2040-esp32-daughter-board.
 *
 * gp2040-esp32-daughter-board is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2, or (at your
 * option) any later version.
 *
 * gp2040-esp32-daughter-board is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gp2040-esp32-daughter-board.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <atomic>
#include <vector>

void check_commands();
bool load_config();
void save_config();

#if defined(ESP32)
constexpr uint8_t i2c_scl = 17;
constexpr uint8_t i2c_sda = 16;
constexpr size_t num_gpio = 40;
constexpr size_t max_avg_win = 32;
constexpr size_t adc_default_avg = 10;
constexpr size_t adc_default_scale = 65536 / 4096;
#endif

enum gpio_config_enum : uint8_t
{
    gpio_config_off = 0,
    gpio_config_in = 1,
    gpio_config_in_up = 2,
    gpio_config_in_dn = 3,
    gpio_config_adc = 4,
};

struct calibration_struct
{
    int pre_offset;
    int scale;
    int post_offset;
};

struct global_config_struct
{
    uint8_t wire_addr;
    gpio_config_enum gpio_config[num_gpio];
    calibration_struct adc_calibration[num_gpio];
    size_t avg_win_sz;
};

global_config_struct config;
std::vector<uint8_t> dig_gpio;
std::vector<uint8_t> adc_gpio;

std::atomic<uint16_t> gpio_values[num_gpio];

uint8_t wire_buffer[32];
uint16_t avg_window[num_gpio][max_avg_win];

int add_to_average(
    int channel,
    int latest_adc_value
)
{
    auto& window = avg_window[channel];

    if (config.avg_win_sz == 0 || config.avg_win_sz == 1)
        return latest_adc_value;

    int avg = latest_adc_value;

    for (int i = 0; i < config.avg_win_sz - 1; i++)
    {
        auto curr = window[i + 1];
        window[i] = curr;
        avg += curr;
    }

    window[config.avg_win_sz - 1] = latest_adc_value;
    avg /= (int)config.avg_win_sz;

    return avg;
}

void gpio_reader(void*)
{
    Serial.println("GPIO reader task started.");

    while (true)
    {
        for (int i = 0; i < adc_gpio.size(); i++)
        {
            const auto adc_pin = adc_gpio[i];

            const auto& cal = config.adc_calibration[i];
            int adc_value = analogRead(adc_pin);
            adc_value = add_to_average(i, adc_value);
            adc_value += cal.pre_offset;
            adc_value *= cal.scale;
            adc_value += cal.post_offset;
            gpio_values[adc_pin].store((uint16_t)adc_value,
                std::memory_order_relaxed);
        }

        uint16_t curr_word = 0;

        for (int i = 0; i < dig_gpio.size(); i++)
        {
            const auto pin_bit = i % 16;
            const auto value = digitalRead(dig_gpio[i]);

            if (value == LOW)
                curr_word |= (uint16_t)(1 << pin_bit);

            if ((pin_bit == 15) || (i == dig_gpio.size() - 1))
            {
                // Use the earlier digital pins to store the state of the
                // entire set of bits
                const auto gpio = dig_gpio[i / 16];

                gpio_values[gpio].store(curr_word,
                    std::memory_order_relaxed);

                curr_word = 0;
            }
        }

        vTaskDelay(1);
    }
}

void receive_data(int count)
{
    //Serial.print("R");
    //Serial.print(count);
}

void send_data()
{
    // WARNING: Wire library internal buffer has only 32 bytes.

    memset(wire_buffer, 0, sizeof(wire_buffer));

    int j = 0;

    for (int i = 0; i < adc_gpio.size(); i++, j += 2)
    {
        if (j == sizeof(wire_buffer))
            break;

        const auto gpio = adc_gpio[i];
        const auto adc_value = gpio_values[gpio].load(
            std::memory_order_relaxed);

        wire_buffer[j] = adc_value & 0xFF;
        wire_buffer[j+1] = (adc_value >> 8) & 0xFF;
    }

    const int num_dig_words = (dig_gpio.size() == 0) ? 0 :
        ((dig_gpio.size() - 1) / 16 + 1);

    for (int i = 0; i < num_dig_words; i++, j += 2)
    {
        if (j == sizeof(wire_buffer))
            break;

        const auto gpio = dig_gpio[i];
        const auto value = gpio_values[gpio].load(
            std::memory_order_relaxed);

        wire_buffer[j] = value & 0xFF;
        wire_buffer[j+1] = (value >> 8) & 0xFF;
    }

    Wire.write(wire_buffer, sizeof(wire_buffer));
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Hello!");

    if (!load_config())
        save_config();

    for (uint8_t i = 0; i < num_gpio; i++)
    {
        switch (config.gpio_config[i])
        {
        case gpio_config_in:
            Serial.print("GPIO ");
            Serial.print(i);
            Serial.println(": Input");
            pinMode(i, INPUT);
            dig_gpio.push_back(i);
            break;
        case gpio_config_in_up:
            Serial.print("GPIO ");
            Serial.print(i);
            Serial.println(": Input Pull-Up");
            pinMode(i, INPUT_PULLUP);
            dig_gpio.push_back(i);
            break;
        case gpio_config_in_dn:
            Serial.print("GPIO ");
            Serial.print(i);
            Serial.println(": Input Pull-Down");
            pinMode(i, INPUT_PULLDOWN);
            dig_gpio.push_back(i);
            break;
        case gpio_config_adc:
            Serial.print("GPIO ");
            Serial.print(i);
            Serial.println(": ADC");
            pinMode(i, INPUT);
            adc_gpio.push_back(i);
            break;
        case gpio_config_off:
            break;
        default:
            break;
        }
    }

    Serial.print("Digital GPIOs: ");
    Serial.println(adc_gpio.size());
    Serial.print("Analog GPIOs: ");
    Serial.println(adc_gpio.size());

    Serial.print("ADC average samples: ");
    Serial.println(config.avg_win_sz);

    Serial.println("Calibration settings:");
    for (int i = 0; i < adc_gpio.size(); i++)
    {
        const auto gpio = adc_gpio[i];
        const auto& cal = config.adc_calibration[gpio];
        Serial.print(gpio);
        Serial.print(" - PREOFF: "); Serial.print(cal.pre_offset);
        Serial.print(", SCALE: "); Serial.print(cal.scale);
        Serial.print(", POSTOFF: "); Serial.println(cal.post_offset);
    }

    Serial.print("Initializing I2C addr=");
    Serial.print(config.wire_addr);
    Serial.print(",sda=");
    Serial.print(i2c_sda);
    Serial.print(",scl=");
    Serial.print(i2c_scl);
    Serial.println("....");

    while (!Wire.begin(config.wire_addr, i2c_sda, i2c_scl, 0U))
        delay(10);

    Wire.onReceive(receive_data);
    Wire.onRequest(send_data);

    Serial.println("Starting ADC task...");
    xTaskCreate(gpio_reader, "gpio_reader", 4000, NULL, 1, NULL);
}

void loop()
{
    check_commands();

    vTaskDelay(5);
}

bool try_read_int(int& value)
{
    value = Serial.parseInt();

    Serial.print(value);
    Serial.print('/');

    return true;
}

bool try_read_gpio_idx(int& gpio)
{
    gpio = Serial.parseInt();

    Serial.print(gpio);
    Serial.print('/');

    if (gpio < 0 || gpio >= num_gpio)
    {
        Serial.print("Invalid GPIO: ");
        Serial.println(gpio);

        return false;
    }

    return true;
}

bool try_read_mode(gpio_config_enum& mode)
{
    int imode = Serial.parseInt();

    Serial.print(imode);
    Serial.print('/');

    mode = (gpio_config_enum)imode;

    switch (imode)
    {
    case gpio_config_off:
    case gpio_config_in:
    case gpio_config_in_up:
    case gpio_config_in_dn:
    case gpio_config_adc:
        return true;
    default:
        Serial.print("Invalid GPIO mode: ");
        Serial.println(imode);
        return false;
    }
}

void process_c_command()
{
    int gpio = 0;

    // Calibration: gpio pre_offset scale post_offset
    if (!try_read_gpio_idx(gpio) ||
        !try_read_int(config.adc_calibration[gpio].pre_offset) ||
        !try_read_int(config.adc_calibration[gpio].scale) ||
        !try_read_int(config.adc_calibration[gpio].post_offset))
    {
        Serial.println();
        Serial.println("ADC [C]alibration: GPIO pre_off scale post_off");

        return;
    }

    const auto& cal = config.adc_calibration[gpio];

    Serial.println();
    Serial.print("ADC ");
    Serial.print(gpio);
    Serial.println(":");
    Serial.print(" PREOFF : ");
    Serial.println(cal.pre_offset);
    Serial.print(" SCALE  : ");
    Serial.println(cal.scale);
    Serial.print(" POSTOFF: ");
    Serial.println(cal.post_offset);
}

void process_m_command()
{
    int gpio = 0;
    gpio_config_enum mode = gpio_config_off;

    // Set GPIO mode
    if (!try_read_gpio_idx(gpio) ||
        !try_read_mode(mode))
    {
        Serial.println();
        Serial.println("Set GPIO [M]ode: GPIO mode");

        return;
    }

    config.gpio_config[gpio] = mode;

    Serial.println();
    Serial.print("GPIO ");
    Serial.print(gpio);
    Serial.print(" mode set to ");
    switch (mode)
    {
    case gpio_config_off:
        Serial.println("off");
        break;
    case gpio_config_in:
        Serial.println("digital");
        break;
    case gpio_config_in_up:
        Serial.println("digital with internal pull-up");
        break;
    case gpio_config_in_dn:
        Serial.println("digital with internal pull-down");
        break;
    case gpio_config_adc:
        Serial.println("analog");
        break;
    default:
        // Should not happen...
        Serial.println("?");
        break;
    }
}

void process_p_command()
{
    Serial.println();

    for (int i = 0; i < adc_gpio.size(); i++)
    {
        const auto gpio = adc_gpio[i];
        const auto adc_value = gpio_values[gpio].load(
            std::memory_order_relaxed);

        Serial.print("ADC ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(adc_value);
    }

    const int num_dig_words = (dig_gpio.size() == 0) ? 0 :
        ((dig_gpio.size() - 1) / 16 + 1);

    Serial.print("Digital: ");
    for (int i = 0; i < num_dig_words; i++)
    {
        const auto gpio = dig_gpio[i];
        const auto dig_value = gpio_values[gpio].load(
            std::memory_order_relaxed);

        Serial.print(dig_value, BIN);
    }
    Serial.println();

}

void process_v_command()
{
    int window_sz;

    if (!try_read_int(window_sz))
    {
        Serial.println();
        Serial.println("ADC a[V]erage samples: count");

        return;
    }

    if (window_sz < 0 || window_sz > max_avg_win)
    {
        Serial.println();
        Serial.print("Invalid average samples ");
        Serial.print(window_sz);
        Serial.print(", max ");
        Serial.println(max_avg_win);

        return;
    }

    config.avg_win_sz = window_sz;

    Serial.println();
    Serial.print("ADC average samples set to ");
    Serial.println(config.avg_win_sz);
}

void process_w_command()
{
    Serial.println();

    save_config();
}

void check_commands()
{
    if (Serial.available() == 0)
        return;

    const auto command = Serial.read();

    Serial.print((char)command);
    Serial.print('/');

    switch (command)
    {
    case 0  : Serial.println("Serial error!"); break;
    case 'c': case 'C': process_c_command(); break;
    case 'm': case 'M': process_m_command(); break;
    case 'p': case 'P': process_p_command(); break;
    case 'v': case 'V': process_v_command(); break;
    case 'w': case 'W': process_w_command(); break;
    case 'r': case 'R': ESP.restart();
    default:
        Serial.print("Unknown command ");
        Serial.println(command);
        break;
    }
}

bool load_config()
{
    const auto fs_open = SPIFFS.begin();
    bool config_loaded = false;

    if (!fs_open)
    {
        Serial.println("Failed to open flash storage!");
    }
    else
    {
        auto filename = "/config.bin";
        auto file = SPIFFS.open(filename, "rb");

        static const auto size = sizeof(config);

        if (!file)
        {
            Serial.println("Failed to open configuration file!");
        }
        else
        {
            const auto read = file.read((uint8_t*)&config, size);
            file.close();

            if (read != size)
            {
                Serial.println("Configuration mismatch!");
                Serial.println("Failed to read configuration file!");
            }
            else
            {
                Serial.println("Loaded configuration file!");

                config_loaded = true;
            }
        }

        SPIFFS.end();
    }

    if (!config_loaded)
    {
        config.wire_addr = DEFAULT_I2C_ADDR;
        config.avg_win_sz = adc_default_avg;

        for (auto& gpio: config.gpio_config)
            gpio = gpio_config_off;

        for (auto& cal: config.adc_calibration)
        {
            cal.scale = adc_default_scale;
            cal.pre_offset = 0;
            cal.post_offset = 0;
        }
    }

    return config_loaded;
}

void save_config()
{
    if (!SPIFFS.begin())
    {
        Serial.println("Failed to open flash storage!");

        return;
    }

    auto filename = "/config.bin";
    auto file = SPIFFS.open(filename, "wb");

    static const auto size = sizeof(config);

    if (!file)
    {
        Serial.println("Failed to open configuration file!");

        return;
    }

    const auto written = file.write((uint8_t*)&config, size);
    file.close();

    if (written != size)
    {
        Serial.println("Configuration mismatch!");
        Serial.println("Failed to write configuration file!");
    }
    else
    {
        Serial.println("Written configuration file!");
    }

    SPIFFS.end();
}
