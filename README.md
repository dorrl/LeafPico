# Leaf Pico

PlatformIO firmware for the sensor node.

- DS18B20: GP15
- Soil-moisture analog output: GP26
- BH1750/LCD I2C: SDA GP4, SCL GP5
- BLE: Pico W built-in Nordic UART service (`Leaf-Pico`)

This firmware requires a **Raspberry Pi Pico W**. It measures sensor values once per minute, writes the result to the LCD, and sends a newline-delimited JSON message through the built-in BLE Nordic UART service. When the BH1750 light level is 5 lx or below, the LCD display and backlight turn off. They turn on again at 10 lx or above, preventing flicker near the threshold. There is no watering or pump-control function in this firmware.
