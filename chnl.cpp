#include "daisy_patch_sm.h"
#include "daisysp.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

// Pins
// #define TFT_RST_PIN hw.D2
// #define TFT_DC_PIN  hw.D3
// #define TFT_CS_PIN  hw.D1
GPIO rst;
GPIO dc;
GPIO cs;
GPIO bl;

SpiHandle spiHandle;

// ST7789 commands
#define ST7789_SWRESET 0x01
#define ST7789_SLPOUT  0x11
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C

// Colors
#define BLACK       0x0000
#define NAVY        0x000F
#define DARKGREEN   0x03E0
#define DARKCYAN    0x03EF
#define MAROON      0x7800
#define PURPLE      0x780F
#define OLIVE       0x7BE0
#define LIGHTGREY   0xC618
#define DARKGREY    0x7BEF
#define BLUE        0x001F
#define GREEN       0x07E0
#define CYAN        0x07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF

// Screen dimensions
#define TFT_WIDTH  240
#define TFT_HEIGHT 280

#define HORIZONTAL 0
#define VERTICAL   1

size_t DSY_SDRAM_BSS display_buffer[TFT_WIDTH * 5];

typedef struct{
    uint16_t WIDTH;
    uint16_t HEIGHT;
    uint8_t SCAN_DIR;
}LCD_1IN69_ATTRIBUTES;
LCD_1IN69_ATTRIBUTES LCD_1IN69;

// Set the buffer size and sample rate
static const size_t kBufferSize = 48000 * 60; // 60 seconds at 48kHz

// Create a DaisySeed object
DaisyPatchSM hw;

// Create a buffer for the audio data
float DSY_SDRAM_BSS buffer[kBufferSize][2]; // stereo audio

// Create a variable for the current position
size_t currentPosition = 0;

// Create a variable for the loop length
float metroFreq = 8.0f; // 16th notes at 120bpm
// Loop length in metro pulses
int loopLengthPulses = 16;
size_t loopLengthSamples;

void ProcessLoopLengthSamples() {
    loopLengthSamples = 48000 / (metroFreq / loopLengthPulses);
}

// Create enum to track looper state
enum LooperState {
    PendingClear,
    PendingRecord,
    Playing,
    Recording,
    Standby
};

LooperState state = Standby;

// Create an encoder
Encoder encoder;

// Create a metro for sequencing
Metro metro;

// Track metro pulses in loop cycle
int count = 1;

// Function to send a command to the ST7789
void ST7789V2SendCommand(uint8_t cmd) {
    dc.Write(0);
    cs.Write(0);
    spiHandle.BlockingTransmit(&cmd, 1);
    cs.Write(1);
}

void DmaEndCallback(void* context, SpiHandle::Result result) {
    if (result == SpiHandle::Result::OK) {
        hw.PrintLine("DMA OK");
    } else {
        hw.PrintLine("DMA failed");
    }
    cs.Write(1);
}

uint16_t DMA_MIN_SIZE = 16;

// Function to send 8-bit data to the ST7789
void ST7789V2SendData8Bit(uint8_t *data, size_t len) {
    dc.Write(1);
    cs.Write(0);
    // spiHandle.DmaTransmit(data, len, nullptr, DmaEndCallback, nullptr);
    while (len > 0) {
		uint16_t chunk_size = len > 65535 ? 65535 : len;
        if (DMA_MIN_SIZE <= len) {
            spiHandle.DmaTransmit(data, chunk_size, nullptr, DmaEndCallback, nullptr);
        } else {
            spiHandle.BlockingTransmit(data, chunk_size);
        }
		data += chunk_size;
		len -= chunk_size;
	}
}

// Function to send 16-bit data to the ST7789
void SendData16Bit(uint16_t value) {
    uint8_t data;
    data = value >> 8;
    ST7789V2SendData8Bit(&data, 1);
    data = value;
    ST7789V2SendData8Bit(&data, 1);
}

// Function to set the window area on the TFT display for subsequent commands
void SetWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend) {
    // Check the scan direction of the display
    if (LCD_1IN69.SCAN_DIR == VERTICAL) {
        // Set the X coordinates (Column Address Set command)
        ST7789V2SendCommand(0x2A);
        SendData16Bit(Xstart);
        SendData16Bit(Xend);
        // Set the Y coordinates (Page Address Set command)
        ST7789V2SendCommand(0x2B);
        SendData16Bit(Ystart + 20);
        SendData16Bit(Yend + 20 - 1);
    } else {
        // Set the X coordinates (Column Address Set command)
        ST7789V2SendCommand(0x2A);
        SendData16Bit(Xstart + 20);
        SendData16Bit(Xend + 20 - 1);
        // Set the Y coordinates (Page Address Set command)
        ST7789V2SendCommand(0x2B);
        SendData16Bit(Ystart);
        SendData16Bit(Yend - 1);
    }
    
    // Memory Write command: subsequent data bytes will be written into the memory area defined above
    ST7789V2SendCommand(0X2C);
}

// Empties looper buffer
void ResetBuffer() {
    // Init buffer by filling with zeros.
    // This prevents white noise playing by default.
    std::fill(&buffer[0][0], &buffer[kBufferSize][1], 0);
}

// Function to reset the ST7789V2 TFT display controller
static void ST7789V2Reset(void) {
    rst.Write(0); // Initiate a reset by setting the reset pin low
    hw.Delay(100); // Wait for 100 milliseconds
    rst.Write(1); // End the reset by setting the reset pin high
    hw.Delay(120); // Wait for 120 milliseconds for the controller to complete its internal reset process
}

// Function to send a command and associated data to the ST7789V2 display controller
static void SendCommandData(uint8_t command, std::initializer_list<uint8_t> data_list) {
    ST7789V2SendCommand(command); // Send the command
    for (uint8_t data : data_list) { // Loop over each data byte in the list
        ST7789V2SendData8Bit(&data, 1); // Send the data byte
    }
}

// Function to initialize the ST7789V2 TFT display controller
static void ST7789V2InitRegister(void) {
    SendCommandData(0x36, {0x00}); // Set Memory Access Control
    SendCommandData(0x3A, {0x05}); // Set Interface Pixel Format
    SendCommandData(0xB2, {0x0B, 0x0B, 0x00, 0x33, 0x35}); // Set Porch Setting
    SendCommandData(0xB7, {0x11}); // Set Gate Control
    SendCommandData(0xBB, {0x35}); // Set VCOM Setting
    SendCommandData(0xC0, {0x2C}); // Set LCM Control
    SendCommandData(0xC2, {0x01}); // Set VDV and VRH Command Enable
    SendCommandData(0xC3, {0x0D}); // Set VRH Set
    SendCommandData(0xC4, {0x20}); // Set VDV Set
    SendCommandData(0xC6, {0x13}); // Set Frame Rate Control in Normal Mode
    SendCommandData(0xD0, {0xA4, 0xA1}); // Set Power Control 1
    SendCommandData(0xD6, {0xA1}); // Set OTP Program
    SendCommandData(0xE0, {0xF0, 0x06, 0x0B, 0x0A, 0x09, 0x26, 0x29, 0x33, 0x41, 0x18, 0x16, 0x15, 0x29, 0x2D}); // Set Positive Voltage Gamma Control
    SendCommandData(0xE1, {0xF0, 0x04, 0x08, 0x08, 0x07, 0x03, 0x28, 0x32, 0x40, 0x3B, 0x19, 0x18, 0x2A, 0x2E}); // Set Negative Voltage Gamma Control
    SendCommandData(0xE4, {0x25, 0x00, 0x00}); // Set Gate Control
    ST7789V2SendCommand(0x21); // Set Display Inversion ON
    ST7789V2SendCommand(0x11); // Set Sleep Out
    hw.Delay(120); // Wait for 120 milliseconds
    ST7789V2SendCommand(0x29); // Set Display ON
}

static void ST7789V2SetAttributes(uint8_t Scan_dir) {
    // Get the screen scan direction
    LCD_1IN69.SCAN_DIR = Scan_dir;
    uint8_t MemoryAccessReg = Scan_dir == HORIZONTAL ? 0X70 : 0x00;
    // Set the read / write scan direction of the frame memory
    ST7789V2SendCommand(0x36); // MX, MY, RGB mode
    ST7789V2SendData8Bit(&MemoryAccessReg, 1); // 0x08 set RGB
}

void ST7789V2Init(uint8_t Scan_dir) {
    // hw reset
    ST7789V2Reset();
    // Set the resolution and scanning method of the screen
    ST7789V2SetAttributes(Scan_dir);
    // Set the initialization register
    ST7789V2InitRegister();
}

void SetPixel(uint16_t X, uint16_t Y, uint16_t Color) {
    SetWindows(X, Y, X, Y);
    SendData16Bit(Color);
}

const uint8_t fontData[] = {
    0b111, 0b101, 0b111, 0b101, 0b101, // A
    0b111, 0b101, 0b110, 0b101, 0b111, // B
    0b111, 0b100, 0b100, 0b100, 0b111, // C
    0b110, 0b101, 0b101, 0b101, 0b110, // D
    0b111, 0b100, 0b110, 0b100, 0b111, // E
    0b111, 0b100, 0b110, 0b100, 0b100, // F
    0b111, 0b100, 0b101, 0b101, 0b111, // G
    0b101, 0b101, 0b111, 0b101, 0b101, // H
    0b111, 0b010, 0b010, 0b010, 0b111, // I
    0b111, 0b001, 0b001, 0b101, 0b111, // J
    0b101, 0b110, 0b100, 0b110, 0b101, // K
    0b100, 0b100, 0b100, 0b100, 0b111, // L
    0b101, 0b111, 0b111, 0b101, 0b101, // M
    0b101, 0b111, 0b111, 0b111, 0b101, // N
    0b111, 0b101, 0b101, 0b101, 0b111, // O
    0b111, 0b101, 0b111, 0b100, 0b100, // P
    0b111, 0b101, 0b101, 0b111, 0b011, // Q
    0b111, 0b101, 0b111, 0b110, 0b101, // R
    0b111, 0b100, 0b111, 0b001, 0b111, // S
    0b111, 0b010, 0b010, 0b010, 0b010, // T
    0b101, 0b101, 0b101, 0b101, 0b111, // U
    0b101, 0b101, 0b101, 0b111, 0b010, // V
    0b101, 0b101, 0b111, 0b111, 0b101, // W
    0b101, 0b101, 0b010, 0b101, 0b101, // X
    0b101, 0b101, 0b010, 0b010, 0b010, // Y
    0b111, 0b001, 0b010, 0b100, 0b111,  // Z
    0b111, 0b101, 0b101, 0b101, 0b111, // 0
    0b010, 0b110, 0b010, 0b010, 0b111, // 1
    0b111, 0b001, 0b111, 0b100, 0b111, // 2
    0b111, 0b001, 0b111, 0b001, 0b111, // 3
    0b101, 0b101, 0b111, 0b001, 0b001, // 4
    0b111, 0b100, 0b111, 0b001, 0b111, // 5
    0b111, 0b100, 0b111, 0b101, 0b111, // 6
    0b111, 0b001, 0b001, 0b001, 0b001, // 7
    0b111, 0b101, 0b111, 0b101, 0b111, // 8
    0b111, 0b101, 0b111, 0b001, 0b111  // 9
};

void DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, int scaleX = 4, int scaleY = 4) {
    // Adjust for 5x3 pixel characters
    const int charWidth = 3;
    const int charHeight = 5;

    // Calculate the offset into the font data for the character
    int offset;
    if (c >= 'A' && c <= 'Z') {
        offset = (c - 'A') * charHeight;
    } else if (c >= '0' && c <= '9') {
        offset = ('Z' - 'A' + 1 + c - '0') * charHeight;
    } else {
        return; // Unsupported character
    }

     // Draw the character
    for (int row = 0; row < charHeight; row++) {
        // Get the row data for the character
        uint8_t rowData = fontData[offset + row];

        for (int col = 0; col < charWidth; col++) {
            // Get the pixel value from the row data
            bool pixel = (rowData >> (charWidth - 1 - col)) & 0x01;

            // Draw the pixel scaled up
            if (pixel) {
                for (int dx = 0; dx < scaleX; dx++) {
                    for (int dy = 0; dy < scaleY; dy++) {
                        SetPixel(x + col*scaleX + dx, y + row*scaleY + dy, color);
                    }
                }
            }
        }
    }
}

void DrawString(int x, int y, const char* str, int color, int scaleX = 4, int scaleY = 4) {
    // The width of a character in pixels
    int charWidth = scaleX;

    // Loop through each character in the string
    for (int i = 0; str[i] != '\0'; i++) {
        // Draw the character
        DrawChar(x + i * charWidth * scaleX, y, str[i], color, scaleX, scaleY);
    }
}

void ClearWindow(int startX = 0, int startY = 0, int endX = TFT_WIDTH, int endY = TFT_HEIGHT) {
    // Array to hold color data
    uint8_t data[2];

    // Set the window to the specified coordinates
    SetWindows(startX, startY, endX, endY);

    // Loop over every pixel in the specified window
    for (uint32_t i = 0; i < (endX - startX + 1) * (endY - startY + 1); i++) {
        // Split the color into high and low bytes
        data[0] = (BLACK >> 8)&0xff;
        data[1] = BLACK;
        // Send the color data to the screen
        ST7789V2SendData8Bit(data, 2);
    }
}

// The audio callback
void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    float in_level   = 0.5;
    float loop_level = 1;
    
    for (size_t i = 0; i < size; i++) {
        if (metro.Process()) {
            if (count < loopLengthPulses) {
                count++;
            } else {
                hw.PrintLine("reset");
                count = 1;
                switch (state) {
                    case LooperState::Recording:
                        state = LooperState::Playing;
                        break;
                    case LooperState::PendingRecord:
                        state = LooperState::Recording;
                        break;
                    case LooperState::PendingClear:
                        state = LooperState::Standby;
                        ResetBuffer();
                        break;
                    case LooperState::Playing:
                    case LooperState::Standby:
                        break;
                }
            }
        }

        // store the inputs * the input gain factor
        float in_l = IN_L[i] * in_level;
        float in_r = IN_R[i] * in_level;

        // If we're recording, store the incoming audio in the buffer
        if (state == LooperState::Recording) {
            buffer[currentPosition][0] = in_l;
            buffer[currentPosition][1] = in_r;
        }

        // Play back the audio from the current position in the buffer
        OUT_L[i] = buffer[currentPosition][0] * loop_level + in_l;
        OUT_R[i] = buffer[currentPosition][1] * loop_level + in_r;

        // Increment the current position, wrapping around to the start of the loop if necessary
        currentPosition = (currentPosition + 1) % loopLengthSamples;
    }
}

int main() {
    // Init hw
    hw.Init();
    hw.StartLog();

    // Init encoder
    hw.PrintLine("Initialising encoder");
    encoder.Init(hw.D4, hw.D5, hw.D6);

    // Reset the looper buffer
    hw.PrintLine("Resetting looper buffer");
    ResetBuffer();

    // Start the audio
    hw.PrintLine("Starting audio");
    hw.StartAudio(AudioCallback);

    // Init metro
    hw.PrintLine("Initialising metro");
    metro.Init(metroFreq, 48000); // 16th notes at 120bpm

    // Init GPIO
    rst.Init(hw.D2, GPIO::Mode::OUTPUT);
    dc.Init(hw.D3, GPIO::Mode::OUTPUT);
    cs.Init(hw.D1, GPIO::Mode::OUTPUT);
    bl.Init(hw.D8, GPIO::Mode::OUTPUT);

    // Init SPI
    SpiHandle::Config spiHandleConfig;
    spiHandleConfig.periph     = SpiHandle::Config::Peripheral::SPI_2;
    spiHandleConfig.mode       = SpiHandle::Config::Mode::MASTER;
    spiHandleConfig.direction  = SpiHandle::Config::Direction::ONE_LINE;
    spiHandleConfig.nss        = SpiHandle::Config::NSS::SOFT;
    spiHandleConfig.pin_config.miso = hw.D8;
    spiHandleConfig.pin_config.mosi = hw.D9;
    spiHandleConfig.pin_config.nss = hw.D1;
    spiHandleConfig.pin_config.sclk = hw.D10;
    spiHandleConfig.clock_phase = SpiHandle::Config::ClockPhase::ONE_EDGE;
    spiHandleConfig.clock_polarity = SpiHandle::Config::ClockPolarity::LOW;
    spiHandleConfig.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_2;
    hw.PrintLine("Initialising SPI");
    SpiHandle::Result result = spiHandle.Init(spiHandleConfig);
    hw.PrintLine(result == SpiHandle::Result::OK ? "SPI init OK" : "SPI init failed");

    // Init display
    bl.Write(1);
    ST7789V2Init(VERTICAL);

    // Draw string
    ClearWindow();
    DrawString(20, 20, std::to_string(loopLengthPulses).c_str(), WHITE);
    DrawString(20, 44, "BARS", WHITE);

    while (1) {
        hw.ProcessAllControls();
        // Update the encoder
        encoder.Debounce();

        // If the encoder was pressed, start or stop recording
        if (encoder.RisingEdge()) {
            switch(state) {
                case LooperState::Playing:
                    state = LooperState::PendingClear;
                    break;
                case LooperState::Standby:
                    state = LooperState::PendingRecord;
                    break;
                case LooperState::Recording:
                case LooperState::PendingRecord:
                case LooperState::PendingClear:
                    break;
            }
        }

        // If the encoder was turned, adjust the loop length
        switch (encoder.Increment()) {
            case 1:
                if (loopLengthPulses < 256) {
                    loopLengthPulses *= 2;
                    ProcessLoopLengthSamples();
                    ClearWindow(20, 20, 64, 40);
                    DrawString(20, 20, std::to_string(loopLengthPulses).c_str(), WHITE);
                }
                break;
            case -1:
                if (loopLengthPulses > 1) {
                    loopLengthPulses /= 2;
                    ProcessLoopLengthSamples();
                    ClearWindow(20, 20, 64, 40);
                    DrawString(20, 20, std::to_string(loopLengthPulses).c_str(), WHITE);
                }
                break;
        }
    }
}