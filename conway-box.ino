// Board: Waveshare ESP32-S3-Zero

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP_I2S.h>
#include <arduinoFFT.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_SDA_PIN 6
#define SCREEN_SCL_PIN 5
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define SCREEN_I2C_FREQ_HZ 800000
#define I2S_SCK_PIN 3
#define I2S_WS_PIN 4
#define I2S_SDOUT_PIN 2

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, SCREEN_I2C_FREQ_HZ, SCREEN_I2C_FREQ_HZ);

uint8_t grid[SCREEN_HEIGHT][SCREEN_WIDTH];
uint8_t newGrid[SCREEN_HEIGHT][SCREEN_WIDTH];
int generation = 0;
const int maxGridLifeSeconds = 60;
const int maxGridStaticSeconds = 5;

const int sampleRate = 44100;
float sampleMax = 16000.0f;  // amp is a little overpowered so clamp it to be safe
float rmsTarget = 8000.0f;
I2SClass i2s;
const int i2sBufLenSamples = 1024;
int16_t i2sBuf[i2sBufLenSamples];

const uint16_t fftLen = 1024;  // must be a power of 2
const uint16_t fftLenBy2 = fftLen / 2;
float vReal[fftLen];
float vImag[fftLen];
const int freqBinMin = 5;
const int freqBinMax = 20;
const int widthStride = 15;
const int heightStride = 15;
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, fftLen, sampleRate);

TaskHandle_t DisplayTask;
TaskHandle_t I2STransferTask;

/**
 * @brief Initializes the grid with random alive/dead values.
 *
 * This function populates the grid with random values, either 0 (dead)
 * or 1 (alive). This is used to initialize the grid at the start of
 * the Game of Life.
 */
void initGrid() {
  for (int i = 0; i < SCREEN_HEIGHT; i++) {
    for (int j = 0; j < SCREEN_WIDTH; j++) {
      grid[i][j] = random(0, 2);
    }
  }
}

/**
 * @brief Counts the number of alive neighbours for a given cell.
 *
 * This function takes two arguments, i and j, which represent the
 * coordinates of the cell in the grid. It then iterates over the
 * eight cells in the Moore neighbourhood of the cell, counting the
 * number of cells that are alive. Note that the cell itself is
 * excluded from the count.
 *
 * The function uses the modulo operator to wrap around the edges of
 * the grid, so the function works correctly even when the cell is
 * located at the edge of the grid.
 *
 * @param i The row of the cell in the grid.
 * @param j The column of the cell in the grid.
 * @return The number of alive neighbours of the cell.
 */
int countAliveNeighbours(int i, int j) {
  int count = 0;
  for (int k = -1; k <= 1; k++) {
    for (int l = -1; l <= 1; l++) {
      int x = (i + k + SCREEN_HEIGHT) % SCREEN_HEIGHT;
      int y = (j + l + SCREEN_WIDTH) % SCREEN_WIDTH;
      count += grid[x][y];
    }
  }
  count -= grid[i][j];
  return count;
}

/**
 * @brief Updates the grid according to the rules of the Game of Life.
 *
 * This function creates a new grid with the same dimensions as the
 * current grid, then iterates over each cell in the current grid.
 * For each cell, it counts the number of alive neighbours, and then
 * applies the rules of the Game of Life to determine whether the cell
 * should be alive or dead in the new grid. The new grid is then
 * copied back into the current grid.
 *
 * @return The number of changes to the grid
 */
int updateGrid() {
  for (int i = 0; i < SCREEN_HEIGHT; i++) {
    for (int j = 0; j < SCREEN_WIDTH; j++) {
      int count = countAliveNeighbours(i, j);
      newGrid[i][j] = (grid[i][j] == 1) ? (count == 2 || count == 3) : (count == 3);
    }
  }

  int changes = 0;
  for (int i = 0; i < SCREEN_HEIGHT; i++) {
    for (int j = 0; j < SCREEN_WIDTH; j++) {
      if (grid[i][j] != newGrid[i][j]) {
        changes++;
      }
      grid[i][j] = newGrid[i][j];
      display.drawPixel(j, i, grid[i][j] ? SSD1306_WHITE : SSD1306_BLACK);
    }
  }

  return changes;
}

/**
 * @brief fills vReal with audio based on the grid
 */
void computeAudio() {

  // Init the complex vector
  for (int i = 0; i < fftLen; i++) {
    vReal[i] = 0.0f;
    vImag[i] = 0.0f;
  }

  // Loop through and see how much each grid pixel has to contribute
  // Also only do the real part since messing with the phase sounds too unpleasant
  float freqScaleWidth = (1.0f / SCREEN_WIDTH) * (freqBinMax - freqBinMin) / 2.0f;
  float freqScaleHeight = (1.0f / SCREEN_HEIGHT) * (freqBinMax - freqBinMin) / 2.0f;
  for (int i = 0; i < SCREEN_HEIGHT; i += heightStride) {
    for (int j = 0; j < SCREEN_WIDTH; j += widthStride) {
      int freqBin = freqBinMin + i * freqScaleHeight + j * freqScaleWidth;

      // Make sure we don't generate something out of bounds
      if (freqBin > fftLenBy2) {
        freqBin = fftLenBy2;
      }
      vReal[freqBin] += grid[i][j] * 1.0f;
    }
  }

  // Create conjugate symmetry
  for (int i = 1; i < fftLenBy2; i++) {
    vReal[fftLen - i] = vReal[i];
    vImag[fftLen - i] = -1.0f * vImag[i];
  }

  FFT.compute(FFTDirection::Reverse);

  // Normalize it so there aren't huge jumps in volumes when the grid starts out
  float rms = 0.0f;
  for (int i = 0; i < fftLen; i++) {
    rms += vReal[i] * vReal[i];
  }
  if (rms > 0.0f) {
    rms = rms / fftLen;
    rms = sqrtf(rms);

    float gain = rmsTarget / rms;
    for (int i = 0; i < fftLen; i++) {
      vReal[i] *= gain;
    }
  }
}

/**
 * @brief Updates the grid and handles re-init if necessary
 */
void computeGrid() {
  static int lastChange = 0;
  static int staticCount = 0;
  static int lastChangeTimeMs = 0;
  static unsigned long lastGridInitMs = 0;

  int changes = updateGrid();
  if (lastChange != changes) {
    lastChangeTimeMs = millis();
  }
  lastChange = changes;

  // If we're oscillating, static, or hit the max life then re-init the grid
  if (((millis() - lastChangeTimeMs) > maxGridStaticSeconds * 1000) || ((millis() - lastGridInitMs) > maxGridLifeSeconds * 1000)) {
    initGrid();
    lastGridInitMs = millis();
    lastChangeTimeMs = lastGridInitMs;
  }
}

/**
 * @brief Fill up the I2S buffer with constrained samples
 */
void updateI2SBuf() {
  for (int i = 0; i < i2sBufLenSamples; i++) {
    i2sBuf[i] = constrain(vReal[i % (fftLen)], -sampleMax, sampleMax);
  }
}

/**
 * @brief Task for display and audio compute
 */
void DisplayTaskCode(void* pvParameters) {
  for (;;) {
    computeGrid();
    computeAudio();
    display.display();
    // Make the audio reflect what's on the screen now
    updateI2SBuf();
  }
}

/**
 * @brief Transfers the I2S buffer to the driver
 */
void I2STransferTaskCode(void* pvParameters) {
  for (;;) {
    // I2S write is blocking
    i2s.write((uint8_t*)i2sBuf, i2sBufLenSamples * sizeof(int16_t));
  }
}

/**
 * @brief Program setup
 */
void setup() {
  Serial.begin(9600);
  useRealRandomGenerator(true);

  Wire.begin(SCREEN_SDA_PIN, SCREEN_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    while (1)
      ;  // do nothing
  }

  i2s.setPins(I2S_SCK_PIN, I2S_WS_PIN, I2S_SDOUT_PIN);
  if (!i2s.begin(I2S_MODE_STD, sampleRate, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("Failed to initialize I2S!");
    while (1)
      ;  // do nothing
  }
  display.clearDisplay();
  initGrid();

  xTaskCreatePinnedToCore(DisplayTaskCode, "DisplayTask", 10000, NULL, 2, &DisplayTask, 0);
  xTaskCreatePinnedToCore(I2STransferTaskCode, "I2STransferTask", 10000, NULL, 3, &I2STransferTask, 0);
}

void loop() {
}
