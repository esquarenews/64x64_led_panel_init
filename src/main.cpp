#include <Arduino.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <ESP32-HUB75-VirtualMatrixPanel_T.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <esp32-hal-psram.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <ctime>
#include <string>

#define ENABLE_BLE 0

#if ENABLE_BLE
#include <NimBLEDevice.h>
#endif

namespace {

constexpr uint16_t kSinglePanelWidth = 64;
constexpr uint16_t kSinglePanelHeight = 64;
constexpr uint8_t kPanelColumns = 3;
constexpr uint8_t kPanelRows = 4;
constexpr uint8_t kChainLength = kPanelColumns * kPanelRows;
constexpr uint16_t kDisplayWidth = kSinglePanelWidth * kPanelColumns;
constexpr uint16_t kDisplayHeight = kSinglePanelHeight * kPanelRows;
constexpr size_t kFramePixelCount =
    static_cast<size_t>(kDisplayWidth) * static_cast<size_t>(kDisplayHeight);
constexpr size_t kFrameByteSize = kFramePixelCount * sizeof(uint16_t);

constexpr uint8_t kExpectedPixelFormat = 0;  // RGB565
constexpr bool kEnableSerialStreaming = true;
constexpr bool kEnableBle = ENABLE_BLE != 0;
#if ENABLE_BLE
constexpr char kBleDeviceName[] = "LRGPanel";
constexpr char kBleServiceUuid[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kBleRxCharUuid[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kBleTxCharUuid[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
#endif
MatrixPanel_I2S_DMA* matrix = nullptr;
VirtualMatrixPanel_T<CHAIN_TOP_RIGHT_DOWN_ZZ>* virtualMatrix = nullptr;

#if ENABLE_BLE
NimBLEServer* bleServer = nullptr;
NimBLECharacteristic* bleTxCharacteristic = nullptr;
NimBLEAdvertising* bleAdvertising = nullptr;
bool bleReadyPending = false;
bool bleSubscribed = false;
#endif

uint16_t* frameCurrent = nullptr;
uint16_t* frameNext = nullptr;
uint16_t* frameStatic = nullptr;
bool framePending = false;
bool firstFrameDisplayed = false;
uint32_t lastReadyEmitMs = 0;
uint32_t bootPatternStartMs = 0;
bool demoPatternToggle = false;
bool transitionActive = false;
uint32_t transitionStartMs = 0;
uint32_t lastRenderMs = 0;
bool transitionsEnabled = false;

constexpr uint32_t kTransitionDurationMs = 2000;
constexpr uint32_t kStaticHoldDurationMs = 600;
constexpr bool kAllowTransitions = true;
constexpr bool kUseStaticIntermediate = true;
constexpr bool kEnableOnboardOverlay = false;

enum class TransitionPhase {
  None,
  ToStatic,
  StaticHold,
  ToImage,
};

TransitionPhase transitionPhase = TransitionPhase::None;
const uint16_t* transitionSource = nullptr;
const uint16_t* transitionTarget = nullptr;
uint32_t staticHoldStartMs = 0;

#pragma pack(push, 1)
struct FrameHeader {
  uint16_t width;
  uint16_t height;
  uint8_t format;
  uint8_t reserved;
  uint32_t dataLength;
};
#pragma pack(pop)

enum class SerialState {
  WaitingForMagic,
  ReadingHeader,
  ReadingPayload,
};


static_assert(sizeof(FrameHeader) == 10, "FrameHeader must be 10 bytes");

SerialState serialState = SerialState::WaitingForMagic;
uint8_t magicIndex = 0;
FrameHeader incomingHeader{};
size_t headerBytesRead = 0;
uint8_t headerBuffer[sizeof(FrameHeader)]{};
size_t frameBytesRead = 0;
uint32_t payloadStartMs = 0;
uint8_t* payloadWritePtr = nullptr;

void processIncomingByte(uint8_t byte);
void emitMessage(const char* msg);
bool ensureFrameBuffers();

constexpr char kMagicSequence[] = {'F', 'R', 'A', 'M'};

uint16_t* allocateDisplayBuffer(const char* label) {
  const size_t requestBytes = kFrameByteSize;
  uint16_t* ptr = nullptr;

  ptr = static_cast<uint16_t*>(
      heap_caps_malloc(requestBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (ptr) {
    std::memset(ptr, 0, requestBytes);
    return ptr;
  }

  ptr = static_cast<uint16_t*>(
      heap_caps_malloc(requestBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (!ptr) {
    ptr = static_cast<uint16_t*>(ps_malloc(requestBytes));
  }
  if (!ptr) {
    ptr = static_cast<uint16_t*>(malloc(requestBytes));
  }

  if (ptr) {
    std::memset(ptr, 0, requestBytes);
  } else {
    char msg[160];
    const size_t internalFree =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t spiramFree = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    snprintf(msg, sizeof(msg),
             "Alloc %s failed (%u bytes). largest internal=%u spiram=%u",
             label ? label : "buffer",
             static_cast<unsigned>(requestBytes),
             static_cast<unsigned>(internalFree),
             static_cast<unsigned>(spiramFree));
    emitMessage(msg);
  }
  return ptr;
}

void emitMessage(const char* msg) {
  if (!msg) {
    return;
  }
  Serial.println(msg);
#if ENABLE_BLE
  if (bleTxCharacteristic && bleServer &&
      bleServer->getConnectedCount() > 0 && bleSubscribed) {
    std::string data(msg);
    data.push_back('\n');
    bleTxCharacteristic->setValue(data);
    bleTxCharacteristic->notify();
  }
#endif
}

struct Rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

inline Rgb rgb565ToRgb(uint16_t color) {
  return {
      static_cast<uint8_t>((((color >> 11) & 0x1F) * 527u + 23u) >> 6),
      static_cast<uint8_t>((((color >> 5) & 0x3F) * 259u + 33u) >> 6),
      static_cast<uint8_t>(((color & 0x1F) * 527u + 23u) >> 6),
  };
}

inline uint16_t rgbTo565(uint8_t r, uint8_t g, uint8_t b) {
  const uint16_t r5 =
      static_cast<uint16_t>((static_cast<uint32_t>(r) * 31u + 127u) / 255u);
  const uint16_t g6 =
      static_cast<uint16_t>((static_cast<uint32_t>(g) * 63u + 127u) / 255u);
  const uint16_t b5 =
      static_cast<uint16_t>((static_cast<uint32_t>(b) * 31u + 127u) / 255u);
  return static_cast<uint16_t>((r5 << 11) | (g6 << 5) | b5);
}

inline uint16_t remapPanelColor(uint16_t color) {
  const Rgb rgb = rgb565ToRgb(color);
  // Panel wiring expects (R_out,G_out,B_out) fed by (G_in,B_in,R_in)
  return rgbTo565(rgb.b, rgb.r, rgb.g);
}

bool formatChristmasMessage(char* out, size_t outSize) {
  if (!out || outSize == 0) {
    return false;
  }
  auto currentTime = []() -> time_t {
    time_t now = time(nullptr);
    if (now > 0) {
      tm nowTm{};
      if (localtime_r(&now, &nowTm) && nowTm.tm_year + 1900 >= 2020) {
        return now;
      }
    }
    // Fallback to build time if RTC/NTP is not set (common on fresh boot).
    tm buildTm{};
    static const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char monthStr[4]{};
    int day = 1, year = 2020, hour = 0, min = 0, sec = 0;
    if (std::sscanf(__DATE__, "%3s %d %d", monthStr, &day, &year) == 3 &&
        std::sscanf(__TIME__, "%d:%d:%d", &hour, &min, &sec) == 3) {
      const char* pos = std::strstr(months, monthStr);
      int month = pos ? static_cast<int>((pos - months) / 3) : 0;
      buildTm.tm_year = year - 1900;
      buildTm.tm_mon = month;
      buildTm.tm_mday = day;
      buildTm.tm_hour = hour;
      buildTm.tm_min = min;
      buildTm.tm_sec = sec;
      return mktime(&buildTm);
    }
    return now;  // whatever we had, even if invalid
  };

  time_t now = currentTime();
  tm nowTm{};
  if (now <= 0 || !localtime_r(&now, &nowTm)) {
    return false;
  }

  // If it's Christmas Day locally, show the greeting.
  if (nowTm.tm_mon == 11 && nowTm.tm_mday == 25) {  // tm_mon is 0-based
    snprintf(out, outSize, "Merry Christmas");
    return true;
  }

  // Target is Dec 25 of the current year; roll to next year if already passed.
  tm targetTm{};
  targetTm.tm_year = nowTm.tm_year;
  targetTm.tm_mon = 11;   // December
  targetTm.tm_mday = 25;  // 25th
  targetTm.tm_hour = 0;
  targetTm.tm_min = 0;
  targetTm.tm_sec = 0;
  time_t target = mktime(&targetTm);
  if (target <= now) {
    targetTm.tm_year += 1;
    target = mktime(&targetTm);
  }

  const double secondsDiff = difftime(target, now);
  // Inclusive countdown: add almost a full day so we don't drop early.
  const double adjusted = std::max(0.0, secondsDiff + 86399.0);
  const unsigned days =
      static_cast<unsigned>(std::floor(adjusted / 86400.0));
  snprintf(out, outSize, "%u days until Christmas", days);
  return true;
}

void drawCountdownOverlay() {
  if (!kEnableOnboardOverlay) {
    return;
  }
  if (!virtualMatrix || !firstFrameDisplayed) {
    return;
  }
  char msg[48];
  if (!formatChristmasMessage(msg, sizeof(msg))) {
    return;
  }
  const size_t len = std::strlen(msg);
  const uint8_t charWidth = 6;   // default font width incl. spacing
  const uint8_t charHeight = 8;  // default font height
  int16_t x = static_cast<int16_t>(kDisplayWidth) -
              static_cast<int16_t>(len * charWidth) - 1;
  if (x < 0) {
    x = 0;
  }
  int16_t y = static_cast<int16_t>(kDisplayHeight) - charHeight;
  virtualMatrix->setTextWrap(false);
  virtualMatrix->setTextSize(1);
  virtualMatrix->setTextColor(remapPanelColor(rgbTo565(255, 255, 255)));
  virtualMatrix->setCursor(x, y);
  virtualMatrix->print(msg);
}

void generateBootTestPattern(uint16_t* buffer) {
  if (!buffer) {
    return;
  }
  // Simple grayscale gradient to validate wiring quickly.
  for (uint16_t y = 0; y < kDisplayHeight; ++y) {
    for (uint16_t x = 0; x < kDisplayWidth; ++x) {
      // Diagonal gradient: darker top-left, brighter bottom-right.
      const uint16_t sum = static_cast<uint16_t>(x + y);
      const uint16_t maxSum = static_cast<uint16_t>(kDisplayWidth + kDisplayHeight);
      const uint8_t level =
          static_cast<uint8_t>((static_cast<uint32_t>(sum) * 255u) / maxSum);
      buffer[y * kDisplayWidth + x] = rgbTo565(level, level, level);
    }
  }
}

void generateDemoPattern(uint16_t* buffer, bool invert) {
  if (!buffer) {
    return;
  }
  // Checkerboard to confirm dynamic updates before first frame arrives.
  const uint16_t on = rgbTo565(invert ? 32 : 192, invert ? 32 : 192, invert ? 32 : 192);
  const uint16_t off = rgbTo565(invert ? 192 : 32, invert ? 192 : 32, invert ? 192 : 32);
  for (uint16_t y = 0; y < kDisplayHeight; ++y) {
    for (uint16_t x = 0; x < kDisplayWidth; ++x) {
      const bool cell = ((x / 16u) + (y / 16u)) & 1u;
      buffer[y * kDisplayWidth + x] = cell ? on : off;
    }
  }
}

void generateStaticFrame(uint16_t* buffer) {
  if (!buffer) {
    return;
  }
  size_t index = 0;
  while (index < kFramePixelCount) {
    const uint32_t randomValue = esp_random();
    buffer[index++] = static_cast<uint16_t>(randomValue & 0xFFFFu);
    if (index < kFramePixelCount) {
      buffer[index++] = static_cast<uint16_t>((randomValue >> 16) & 0xFFFFu);
    }
  }
}

bool ensureFrameBuffers() {
  if (!frameCurrent) {
    frameCurrent = allocateDisplayBuffer("frameCurrent");
  }
  if (!frameNext) {
    frameNext = allocateDisplayBuffer("frameNext");
  }

  if (!frameCurrent) {
    return false;
  }
  if (!frameNext || !kAllowTransitions) {
    transitionsEnabled = false;
    frameNext = frameCurrent;
    emitMessage("Transitions disabled (single buffer).");
  } else {
    transitionsEnabled = true;
    emitMessage("Transitions enabled (double buffer).");
    if (kUseStaticIntermediate && !frameStatic) {
      frameStatic = allocateDisplayBuffer("frameStatic");
      if (!frameStatic) {
        emitMessage("Static buffer allocation failed. Falling back to direct transitions.");
      }
    }
  }

  payloadWritePtr = reinterpret_cast<uint8_t*>(frameNext);
  transitionPhase = TransitionPhase::None;
  transitionSource = nullptr;
  transitionTarget = nullptr;
  staticHoldStartMs = 0;
  return true;
}

#if ENABLE_BLE
class BleRxCallbacks : public NimBLECharacteristicCallbacks {
 public:
  void onWrite(NimBLECharacteristic* characteristic,
               NimBLEConnInfo& /*connInfo*/) override {
    const std::string& value = characteristic->getValue();
    for (uint8_t byte : value) {
      processIncomingByte(byte);
    }
  }
};

class BleTxCallbacks : public NimBLECharacteristicCallbacks {
 public:
  void onSubscribe(NimBLECharacteristic* /*characteristic*/,
                   NimBLEConnInfo& /*connInfo*/,
                   uint16_t subValue) override {
    bleSubscribed = subValue != 0;
    if (bleSubscribed) {
      bleReadyPending = true;
    }
  }
};

class BleServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer* /*server*/, NimBLEConnInfo& /*connInfo*/)
      override {
    emitMessage("BLE CONNECTED");
    bleSubscribed = false;
    bleReadyPending = false;
  }

  void onDisconnect(NimBLEServer* /*server*/, NimBLEConnInfo& /*connInfo*/,
                    int /*reason*/) override {
    emitMessage("BLE DISCONNECTED");
    if (bleAdvertising) {
      bleAdvertising->start();
    }
    bleSubscribed = false;
    bleReadyPending = false;
  }
};

BleRxCallbacks bleRxCallbacks;
BleTxCallbacks bleTxCallbacks;
BleServerCallbacks bleServerCallbacks;

void initBle() {
  NimBLEDevice::init(kBleDeviceName);
  // Keep BLE init minimal to avoid stalls on some boards.
  NimBLEDevice::setPower(ESP_PWR_LVL_P6);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(&bleServerCallbacks);

  NimBLEService* service = bleServer->createService(kBleServiceUuid);

  bleTxCharacteristic =
      service->createCharacteristic(kBleTxCharUuid, NIMBLE_PROPERTY::NOTIFY);
  bleTxCharacteristic->setCallbacks(&bleTxCallbacks);

  NimBLECharacteristic* rxCharacteristic = service->createCharacteristic(
      kBleRxCharUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rxCharacteristic->setCallbacks(&bleRxCallbacks);

  service->start();

  bleAdvertising = NimBLEDevice::getAdvertising();
  bleAdvertising->addServiceUUID(kBleServiceUuid);
  bleAdvertising->setConnectableMode(BLE_GAP_CONN_MODE_UND);
  bleAdvertising->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
  bleAdvertising->enableScanResponse(true);
  bleAdvertising->setName(kBleDeviceName);
  NimBLEDevice::setDeviceName(kBleDeviceName);
  bleAdvertising->start();

  emitMessage("BLE ADVERTISING");
}
#endif

#if ENABLE_BLE
void serviceBleReady() {
  if (!bleReadyPending) {
    return;
  }
  if (!bleTxCharacteristic || !bleServer) {
    return;
  }
  if (bleServer->getConnectedCount() == 0) {
    return;
  }
  if (!bleSubscribed) {
    return;
  }
  emitMessage("READY");
  bleReadyPending = false;
}
#endif

void resetReceiver() {
  serialState = SerialState::WaitingForMagic;
  magicIndex = 0;
  headerBytesRead = 0;
  frameBytesRead = 0;
  payloadStartMs = 0;
  payloadWritePtr = (transitionsEnabled && frameNext)
                        ? reinterpret_cast<uint8_t*>(frameNext)
                        : (frameCurrent ? reinterpret_cast<uint8_t*>(frameCurrent)
                                        : nullptr);
}

void processIncomingByte(uint8_t byte) {
  switch (serialState) {
    case SerialState::WaitingForMagic:
      if (byte == kMagicSequence[magicIndex]) {
        ++magicIndex;
        if (magicIndex == sizeof(kMagicSequence)) {
          serialState = SerialState::ReadingHeader;
        }
      } else {
        magicIndex = (byte == kMagicSequence[0]) ? 1 : 0;
      }
      break;

    case SerialState::ReadingHeader:
      headerBuffer[headerBytesRead++] = byte;
      if (headerBytesRead == sizeof(FrameHeader)) {
        std::memcpy(&incomingHeader, headerBuffer, sizeof(FrameHeader));

        const uint32_t expectedLength =
            static_cast<uint32_t>(kFramePixelCount * sizeof(uint16_t));

        // Emit length info over available transports.
#if ENABLE_BLE
        if (bleSubscribed || Serial) {
#else
        if (Serial) {
#endif
          char info[48];
          snprintf(info, sizeof(info), "LEN %u", incomingHeader.dataLength);
          emitMessage(info);
        }

        if (incomingHeader.format != kExpectedPixelFormat) {
          emitMessage("ERR format");
          resetReceiver();
          break;
        }

        if (incomingHeader.width != kDisplayWidth ||
            incomingHeader.height != kDisplayHeight) {
          emitMessage("ERR size");
          resetReceiver();
          break;
        }

        if (incomingHeader.dataLength != expectedLength) {
          emitMessage("ERR length");
          resetReceiver();
          break;
        }

        frameBytesRead = 0;
        payloadWritePtr = reinterpret_cast<uint8_t*>(frameNext);
        payloadStartMs = millis();
        serialState = SerialState::ReadingPayload;
      }
      break;

    case SerialState::ReadingPayload:
      if (!payloadWritePtr) {
        return;
      }
      payloadWritePtr[frameBytesRead++] = byte;
      if ((frameBytesRead % 8192u) == 0) {
        char buf[32];
        snprintf(buf, sizeof(buf), "RX %u/%u",
                 static_cast<unsigned>(frameBytesRead),
                 static_cast<unsigned>(incomingHeader.dataLength));
        emitMessage(buf);
        delay(2);  // brief pause to let UART buffer drain
      }
      if (frameBytesRead == incomingHeader.dataLength) {
        framePending = true;
        emitMessage("OK");
        resetReceiver();
      }
      break;
  }
}

void handleSerial() {
  while (Serial.available()) {
    const uint8_t byte = static_cast<uint8_t>(Serial.read());

    processIncomingByte(byte);
  }
}

void renderFrame(const uint16_t* frame) {
  if (!virtualMatrix || !frame) {
    return;
  }

  size_t index = 0;
  for (uint16_t y = 0; y < kDisplayHeight; ++y) {
    for (uint16_t x = 0; x < kDisplayWidth; ++x, ++index) {
      virtualMatrix->drawPixel(x, y, remapPanelColor(frame[index]));
    }
  }
}

void renderFrameWithOverlay(const uint16_t* frame, bool addOverlay) {
  renderFrame(frame);
  if (addOverlay) {
    drawCountdownOverlay();
  }
}

void renderTransition() {
  if (!virtualMatrix || !transitionSource || !transitionTarget) {
    transitionActive = false;
    transitionPhase = TransitionPhase::None;
    return;
  }

  const uint32_t now = millis();
  const float progress = std::min(
      1.0f,
      static_cast<float>(now - transitionStartMs) /
          static_cast<float>(kTransitionDurationMs));

  size_t index = 0;
  for (uint16_t y = 0; y < kDisplayHeight; ++y) {
    for (uint16_t x = 0; x < kDisplayWidth; ++x, ++index) {
      const Rgb fromColor = rgb565ToRgb(transitionSource[index]);
      const Rgb toColor = rgb565ToRgb(transitionTarget[index]);
      const auto lerp = [progress](uint8_t from, uint8_t to) -> uint8_t {
        const float blended =
            static_cast<float>(from) +
            (static_cast<float>(to) - static_cast<float>(from)) * progress;
        const int value = static_cast<int>(std::lround(blended));
        return static_cast<uint8_t>(std::clamp(value, 0, 255));
      };
      const uint8_t r = lerp(fromColor.r, toColor.r);
      const uint8_t g = lerp(fromColor.g, toColor.g);
      const uint8_t b = lerp(fromColor.b, toColor.b);
      virtualMatrix->drawPixel(x, y, remapPanelColor(rgbTo565(r, g, b)));
    }
  }

  if (progress >= 1.0f) {
    switch (transitionPhase) {
      case TransitionPhase::ToStatic:
        if (frameStatic) {
          std::memcpy(frameCurrent, frameStatic, kFrameByteSize);
          renderFrame(frameCurrent);
          transitionPhase = TransitionPhase::StaticHold;
          transitionSource = nullptr;
          transitionTarget = nullptr;
          transitionActive = false;
          staticHoldStartMs = now;
          emitMessage("STATIC");
          return;
        }
        [[fallthrough]];
      case TransitionPhase::StaticHold:
      case TransitionPhase::ToImage:
        if (frameCurrent != frameNext) {
          std::memcpy(frameCurrent, frameNext, kFrameByteSize);
        }
        renderFrameWithOverlay(frameCurrent, true);
        transitionActive = false;
        transitionPhase = TransitionPhase::None;
        transitionSource = nullptr;
        transitionTarget = nullptr;
        emitMessage("DONE");
        break;
      case TransitionPhase::None:
        transitionActive = false;
        transitionSource = nullptr;
        transitionTarget = nullptr;
        emitMessage("DONE");
        break;
    }
  }
}

void updateDisplay() {
  const uint32_t now = millis();
  if (now - lastRenderMs < 30) {
    return;
  }
  lastRenderMs = now;

  if (transitionsEnabled && transitionActive) {
    renderTransition();
    return;
  }

  if (transitionPhase == TransitionPhase::StaticHold) {
    if (!(kUseStaticIntermediate && frameStatic)) {
      transitionPhase = TransitionPhase::ToImage;
    } else if (now - staticHoldStartMs < kStaticHoldDurationMs) {
      return;
    } else {
      transitionPhase = TransitionPhase::ToImage;
    }
    transitionSource = frameCurrent;
    transitionTarget = frameNext;
    transitionActive = true;
    transitionStartMs = now;
    renderTransition();
    return;
  }

  if (!framePending) {
    return;
  }

  framePending = false;

  if (!firstFrameDisplayed) {
    std::memcpy(frameCurrent, frameNext, kFrameByteSize);
    firstFrameDisplayed = true;
    renderFrameWithOverlay(frameCurrent, true);
    emitMessage("SHOW");
  } else if (transitionsEnabled) {
    if (kUseStaticIntermediate && frameStatic) {
      generateStaticFrame(frameStatic);
      transitionPhase = TransitionPhase::ToStatic;
      transitionSource = frameCurrent;
      transitionTarget = frameStatic;
    } else {
      transitionPhase = TransitionPhase::ToImage;
      transitionSource = frameCurrent;
      transitionTarget = frameNext;
    }
    transitionActive = true;
    transitionStartMs = now;
    renderTransition();
  } else {
    if (frameCurrent != frameNext) {
      std::memcpy(frameCurrent, frameNext, kFrameByteSize);
    }
    renderFrameWithOverlay(frameCurrent, true);
    emitMessage("DONE");
  }
}


}  // namespace

void setup() {

  Serial.begin(115200);
  delay(200);
  Serial.println();
  emitMessage("Starting HUB75 twelve-panel display...");


  if (!psramInit()) {
    emitMessage("PSRAM init failed; external RAM not available.");
  } else if (!psramFound()) {
    emitMessage("PSRAM not detected after init.");
  } else {
    const size_t psramSize = ESP.getPsramSize();
    char psramInfo[64];
    snprintf(psramInfo, sizeof(psramInfo), "PSRAM ready (%u bytes)",
             static_cast<unsigned>(psramSize));
    emitMessage(psramInfo);
  }

  const size_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  const size_t spiramFree = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  char ramInfo[96];
  snprintf(ramInfo, sizeof(ramInfo), "RAM internal=%u PSRAM=%u",
           static_cast<unsigned>(internalFree),
           static_cast<unsigned>(spiramFree));
  emitMessage(ramInfo);

  emitMessage("Matrix init starting...");
  HUB75_I2S_CFG mxconfig(kSinglePanelWidth, kSinglePanelHeight, kChainLength);
  mxconfig.gpio.r1 = 9;
  mxconfig.gpio.g1 = 10;
  mxconfig.gpio.b1 = 11;
  mxconfig.gpio.r2 = 12;
  mxconfig.gpio.g2 = 13;
  mxconfig.gpio.b2 = 14;
  mxconfig.gpio.a = 4;
  mxconfig.gpio.b = 5;
  mxconfig.gpio.c = 6;
  mxconfig.gpio.d = 7;
  mxconfig.gpio.e = 8;
  mxconfig.gpio.clk = 16;
  mxconfig.gpio.lat = 17;
  mxconfig.gpio.oe = 18;

  mxconfig.double_buff = false;
  mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_10M;
  mxconfig.clkphase = false;
  mxconfig.driver = HUB75_I2S_CFG::ICN2038S;
  mxconfig.setPixelColorDepthBits(6);
  mxconfig.latch_blanking = 2;
  mxconfig.min_refresh_rate = 200;

  matrix = new MatrixPanel_I2S_DMA(mxconfig);
  if (!matrix || !matrix->begin(mxconfig)) {
    emitMessage(
        "MatrixPanel_I2S_DMA.begin() failed - check wiring and panel power.");
    while (true) {
      delay(1000);
    }
  }

  emitMessage("Matrix init complete.");

  matrix->setBrightness8(160);  // limit overall brightness to reduce load
  matrix->clearScreen();

  emitMessage("Virtual matrix starting...");
  virtualMatrix = new VirtualMatrixPanel_T<CHAIN_TOP_RIGHT_DOWN_ZZ>(
      kPanelRows, kPanelColumns, kSinglePanelWidth, kSinglePanelHeight);
  if (!virtualMatrix) {
    emitMessage("Virtual matrix allocation failed.");
    while (true) {
      delay(1000);
    }
  }

  virtualMatrix->setDisplay(*matrix);
  virtualMatrix->clearScreen();

  emitMessage("Virtual matrix ready.");

#if ENABLE_BLE
  emitMessage("BLE init starting...");
  initBle();
  emitMessage("BLE init complete.");
#endif

  emitMessage("Allocating frame buffers...");
  if (!ensureFrameBuffers()) {
    emitMessage("Frame buffer allocation failed.");
    while (true) {
      delay(1000);
    }
  }
  emitMessage("Frame buffers ready.");
  generateBootTestPattern(frameCurrent);
  renderFrame(frameCurrent);
  emitMessage("BOOT PATTERN SHOWN");
  firstFrameDisplayed = false;
  bootPatternStartMs = millis();
  lastReadyEmitMs = millis();
  resetReceiver();

  if (!kEnableSerialStreaming) {
    emitMessage("STATIC");
    return;
  }

  emitMessage("READY");
}

void loop() {
  // If a payload stalls, reset to avoid getting stuck silently.
  if (serialState == SerialState::ReadingPayload && payloadStartMs != 0) {
    const uint32_t now = millis();
    if (now - payloadStartMs > 60000) {
      char buf[64];
      snprintf(buf, sizeof(buf), "ERR payload timeout %u/%u",
               static_cast<unsigned>(frameBytesRead),
               static_cast<unsigned>(incomingHeader.dataLength));
      emitMessage(buf);
      resetReceiver();
      payloadStartMs = 0;
    }
  }

  if (kEnableSerialStreaming) {
    handleSerial();
    updateDisplay();
  } else {
    delay(50);
  }

  // Periodically re-emit READY until the first frame is displayed to help
  // external senders that connect after boot.
  if (!firstFrameDisplayed && serialState != SerialState::ReadingPayload) {
    const uint32_t now = millis();
    if (now - lastReadyEmitMs >= 2000) {
      emitMessage("READY");
      lastReadyEmitMs = now;
    }
  }

  // Animate a simple demo pattern until the first frame arrives so we know
  // the panel is updating.
  if (!firstFrameDisplayed && !framePending && serialState == SerialState::WaitingForMagic) {
    const uint32_t now = millis();
    if (now - bootPatternStartMs >= 1000) {
      demoPatternToggle = !demoPatternToggle;
      generateDemoPattern(frameCurrent, demoPatternToggle);
      renderFrame(frameCurrent);
      bootPatternStartMs = now;
    }
  }
#if ENABLE_BLE
  serviceBleReady();
#endif
}
