#define UP_SPEED 256000
#define DEFAULT_SPEED 115200
#define TX_PIN 17
#define RX_PIN 16
// Supported baud rates: 2400 4800 9600 19200 38400 57600 115200 230400 256000 512000 921600
#define NEXTION_PACKET_SIZE 4096
#define NEXTION_ACK_BYTE 0x05
#define NEXTION_ERROR_BYTE 0x08

HardwareSerial nextion(2);

int baudRates[] = {9600, 19200, 38400, 57600, 115200, 230400, 256000, 512000, 921600};
int numBauds = sizeof(baudRates) / sizeof(baudRates[0]);

void sendCmd(const char *cmd)
{
  nextion.print(cmd);
  nextion.write(0xFF);
  nextion.write(0xFF);
  nextion.write(0xFF);
}

bool checkResponse()
{
  unsigned long start = millis();
  String resp = "";
  while (millis() - start < 300)
  {
    while (nextion.available())
    {
      char c = nextion.read();
      resp += c;
    }
  }
  if (resp.length() > 0)
  {
    Serial.print("Response received: ");
    for (int i = 0; i < resp.length(); i++)
    {
      Serial.printf("%02X ", (uint8_t)resp[i]);
    }
    Serial.println();
    if (resp[0] != 0xff)
      return true;
    else
      return false;
  }
  return false;
}

int getbaud()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nStarting baud rate detection...");

  for (int i = 0; i < numBauds; i++)
  {
    int baud = baudRates[i];
    Serial.printf("Trying %d bps ...\n", baud);

    nextion.begin(baud, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(200);

    // Clear buffer
    while (nextion.available())
      nextion.read();

    // Send bauds=<baud>
    char buf[32];
    sprintf(buf, "bauds=%d", baud);
    sendCmd(buf);
    delay(200);

    // Send get baud
    sendCmd("get baud");

    if (checkResponse())
    {
      Serial.printf("✅ Found working baud rate: %d bps\n", baud);
      return baud;
    }
    else
    {
      Serial.println("❌ No response");
    }
  }
  return -1;
}

// Upgrade state
enum UpgradeState
{
  UPGRADE_IDLE,
  UPGRADE_CMD_SENT,
  UPGRADE_READY,
  UPGRADE_TRANSFERRING,
  UPGRADE_COMPLETE,
  UPGRADE_ERROR
};
UpgradeState upgradeState = UPGRADE_IDLE;

// Prepare state
enum PrepareState
{
  PREP_IDLE,
  PREP_IN_PROGRESS,
  PREP_DONE,
  PREP_ERROR
};
PrepareState prepareState = PREP_IDLE;

static size_t expectedFileSize = 0;
static size_t totalReceived = 0;
static size_t currentPacketSize = 0;
static uint8_t packetBuffer[NEXTION_PACKET_SIZE];
static unsigned long lastProgressTime = 0;

// ---------------- WHMI-WRIS Send Packet ----------------
bool sendPacket(uint8_t *data, size_t size, int retryCount = 3)
{
  for (int attempt = 1; attempt <= retryCount; attempt++)
  {
    while (nextion.available())
      nextion.read();
    size_t written = 0;
    const size_t CHUNK_SIZE = 256;
    for (size_t i = 0; i < size; i += CHUNK_SIZE)
    {
      size_t chunkSize = min(CHUNK_SIZE, size - i);
      size_t chunkWritten = nextion.write(data + i, chunkSize);
      written += chunkWritten;
      delayMicroseconds(500);
    }
    if (written != size)
    {
      if (attempt < retryCount)
      {
        delay(100);
        continue;
      }
      return false;
    }

    unsigned long startTime = millis();
    while (millis() - startTime < 5000)
    {
      yield();
      if (nextion.available())
      {
        uint8_t resp = nextion.read();
        if (resp == NEXTION_ACK_BYTE)
          return true;
        else if (resp == NEXTION_ERROR_BYTE)
        {
          if (attempt < retryCount)
          {
            delay(200);
            break;
          }
          return false;
        }
      }
      delay(10);
    }
  }
  return false;
}
void Testband()
{
  for (int i = 0; i < numBauds; i++)
  {
    int baud = baudRates[i];

    // Send command at current speed to switch display baud rate
    nextion.printf("baud=%d", baud);
    nextion.write(0xFF);
    nextion.write(0xFF);
    nextion.write(0xFF);
    delay(500);

    // Immediately initialize serial port at new speed
    nextion.begin(baud, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(300);

    // Clear buffer
    while (nextion.available())
      nextion.read();

    // Verify if actually at this baud rate
    sendCmd("get baud");
    if (checkResponse())
    {
      Serial.printf("✅ Display confirmed current baud rate: %d bps\n", baud);
    }
    else
    {
      Serial.printf("❌ No response after setting to %d bps\n", baud);
    }

    delay(1000);
  }
}

void startPrepare(size_t fileSize)
{
  expectedFileSize = fileSize;
  totalReceived = 0;
  currentPacketSize = 0;
  upgradeState = UPGRADE_CMD_SENT;
  prepareState = PREP_IN_PROGRESS;

  xTaskCreate([](void *)
              {
    for (int itry = 0; itry < 5; itry++) {
      Serial.printf("[PrepareTask] Task started %d\n", itry);
      //nextion.updateBaudRate(DEFAULT_SPEED);
      //delay(2000);
      if (getbaud() == -1) {
        prepareState = PREP_ERROR;
        upgradeState = UPGRADE_ERROR;
        vTaskDelete(NULL);
        return;
      }

      // Clear display serial buffer
      while (nextion.available()) nextion.read();

      // Send reset command
      Serial.println("[PrepareTask] Sending reset");
      nextion.print("rest");
      nextion.write(0xFF);
      nextion.write(0xFF);
      nextion.write(0xFF);
      delay(2000);
      while (nextion.available()) {
        uint8_t resp = nextion.read();
        Serial.print("[PrepareTask] Received byte: 0x");
        Serial.println(resp, HEX);
        nextion.read();
      }
      // Set baud rate
      if (itry == 0) {
        Serial.printf("[PrepareTask] Setting baud rate %d\n", UP_SPEED);
        nextion.printf("baud=%d", UP_SPEED);
      } else {
        Serial.printf("[PrepareTask] Setting baud rate %d\n", DEFAULT_SPEED);
        nextion.printf("baud=%d", DEFAULT_SPEED);
      }
      nextion.write(0xFF);
      nextion.write(0xFF);
      nextion.write(0xFF);
      delay(2000);
      while (nextion.available()) {
        uint8_t resp = nextion.read();
        Serial.print("[PrepareTask] Received byte: 0x");
        Serial.println(resp, HEX);
        nextion.read();
      }

      if (itry == 0) {
        nextion.updateBaudRate(UP_SPEED);
        Serial.printf("[PrepareTask] Switching serial port to %d\n", UP_SPEED);
      } else {
        nextion.updateBaudRate(DEFAULT_SPEED);
        Serial.printf("[PrepareTask] Switching serial port to %d\n", DEFAULT_SPEED);
      }
      delay(2000);
      while (nextion.available()) {
        uint8_t resp = nextion.read();
        Serial.print("[PrepareTask] Received byte: 0x");
        Serial.println(resp, HEX);
        nextion.read();
      }
      String cmd;
      // Send WHMI-WRIS command
      if (itry == 0)
        cmd = "whmi-wri " + String(expectedFileSize) + "," + String(UP_SPEED) + ",0";
      else
        cmd = "whmi-wri " + String(expectedFileSize) + "," + String(DEFAULT_SPEED) + ",0";

      Serial.println("[PrepareTask] Sending command: ");
      Serial.println(cmd);
      nextion.print(cmd);
      nextion.write(0xFF);
      nextion.write(0xFF);
      nextion.write(0xFF);

      bool gotAck = false;
      unsigned long t0 = millis();
      while (millis() - t0 < 10000) {
        yield();
        if (nextion.available()) {
          uint8_t resp = nextion.read();
          Serial.print("[PrepareTask] Received byte: 0x");
          Serial.println(resp, HEX);
          if (resp == NEXTION_ACK_BYTE) {
            Serial.println("[PrepareTask] ACK received");
            gotAck = true;
            break;
          } else {
            Serial.println("[PrepareTask] Error byte received");
            if (itry == 4) {
              prepareState = PREP_ERROR;
              upgradeState = UPGRADE_ERROR;
              vTaskDelete(NULL);
              return;
            }
            break;
          }
        }
        delay(10);
      }

      if (gotAck) {
        prepareState = PREP_DONE;
        upgradeState = UPGRADE_READY;
        Serial.println("[PrepareTask] Display preparation complete");
        break;
      } else {
        if (itry == 4) {
          prepareState = PREP_ERROR;
          upgradeState = UPGRADE_ERROR;
        }
        Serial.println("[PrepareTask] Display response timeout");
      }
      delay(2000);
    }
    vTaskDelete(NULL); },
              "PrepareTask", 4096, NULL, 1, NULL);
}