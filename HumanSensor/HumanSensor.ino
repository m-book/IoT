
#include <M5Stack.h>
#include <Wire.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

const byte MLX90640_address = 0x33; 
#define TA_SHIFT 8 

#define COLS 32
#define ROWS 24
#define COLS_2 (COLS * 2)
#define ROWS_2 (ROWS * 2)

float pixelsArraySize = COLS * ROWS;
float pixels[COLS * ROWS];
float pixels_2[COLS_2 * ROWS_2];
float reversePixels[COLS * ROWS];

byte speed_setting = 2 ;
bool reverseScreen = false;
//bool reverseScreen = true;

#define INTERPOLATED_COLS 32
#define INTERPOLATED_ROWS 32

static float mlx90640To[COLS * ROWS];
paramsMLX90640 mlx90640;
float signedMag12ToFloat(uint16_t val);

int MINTEMP = 24;
int min_v = 24;
int min_cam_v = -40;

int MAXTEMP = 35; 
int max_v = 35;
int max_cam_v = 300;
int resetMaxTemp = 45;

const uint16_t camColors[] = {0x480F,
                              0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810, 0x3010, 0x3010,
                              0x3010, 0x2810, 0x2810, 0x2810, 0x2810, 0x2010, 0x2010, 0x2010, 0x1810, 0x1810,
                              0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811, 0x0811, 0x0811, 0x0011, 0x0011,
                              0x0011, 0x0011, 0x0011, 0x0031, 0x0031, 0x0051, 0x0072, 0x0072, 0x0092, 0x00B2,
                              0x00B2, 0x00D2, 0x00F2, 0x00F2, 0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192,
                              0x0192, 0x01B2, 0x01D2, 0x01F3, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0253, 0x0273,
                              0x0293, 0x02B3, 0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353, 0x0373,
                              0x0394, 0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474,
                              0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554, 0x0574,
                              0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572, 0x0571, 0x0591, 0x0591,
                              0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE, 0x05AE, 0x05AD, 0x05AD,
                              0x05AD, 0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB, 0x05CA, 0x05CA, 0x05CA, 0x05C9,
                              0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7, 0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5,
                              0x05E5, 0x0604, 0x0604, 0x0604, 0x0603, 0x0603, 0x0602, 0x0602, 0x0601, 0x0621,
                              0x0621, 0x0620, 0x0620, 0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640,
                              0x1E40, 0x1E40, 0x2640, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60,
                              0x3E60, 0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680, 0x5E80, 0x5E80, 0x6680,
                              0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0, 0x8EA0,
                              0x8EC0, 0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0, 0xB6E0, 0xB6E0,
                              0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0, 0xCEE0, 0xD6E0, 0xD700, 0xDF00, 0xDEE0,
                              0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640, 0xE620, 0xE600, 0xE5E0, 0xE5C0,
                              0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520, 0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480,
                              0xE460, 0xEC40, 0xEC20, 0xEC00, 0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40,
                              0xEB20, 0xEB00, 0xEAE0, 0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200,
                              0xF1E0, 0xF1C0, 0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0,
                              0xF080, 0xF060, 0xF040, 0xF020, 0xF800,
                             };



float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, float *dest, uint8_t dest_rows, uint8_t dest_cols);

long loopTime, startTime, endTime, fps;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    pCharacteristic->setValue("Hello World!");
  }

  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
  }
};

void task_ble(void* arg) {
  M5.setWakeupButton(BUTTON_A_PIN);
  m5.Speaker.mute();

  BLEDevice::init("m5-stack");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void setup()
{
  M5.begin();
  Wire.begin();
  Wire.setClock(450000);
  Serial.begin(115200);
  M5.Lcd.begin();
  M5.Lcd.setRotation(1);

  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextColor(YELLOW, BLACK);

  while (!Serial); 
  Serial.println("M5Stack MLX90640 IR Camera");
  M5.Lcd.setTextSize(2);

  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");

  int SetRefreshRate;

  SetRefreshRate = MLX90640_SetRefreshRate (0x33, 0x05);

  M5.Lcd.fillScreen(TFT_BLACK);
  int icolor = 0;
  for (int icol = 0; icol <= 248;  icol++)
  {
    M5.Lcd.drawRect(36, 208, icol, 284 , camColors[icolor]);
    icolor++;
  }
  infodisplay();



  M5.setWakeupButton(BUTTON_A_PIN);
  m5.Speaker.mute();

  BLEDevice::init("m5-stack");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}


void loop()
{
  loopTime = millis();
  startTime = loopTime;

  if (M5.BtnA.pressedFor(1000)) {
    if (MINTEMP <= 5 )
    {
      MINTEMP = MAXTEMP - 5;
    }
    else
    {
      MINTEMP = MINTEMP - 5;
    }
    infodisplay();
  }

  if (M5.BtnA.wasPressed()) {
    if (MINTEMP <= 0)
    {
      MINTEMP = MAXTEMP - 1;
    }
    else
    {
      MINTEMP--;
    }
    infodisplay();
  }

  if (M5.BtnB.wasPressed()) {
    MINTEMP = min_v - 1;
    MAXTEMP = max_v + 1;
    infodisplay();
  }
  
  if (M5.BtnB.pressedFor(1000)) {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.drawCentreString("Power Off...", 160, 80, 4);
    delay(1000);
    M5.powerOFF();
  }
  
  if (M5.BtnC.pressedFor(1000)) {
    if (MAXTEMP >= max_cam_v)
    {
      MAXTEMP = MINTEMP + 1;
    }
    else
    {
      MAXTEMP = MAXTEMP + 5;
    }
    infodisplay();
  }

  if (M5.BtnC.wasPressed()) {
    if (MAXTEMP >= max_cam_v )
    {
      MAXTEMP = MINTEMP + 1;
    }
    else
    {
      MAXTEMP++;
    }
    infodisplay();
  }

  M5.update();

  for (byte x = 0 ; x < speed_setting ; x++)
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
    float tr = Ta - TA_SHIFT; 
    float emissivity = 0.95;
    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, pixels);
  }

  if (reverseScreen == 1)
  {
    for (int x = 0 ; x < pixelsArraySize ; x++)
    {
      if (x % COLS == 0)
      {
        for (int j = 0 + x, k = (COLS-1) + x; j < COLS + x ; j++, k--)
        {
          reversePixels[j] = pixels[k];
        }
      }
    }
  }

  float dest_2d[INTERPOLATED_ROWS * INTERPOLATED_COLS];
  int ROWS_i,COLS_j;

  if (reverseScreen == 1)
  {
    interpolate_image(reversePixels, ROWS, COLS, dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);
  }
  else
  {

    interpolate_image(pixels, ROWS, COLS, dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);
    for(int y = 0;y < ROWS;y++)
    {
      for(int x = 0;x < COLS;x++)
      {

        pixels_2[(((y * 2) * (COLS*2)) + (x * 2))] = pixels[y*COLS+x];
        
        if(x != 31)
          pixels_2[(((y * 2) * (COLS*2)) + (x * 2)+1)] = ( pixels_2[(((y * 2) * (COLS*2)) + (x * 2))] + pixels_2[(((y * 2) * (COLS*2)) + (x * 2)+2)]) / 2;
        else
          pixels_2[(((y * 2) * (COLS*2)) + (x * 2)+1)] = ( pixels_2[(((y * 2) * (COLS*2)) + (x * 2))] );
          
      }
    }
 
    for(int y = 0;y < ROWS;y++)//24
    {
      for(int x = 0;x < COLS_2;x++)//64
      {
        if(y != 23)
          pixels_2[(((y * 2) + 1) * (COLS_2)) + x] = ( pixels_2[(((y * 2) * COLS_2) + x)] + pixels_2[((((y * 2) + 2) * COLS_2) + x)] ) / 2;
        else
          pixels_2[(((y * 2) + 1) * (COLS_2)) + x] = ( pixels_2[(((y * 2) * COLS_2) + x)] + pixels_2[(((y * 2) * COLS_2) + x)] ) / 2;
      }
    }   
  }

  uint16_t boxsize = min(M5.Lcd.width() / INTERPOLATED_ROWS, M5.Lcd.height() / INTERPOLATED_COLS);
  uint16_t boxWidth = M5.Lcd.width() / INTERPOLATED_ROWS;
  uint16_t boxHeight = (M5.Lcd.height() - 31) / INTERPOLATED_COLS;
  drawpixels(dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS, boxWidth, boxHeight, false);
  max_v = MINTEMP;
  min_v = MAXTEMP;
  int spot_v = pixels[360];
  spot_v = pixels[768/2-16];

   for ( int itemp = 0; itemp < sizeof(pixels) / sizeof(pixels[0]); itemp++ )
  {
    if ( pixels[itemp] > max_v )
    {
      max_v = pixels[itemp];
    }
    if ( pixels[itemp] < min_v )
    {
      min_v = pixels[itemp];
    }
  }


  M5.Lcd.setTextSize(2);
  M5.Lcd.fillRect(164, 220, 75, 18, TFT_BLACK);
  M5.Lcd.fillRect(60, 220, 200, 18, TFT_BLACK); 
    int icolor = 0;

  M5.Lcd.setCursor(60, 222);
  M5.Lcd.setTextColor(TFT_WHITE);

  if (max_v > max_cam_v | max_v < min_cam_v ) {
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.printf("Error", 1);
  }
  else
  {
    M5.Lcd.printf("Min:", 1);
    M5.Lcd.print(min_v, 1);
    M5.Lcd.printf("C  " , 1);
    M5.Lcd.printf("Max:", 1);
    M5.Lcd.print(max_v, 1);
    M5.Lcd.printf("C" , 1);
    M5.Lcd.setCursor(180, 94);
    M5.Lcd.print(spot_v, 1);
    M5.Lcd.printf("C" , 1);
    M5.Lcd.drawCircle(160, 120, 6, TFT_WHITE);
    M5.Lcd.drawLine(160, 110, 160, 130, TFT_WHITE);
    M5.Lcd.drawLine(150, 120, 170, 120, TFT_WHITE);
  }
  loopTime = millis();
  endTime = loopTime;
  fps = 1000 / (endTime - startTime);
  M5.Lcd.fillRect(300, 209, 20, 12, TFT_BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(284, 210);
  M5.Lcd.print("fps:" + String( fps ));
  M5.Lcd.setTextSize(1);

  char numStr[6];
  int count = 0;

  for(int i=0; i < COLS * ROWS; i++){
    if(i %10 != 0){
      continue;
    }
    snprintf(numStr, 6, "%f", pixels[i]);
    pCharacteristic->setValue(numStr);
    pCharacteristic->notify();
    count++;
  }
  Serial.println(count);
    delay(200);

}


void infodisplay(void) {
  M5.Lcd.fillRect(0, 198, 320, 4, TFT_WHITE);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.fillRect(284, 223, 320, 240, TFT_BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(284, 222);
  M5.Lcd.print(MAXTEMP , 1); 
  M5.Lcd.printf("C" , 1);
  M5.Lcd.setCursor(0, 222);
  M5.Lcd.fillRect(0, 222, 36, 16, TFT_BLACK);
  M5.Lcd.print(MINTEMP , 1);
  M5.Lcd.printf("C" , 1);
  M5.Lcd.setCursor(106, 224);
}

void drawpixels(float *p, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight, boolean showVal) {
  int colorTemp;
  for (int y = 0; y < rows; y++) 
  {
    for (int x = 0; x < cols; x++) 
    {
      float val = get_point(p, rows, cols, x, y);
      
      if (val >= MAXTEMP) 
        colorTemp = MAXTEMP;
      else if (val <= MINTEMP) 
        colorTemp = MINTEMP;
      else colorTemp = val;

      uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);
      //draw the pixels!
      uint16_t color;
      color = val * 2;
      M5.Lcd.fillRect(boxWidth * x, boxHeight * y, boxWidth, boxHeight, camColors[colorIndex]);
    }
  }
}

boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}
