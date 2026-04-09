#include "Arduino.h"
#include "WiFiMulti.h"
#include "Audio.h"
#include <Arduino_GFX_Library.h>
#include <LovyanGFX.hpp>
#include "FS.h"
#include "SD_MMC.h"

#define BCK  8
#define LCK 10
#define DIN  9

#define GFX_BL 46
#define LCD_RST 40

unsigned short bck=0xA400;

int clk = 16;
int cmd = 15;
int d0 = 17;
int d1 = 18;
int d2 = 13;
int d3 = 14;  // GPIO 34 is not broken-out on ESP32-S3-DevKitC-1 v1.1

bool mode=1; // 0 is sd card 1 is webradio

int lines[320]={0};

Audio audio;
WiFiMulti wifiMulti;

String ssid = "xxxxxxx";
String password = "xxxxxxx;

LGFX_Sprite sprite;

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  45 /* DC */, 21 /* CS */, 38 /* SCK */, 39 /* MOSI */
);

Arduino_GFX *gfx = new Arduino_ST7789(
  bus, 47 /* RST */, 0 /* rotation */, false /* IPS */,
  172 /* width */, 320 /* height */,
  34 /*col_offset1*/, 0,
  34 /*col_offset2*/, 0
);

#define AUDIO_VIS_SAMPLES 1024
#define PIXELS_PER_BLOCK 4
#define SAMPLES_PER_PIXEL (AUDIO_VIS_SAMPLES / PIXELS_PER_BLOCK)

int32_t audioVisBuffer[AUDIO_VIS_SAMPLES];
int audioVisIndex = 0;
uint16_t pixelHeights[PIXELS_PER_BLOCK]; // 4 visine

const int SCREEN_HEIGHT = 160; // prilagodi stvarnoj visini ekrana

volatile uint16_t latestHeights[2];
volatile bool newData = false;

// 🎚 EQ gaini (možeš mijenjati u realnom vremenu)
float bassGain   = 1; // niske frekvencije
float midGain    = 1; // srednje frekvencije
float trebleGain = 1; // visoke frekvencije

// unutarnje varijable filtera
int32_t lowPass  = 0;
int32_t highPass = 0;

void audio_process_raw_samples(int32_t* outBuff, int16_t validSamples) {

    for (int i = 0; i < validSamples; i++) {

        // 32 → 16 bit
        int32_t x = outBuff[i] >> 16;

        // -------------------------
        //   BASS  (low-pass)
        //   cutoff ~300 Hz
        // -------------------------
        lowPass = lowPass + ((x - lowPass) >> 5);
        int32_t bass = lowPass;

        // -------------------------
        //   TREBLE (high-pass)
        //   cutoff ~5 kHz
        // -------------------------
        highPass = highPass + ((x - highPass) >> 2);
        int32_t treble = x - highPass;

        // -------------------------
        //   MID (band-pass)
        //   mid = original - bass - treble
        // -------------------------
        int32_t mid = x - bass - treble;

        // -------------------------
        //   APPLY GAINS
        // -------------------------
        int32_t y =
            bass   * bassGain +
            mid    * midGain +
            treble * trebleGain;

        // -------------------------
        //   PEAK LIMITER (soft clipper)
        // -------------------------
        const int32_t limit = 30000;

        if (y > limit)
            y = limit + (y - limit) / 4;   // mekano savijanje vrha
        else if (y < -limit)
            y = -limit + (y + limit) / 4;

        // sigurnosni hard clip (nikad ne bi trebao aktivirati)
        if (y > 32767) y = 32767;
        if (y < -32768) y = -32768;

        // back to 32-bit
        outBuff[i] = y << 16;

        // visualization
        audioVisBuffer[audioVisIndex] = outBuff[i];
        audioVisIndex = (audioVisIndex + 1) % AUDIO_VIS_SAMPLES;
    }

    // --- izračun visina za 4 piksela ---
    for (int px = 0; px < PIXELS_PER_BLOCK; px++) {
        int64_t sum = 0;
        for (int s = 0; s < SAMPLES_PER_PIXEL; s++) {
            int idx = (px * SAMPLES_PER_PIXEL + s) % AUDIO_VIS_SAMPLES;
            sum += abs(audioVisBuffer[idx]);
        }

        int32_t avg = sum / SAMPLES_PER_PIXEL;

        latestHeights[px] = (uint16_t)((int64_t)avg * SCREEN_HEIGHT / 2147483647);
        if (latestHeights[px] > SCREEN_HEIGHT)
            latestHeights[px] = SCREEN_HEIGHT;
    }

    newData = true;
}


/*
void audio_process_raw_samples(int32_t* outBuff, int16_t validSamples) {
    // spremanje u kružni buffer za vizualizaciju
    for (int i = 0; i < validSamples; i++) {
        audioVisBuffer[audioVisIndex] = outBuff[i];
        audioVisIndex = (audioVisIndex + 1) % AUDIO_VIS_SAMPLES;
    }

    // izračun visina za 4 piksela
    for (int px = 0; px < PIXELS_PER_BLOCK; px++) {
        int64_t sum = 0;
        for (int s = 0; s < SAMPLES_PER_PIXEL; s++) {
            int idx = (px * SAMPLES_PER_PIXEL + s) % AUDIO_VIS_SAMPLES;
            sum += abs(audioVisBuffer[idx]);
        }

        int32_t avg = sum / SAMPLES_PER_PIXEL;

        // skaliranje na visinu ekrana
        latestHeights[px] = (uint16_t)((int64_t)avg * SCREEN_HEIGHT / 2147483647);
        if (latestHeights[px] > SCREEN_HEIGHT) 
            latestHeights[px] = SCREEN_HEIGHT;
    }

    newData = true; // signal da imamo novi audio za crtanje
}
*/



void lcd_reg_init(void) {

  static const uint8_t init_operations[] = {
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x11,  // 2: Out of sleep mode, no args, w/delay
    END_WRITE,
    DELAY, 120,

    BEGIN_WRITE,
    WRITE_C8_D16, 0xDF, 0x98, 0x53,
    WRITE_C8_D8, 0xB2, 0x23, 

    WRITE_COMMAND_8, 0xB7,
    WRITE_BYTES, 4,
    0x00, 0x47, 0x00, 0x6F,

    WRITE_COMMAND_8, 0xBB,
    WRITE_BYTES, 6,
    0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0,

    WRITE_C8_D16, 0xC0, 0x44, 0xA4,
    WRITE_C8_D8, 0xC1, 0x16, 

    WRITE_COMMAND_8, 0xC3,
    WRITE_BYTES, 8,
    0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77,

    WRITE_COMMAND_8, 0xC4,
    WRITE_BYTES, 12,
    0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82,

    WRITE_COMMAND_8, 0xC8,
    WRITE_BYTES, 32,
    0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00, 0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,

    WRITE_COMMAND_8, 0xD0,
    WRITE_BYTES, 5,
    0x04, 0x06, 0x6B, 0x0F, 0x00,

    WRITE_C8_D16, 0xD7, 0x00, 0x30,
    WRITE_C8_D8, 0xE6, 0x14, 
    WRITE_C8_D8, 0xDE, 0x01, 

    WRITE_COMMAND_8, 0xB7,
    WRITE_BYTES, 5,
    0x03, 0x13, 0xEF, 0x35, 0x35,

    WRITE_COMMAND_8, 0xC1,
    WRITE_BYTES, 3,
    0x14, 0x15, 0xC0,

    WRITE_C8_D16, 0xC2, 0x06, 0x3A,
    WRITE_C8_D16, 0xC4, 0x72, 0x12,
    WRITE_C8_D8, 0xBE, 0x00, 
    WRITE_C8_D8, 0xDE, 0x02, 

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3,
    0x00, 0x02, 0x00,

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3,
    0x01, 0x02, 0x00,

    WRITE_C8_D8, 0xDE, 0x00, 
    WRITE_C8_D8, 0x35, 0x00, 
    WRITE_C8_D8, 0x3A, 0x05, 

    WRITE_COMMAND_8, 0x2A,
    WRITE_BYTES, 4,
    0x00, 0x22, 0x00, 0xCD,

    WRITE_COMMAND_8, 0x2B,
    WRITE_BYTES, 4,
    0x00, 0x00, 0x01, 0x3F,

    WRITE_C8_D8, 0xDE, 0x02, 

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3,
    0x00, 0x02, 0x00,
    
    WRITE_C8_D8, 0xDE, 0x00, 
    WRITE_C8_D8, 0x36, 0x00,
    WRITE_COMMAND_8, 0x21,
    END_WRITE,
    
    DELAY, 10,

    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x29,  // 5: Main screen turn on, no args, w/delay
    END_WRITE
  };
  bus->batchOperation(init_operations, sizeof(init_operations));
}

// ---------------- LCD TASK ----------------
int nbuf=80;

void drawAudioVis() {
    sprite.fillSprite(bck);

    sprite.drawString(String(bassGain),10,4);
    sprite.drawString(String(midGain),50,4);
    sprite.drawString(String(trebleGain),90,4);

    for(int i=0;i<320;i++){
    sprite.drawFastVLine(i,75-lines[i]/2,lines[i],0x260C);
    sprite.drawFastVLine(i,75-((lines[i]/2)*0.6),lines[i]*0.6,0xEC36);
    }

    gfx->draw16bitRGBBitmap(0, 30, (uint16_t*)sprite.getBuffer(), 320, 140);
}

void lcdTask(void *param) {
  while (1) {
    
       if(newData){
        newData = false;

        // shift ULijevo (GLATKO!)
        for(int i=0;i<316;i++)
        lines[i]=lines[i+4];

        lines[319]=latestHeights[3];
        lines[318]=latestHeights[2];
        lines[317]=latestHeights[1];
        lines[316]=latestHeights[0];
       
    }
   drawAudioVis();

    vTaskDelay(10);   // ~100 FPS, dovoljno brzo, ne blokira audio
  }
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);

  pinMode(LCD_RST, OUTPUT);
  digitalWrite(LCD_RST, 0);
  delay(10);
  digitalWrite(LCD_RST, 1);

  analogWrite(GFX_BL, 80); // screen brightness

  gfx->begin();
  lcd_reg_init();
  gfx->setRotation(3);
  gfx->fillScreen(0);
  gfx->drawFastHLine(0,171,320,TFT_WHITE);
  gfx->drawFastHLine(0,29,320,TFT_WHITE);

  sprite.createSprite(320,140);
 
  audio.setPinout(BCK, LCK, DIN);
  audio.setVolume(16);


  if(mode==1)
  {
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(ssid.c_str(), password.c_str());
  wifiMulti.run();
  audio.connecttohost("http://s3.iqstreaming.com:8002/;");
  }
  else
  {
    if(! SD_MMC.setPins(clk, cmd, d0, d1, d2, d3)){
    Serial.println("Pin change failed!");
    return;
  }
  if (!SD_MMC.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  audio.connecttoFS(SD_MMC, "song.mp3");
  }



  // 🔥 LCD ide na CORE 1
  xTaskCreatePinnedToCore(
    lcdTask,
    "LCD",
    4096,
    NULL,
    1,
    NULL,
    1
  );
}

// ---------------- LOOP ----------------

void loop() {
 int potBass   = analogRead(4);
int potMid    = analogRead(6);
int potTreble = analogRead(5);

bassGain   = map(potBass,   0, 4095, 2, 18) / 10.0;
midGain    = map(potMid,    0, 4095, 2, 18) / 10.0;
trebleGain = map(potTreble, 0, 4095, 2, 18) / 10.0;
    

  audio.loop();   // audio ostaje na core 0
  vTaskDelay(1);
 
}

// optional callbacks
void audio_info(const char *info) { Serial.println(info); }
void audio_id3data(const char *info) { Serial.println(info); }
void audio_eof_mp3(const char *info) { Serial.println(info); }
void audio_showstation(const char *info) { Serial.println(info); }
void audio_showstreamtitle(const char *info) { Serial.println(info); }
void audio_bitrate(const char *info) { Serial.println(info); }
void audio_commercial(const char *info) { Serial.println(info); }
void audio_icyurl(const char *info) { Serial.println(info); }
void audio_lasthost(const char *info) { Serial.println(info); }
