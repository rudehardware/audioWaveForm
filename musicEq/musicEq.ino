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
#define BUT_PIN 3

int volume=9;

unsigned short bck=0xA400;

unsigned short color[24] = {
  0x00F8, 0x40F9, 0xE0FB, 0x20FD, 0xE0FF, 0xE0BF,
  0xE087, 0xE007, 0xF007, 0xFF07, 0xDF07, 0xBF03,
  0x1F00, 0x1F40, 0x1F78, 0x1FA0, 0x1FF8, 0x1BF8,
  0x19F8, 0x18F8, 0xA0F9, 0x00FA, 0x00F4,0x00E0};
  
 uint16_t gray[12] = {
    0xFFFF, // 0  - gotovo bijela
    0xE79C, // 1
    0xD6BA, // 2
    0xC638, // 3
    0xB5B6, // 4
    0xA534, // 5
    0x9492, // 6
    0x8410, // 7
    0x73AE, // 8
    0x632C, // 9
    0x528A, // 10
    0x4208  // 11 - gotovo crna
};

int clk = 16;
int cmd = 15;
int d0 = 17;
int d1 = 18;
int d2 = 13;
int d3 = 14;

bool mode=1; // 0 = sd card, 1 = webradio
bool deb=0;
bool deb2=0;
int colorIndex=10;
int lines[320]={0};

Audio audio;
WiFiMulti wifiMulti;

String ssid = "XXXXXXXXXXXXX";
String password = "XXXXXXXXXX";

LGFX_Sprite sprite;
LGFX_Sprite sprite2;

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  45 /* DC */, 21 /* CS */, 38 /* SCK */, 39 /* MOSI */
);

Arduino_GFX *gfx = new Arduino_ST7789(
  bus, 47 /* RST */, 0 /* rotation */, false /* IPS */,
  172 /* width */, 320 /* height */,
  34 /*col_offset1*/, 0,
  34 /*col_offset2*/, 0
);

// ---------------- AUDIO VIS / FFT ----------------

#define AUDIO_VIS_SAMPLES 1024
#define PIXELS_PER_BLOCK 4
#define SAMPLES_PER_PIXEL (AUDIO_VIS_SAMPLES / PIXELS_PER_BLOCK)

int32_t audioVisBuffer[AUDIO_VIS_SAMPLES];
int audioVisIndex = 0;

const int SCREEN_HEIGHT = 160;

volatile uint16_t latestHeights[4];
volatile bool newData = false;

// EQ gains
float bassGain   = 1.0f;
float midGain    = 1.0f;
float trebleGain = 1.0f;

// filter state
int32_t lowPass  = 0;
int32_t highPass = 0;

// --------- FFT (KISS-style minimal implementation) ---------

#define FFT_SIZE 1024
#define FFT_BINS 24

struct kiss_fft_cpx {
  float r;
  float i;
};

kiss_fft_cpx fftIn[FFT_SIZE];
kiss_fft_cpx fftOut[FFT_SIZE];
uint16_t spectrum[FFT_BINS];

bool showSpectrum = false; // false = waveform, true = spectrum

void kiss_fft(int n, kiss_fft_cpx *in, kiss_fft_cpx *out) {
  // simple radix-2 iterative FFT, N must be power of 2
  int i, j, k, m;
  int n1, n2, a;
  float c, s, t1, t2;

  // bit-reverse
  j = 0;
  for (i = 0; i < n; i++) {
    out[j] = in[i];
    m = n >> 1;
    while (m >= 1 && j >= m) {
      j -= m;
      m >>= 1;
    }
    j += m;
  }

  // FFT
  n1 = 0;
  n2 = 1;

  for (i = 0; (1 << i) < n; i++) {
    n1 = n2;
    n2 = n2 << 1;
    a = 0;

    for (j = 0; j < n1; j++) {
      float angle = -2.0f * PI * j / n2;
      c = cosf(angle);
      s = sinf(angle);

      for (k = j; k < n; k += n2) {
        t1 = c * out[k + n1].r - s * out[k + n1].i;
        t2 = s * out[k + n1].r + c * out[k + n1].i;

        float r0 = out[k].r;
        float i0 = out[k].i;

        out[k + n1].r = r0 - t1;
        out[k + n1].i = i0 - t2;
        out[k].r      = r0 + t1;
        out[k].i      = i0 + t2;
      }
      a++;
    }
  }
}

void computeSpectrumFromBuffer() {

    // --- 1) tAKE 1024 SAMPES ---
    int idx = audioVisIndex - FFT_SIZE;
    if (idx < 0) idx += AUDIO_VIS_SAMPLES;

    for (int i = 0; i < FFT_SIZE; i++) {
        int32_t s = audioVisBuffer[idx] >> 16;  // 32→16
        fftIn[i].r = (float)s;
        fftIn[i].i = 0.0f;

        idx++;
        if (idx >= AUDIO_VIS_SAMPLES) idx = 0;
    }

    // --- 2) FFT ---
    kiss_fft(FFT_SIZE, fftIn, fftOut);

  
    int N2 = FFT_SIZE / 2;  

    float logMin = logf(1.0f);
    float logMax = logf((float)(N2 - 1));

    for (int b = 0; b < FFT_BINS; b++) {

        
        float t1 = (float)b / FFT_BINS;
        float t2 = (float)(b + 1) / FFT_BINS;

        int startBin = (int)expf(logMin + (logMax - logMin) * t1);
        int endBin   = (int)expf(logMin + (logMax - logMin) * t2);

        if (startBin < 1) startBin = 1;
        if (endBin >= N2) endBin = N2 - 1;
        if (endBin < startBin) endBin = startBin;

        // --- 4) Magnituda ---
        float sum = 0.0f;
        for (int k = startBin; k <= endBin; k++) {
            float re = fftOut[k].r;
            float im = fftOut[k].i;
            sum += sqrtf(re * re + im * im);
        }

        sum /= (float)(endBin - startBin + 1);

        // --- 5) Log amplitude scaling ---
        float v = log10f(sum + 1.0f) * 18.0f;

        if (v < 0) v = 0;
        if (v > 120) v = 120;

        spectrum[b] = (uint16_t)v;
    }
}


// ---------------- AUDIO CALLBACK ----------------

void audio_process_raw_samples(int32_t* outBuff, int16_t validSamples) {

  for (int i = 0; i < validSamples; i++) {

    // 32 → 16 bit
    int32_t x = outBuff[i] >> 16;

    // BASS (low-pass, ~300 Hz)
    lowPass = lowPass + ((x - lowPass) >> 5);
    int32_t bass = lowPass;

    // TREBLE (high-pass, ~5 kHz)
    highPass = highPass + ((x - highPass) >> 2);
    int32_t treble = x - highPass;

    // MID (band-pass)
    int32_t mid = x - bass - treble;

    // APPLY GAINS
    int32_t y =
    (int32_t)(bass   * bassGain) +
    (int32_t)(mid    * midGain) +
    (int32_t)(treble * trebleGain);

    // PEAK LIMITER (soft clipper)
    const int32_t limit = 30000;

    if (y > limit)
      y = limit + (y - limit) / 4;
    else if (y < -limit)
      y = -limit + (y + limit) / 4;

    // sigurnosni hard clip
    if (y > 32767) y = 32767;
    if (y < -32768) y = -32768;

    // back to 32-bit
    outBuff[i] = y << 16;

    // spremi u buffer za vizualizaciju / FFT
    audioVisBuffer[audioVisIndex] = outBuff[i];
    audioVisIndex = (audioVisIndex + 1) % AUDIO_VIS_SAMPLES;
  }

  // waveform visine (4 bloka)
  for (int px = 0; px < PIXELS_PER_BLOCK; px++) {
    int64_t sum = 0;
    for (int s = 0; s < SAMPLES_PER_PIXEL; s++) {
      int idx = (px * SAMPLES_PER_PIXEL + s) % AUDIO_VIS_SAMPLES;
      sum += llabs(audioVisBuffer[idx]);
    }

    int32_t avg = sum / SAMPLES_PER_PIXEL;

    latestHeights[px] = (uint16_t)((int64_t)avg * SCREEN_HEIGHT / 2147483647);
    if (latestHeights[px] > SCREEN_HEIGHT)
      latestHeights[px] = SCREEN_HEIGHT;
  }

  newData = true;
}

// ---------------- LCD / GFX ----------------

void lcd_reg_init(void) {

  static const uint8_t init_operations[] = {
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x11,
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
    0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,
    0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,

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
    WRITE_COMMAND_8, 0x29,
    END_WRITE
  };
  bus->batchOperation(init_operations, sizeof(init_operations));
}

// waveform crtanje
void drawWaveformSprite() {
  sprite.fillSprite(bck);

  for(int i=0;i<320;i++){
    sprite.drawFastVLine(i,75-lines[i]/2,lines[i],color[colorIndex+1]);
    sprite.drawFastVLine(i,75-((lines[i]/2)*0.6),lines[i]*0.6,color[colorIndex]);
  }
}

// spectrum crtanje
void drawSpectrumSprite() {
  sprite.fillSprite(bck);

 // sprite.drawString(String(bassGain),10,4);
 // sprite.drawString(String(midGain),50,4);
 // sprite.drawString(String(trebleGain),90,4);

  int baseY = 70;
  int barWidth = 10;
  int gap = 3;

  for (int i = 0; i < FFT_BINS; i++) {
    int h = spectrum[i]-60;
    //if (h > 120) h = 120;
   
    int x = i * (barWidth + gap)+4;
    int y = baseY - h;

    sprite.fillRect(x, y, barWidth, h, color[i]);
    sprite.fillRect(x, baseY, barWidth, h, color[i]);
  }
}

void drawHeader()
{
    sprite2.fillSprite(0);

  sprite2.drawString("BASS: "+String(bassGain),10,6,2);
  sprite2.drawString("MID: "+String(midGain),100,6,2);
  sprite2.drawString("HI: "+String(trebleGain),180,6,2);
   sprite2.drawFastHLine(0,29,320,0x8A52);
   gfx->draw16bitRGBBitmap(0, 0, (uint16_t*)sprite2.getBuffer(), 320, 30);
}


void drawAudioVis() {
  if (showSpectrum) {
    drawSpectrumSprite();
  } else {
    drawWaveformSprite();
  }

  gfx->draw16bitRGBBitmap(0, 30, (uint16_t*)sprite.getBuffer(), 320, 140);
}

// LCD task
void lcdTask(void *param) {
  while (1) {

    if(newData){
      newData = false;

      // waveform shift
      for(int i=0;i<316;i++)
        lines[i]=lines[i+4];

      lines[319]=latestHeights[3];
      lines[318]=latestHeights[2];
      lines[317]=latestHeights[1];
      lines[316]=latestHeights[0];

      // FFT / spectrum update
      computeSpectrumFromBuffer();
    }

    drawAudioVis();
    vTaskDelay(5);
  }
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);

  pinMode(BUT_PIN, INPUT_PULLUP); //button PIN
    pinMode(0, INPUT_PULLUP);
  pinMode(LCD_RST, OUTPUT);
  digitalWrite(LCD_RST, 0);
  delay(10);
  digitalWrite(LCD_RST, 1);

  analogWrite(GFX_BL, 70);

  gfx->begin();
  lcd_reg_init();
  gfx->setRotation(3);
  gfx->fillScreen(0);
  gfx->drawFastHLine(0,171,320,gray[10]);
 

  sprite.createSprite(320,140);
  sprite2.createSprite(320,30);

  audio.setPinout(BCK, LCK, DIN);
  audio.setVolume(volume);

  if(mode==1)
  {
    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(ssid.c_str(), password.c_str());
    wifiMulti.run();
    audio.connecttohost("https://stream.zeno.fm/sm8uc3eg4p8uv");
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

  if(digitalRead(BUT_PIN)==0)
  {
    if(deb==0)
    {
      deb=1;
      showSpectrum=!showSpectrum;
    }
  }else deb=0;

    if(digitalRead(0)==0)
  {
    if(deb2==0)
    {
      deb2=1;
      colorIndex++; if(colorIndex>22) colorIndex=0;
    }
  }else deb2=0;

  bassGain   = map(potBass,   0, 4095, 2, 18) / 10.0;
  midGain    = map(potMid,    0, 4095, 2, 18) / 10.0;
  trebleGain = map(potTreble, 0, 4095, 2, 18) / 10.0;

  drawHeader();
 
  audio.loop();
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

