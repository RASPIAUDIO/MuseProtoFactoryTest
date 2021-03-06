/* MUSE LIGHT factory test

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
//////////////////////////////////////////////////////////////////////////
//
// MUSE LIGHT factory test
//
//////////////////////////////////////////////////////////////////////////
#include "Audio.h"
#include "SD.h"
#include "FS.h"
//#include "driver/i2s.h"

extern "C"
{
#include "hal_i2c.h"
#include "hal_i2s.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
}

#include "Arduino.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include <sys/socket.h>

#include <dirent.h>

#include <driver/i2s.h>

#include <NeoPixelBus.h>
#include "SPIFFS.h"

// Digital I/O used
#define SD_CS         13
#define SPI_MOSI      15
#define SPI_MISO      2
#define SPI_SCK       14
#define I2S_DOUT      26
#define I2S_BCLK      5
#define I2S_LRC       25
#define I2S_DIN       35
#define I2SR (i2s_port_t)0






RgbColor RED(255, 0, 0);
RgbColor GREEN(0, 255, 0);
RgbColor BLUE(0, 0, 255);
RgbColor WHITE(255, 255, 255);
RgbColor BLACK(0, 0, 0);
RgbColor YELLOW(255, 255, 0);
RgbColor REDL(64, 0, 0);
RgbColor GREENL(0, 64, 0);
RgbColor BLUEL(0, 0, 64);
RgbColor WHITEL(64, 64, 64);
RgbColor BLACKL(0, 0, 0);

#define ONled 0

#define BT GPIO_NUM_0         // button
#define SDD GPIO_NUM_34       // Sd detect
#define PW GPIO_NUM_21        // Amp power ON
//#define GS GPIO_NUM_23        // Amp Gain


bool testOK;
#define TAG "bt_sp"
// typedef int (*http_data_cb) (http_parser*, const char *at, size_t length);
// typedef int (*http_cb) (http_parser*);



static File root;
static File file;
static bool mp3ON;

const uint16_t PixelCount = 6;
const uint8_t PixelPin = 22;  
// three element pixels, in different order and speeds
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
#define BLOCK_SIZE 128


 const i2s_config_t i2s_configR = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX ), // Receive, transfer
      .sample_rate = 44100,                         // 16KHz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // 
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // although the SEL config should be left, it seems to transmit on right
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
      .dma_buf_count = 4,                           // number of buffers
      .dma_buf_len = BLOCK_SIZE                           // samples per buffer
  };
  
      i2s_pin_config_t pin_configR=
      {
      .bck_io_num = 5 ,    // BCKL
      .ws_io_num = 25 ,    // LRCL
      .data_out_num = 26,  // DOUT
      .data_in_num = 35    // DIN
      };


uint8_t header[]={
      0x52,0x49,0x46,0x46,    //"RIFF"
      0x24,0x7D,0x00,0x00,    //taille fichier - 8 (little endian)
      0x57,0x41,0x56,0x45,    //"WAVE"
      0x66,0x6d,0x74,0x20,    //"fmt "
      0x10,0x00,0x00,0x00,    //nb d'octets du bloc
      0x01,0x00,              //format PCM
      0x01,0x00,              //nombre de canaux
      0x44,0xAC,0x00,0x00,    //frequence d'echantillonnage 44100
      0x88,0x58,0x01,0x00,    //nombre d'octets a lire par seconde   88200 
      0x02,0x00,              //nombre d'octets par bloc d'échantillonnage
      0x10,0x00,              //nb de bits par echantillon
      0x64,0x61,0x74,0x61,    //"data"
      0x00,0x7D,0x00,0x00};   //nombre d'octets de donnees



      
//////////////////////////////////////////////////////////////////////////
//led refresh for Muse Light (6 leds ws2812)
/////////////////////////////////////////////////////////////////////////
void ledRefresh(int n, uint32_t v, int s)
{
strip.SetPixelColor(n,v&s);
strip.Show();
}


////////////////////////////////////////////////////////////////////////////////////////
//
// Task playing audio signal (mono, 8 bits, 44100)
//
////////////////////////////////////////////////////////////////////////////////////////


static void playAudio(void* data)
{ 
   int16_t s0,s1;
   int8_t c[16000];
   int l;  
   size_t t;
   uint16_t s16[64];

   int a = 0;
   i2s_set_clk(I2SR,44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
     
   File f = SPIFFS.open("/500hz44100Mono.wav", FILE_READ);
   if(f == NULL) printf("err opening file\n");
// header read
   f.read((uint8_t*)c, 44);
// data read   
   l = (int)f.read((uint8_t*)c, 16000);
   if(l < 0) printf("Erreur SD\n");

//  i2s_zero_dma_buffer(I2SR);
  for(int i = 0;i < l; i++)
   {  
// sample value 8bits -> 16        
       s0 = (((int16_t)(c[i]&0xFF)) - 128) << 8 ;       
// attenuation  a 
       s0 = s0 >> a; 
// buffering -> s16                
       s16[i % 64] = (uint16_t)s0;      
      if(((i+1) % 64) ==  0)  
      {
       int n = 0; 
//sending       
      while(n == 0) n = i2s_write_bytes(I2SR, (const char*)s16,128,portMAX_DELAY);   
      }      
   }
// muting after playing   
   for(int i=0;i<64;i++)s16[i] = 0;
     int n = 0; 
   while(n == 0) n = i2s_write_bytes(I2SR, (const char*)s16,128,portMAX_DELAY);     
   i2s_zero_dma_buffer(I2SR);

   f.close();
   printf ("Play End\n");
   vTaskDelete(NULL);
}
//////////////////////////////////////////////////////////////////////////////
// Task recording signal mono 16bits 44100
//
//////////////////////////////////////////////////////////////////////////////
static void recordAudio(void* data)
{
  static const int bytesToRead = 32000;
  uint8_t b[32000];
  size_t t;
  int16_t v[1024];
  int DC; 
  int i,j;
  int16_t max1, max2;
  int imax1, imax2;
  int16_t min1, min2;
  int imin1, imin2;
  i2s_set_clk(I2SR,44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  
 // File f = SD.open("/record.wav", FILE_WRITE);
 // f.write(header, 44);  

  delay(100);

  do
  {
     int n = 0;
     while(n == 0)n = i2s_read_bytes(I2SR,(char*)&b[i],128 ,portMAX_DELAY);
     i = i + n;
  }while(i < bytesToRead);

//  f.write(b, bytesToRead);
//  f.close();

//selecting sample (1024)
j = 0;
for(i=16000;i<18048;i=i+2)
{
  v[j++] = (int16_t)(b[i+1] << 8) + (int16_t) b[i];
}

// DC component
DC = 0;
for(j=0;j<1024;j++)
{
  DC = DC + v[j];
}

DC = DC / 1024;

printf("DC = %d\n",DC);

// correction
for(j=0;j<1024;j++) 
{
  v[j] = v[j] - DC;
}
 max1 = max2 = 0;
//first max
 for(j=0;j<88;j++)
 {
  if(v[j] > max1)
  {
    max1 = v[j];
    imax1 = j;
  }
 }
// next max
 for(j=88;j<176;j++)
 {
  if(v[j] > max2)
  {
    max2 = v[j];
    imax2 = j;
  }
 }
//first min
 min1 = min2 = 0;
 for(j=0;j<88;j++)
 {
  if(v[j] < min1)
  {
    min1 = v[j];
    imin1 = j;
  }
 }
//next min
 for(j=88;j<176;j++)
 {
  if(v[j] < min2)
  {
    min2 = v[j];
    imin2 = j;
  }
 }
  printf("==================> %d %d  %d %d %d\n",max1,max2, imax1,imax2, imax2-imax1);
  printf("==================> %d %d  %d %d %d\n",min1,min2, imin1,imin2, imin2-imin1);
  
// test 88 samples = 2ms -> 500hz
 // if (((imax2-imax1) == 88) && ((imin2-imin1) == 88) && (abs(imax1-imin1) >= 40) && (abs(imax1-imin1)<=50)) testOK = true;

#define DT 4
#define LO (88 - DT)
#define HI (88 + DT)
#define LOH 40
#define HIH 50
  testOK = true;
  if((imax2-imax1) < LO) testOK = false;
  if((imax2-imax1) > HI) testOK = false;
  if((imin2-imin1) < LO) testOK = false;
  if((imin2-imin1) > HI) testOK = false;
  if(abs(imax1-imin1) < LOH) testOK = false;
  if(abs(imax1-imin1) > HIH) testOK = false;
  
 

  printf("fin\n");
  vTaskDelete(NULL);
}


/////////////////////////////////////////////////////////////////////////////////////////
//
// APP init
/////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(115200);
    esp_err_t err;
    
    if(!SPIFFS.begin(true))Serial.println("Erreur SPIFFS");

///////////////////////////////////////////   
// init GPIO pins  
/////////////////////////////////////////////  
// BUTTON
    gpio_reset_pin(BT);
    gpio_set_direction(BT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BT, GPIO_PULLUP_ONLY);   
 
// power enable
    gpio_reset_pin(PW);
    gpio_set_direction(PW, GPIO_MODE_OUTPUT);

// gain slot    
 //   gpio_reset_pin(GS);
  //  gpio_set_direction(GS, GPIO_MODE_OUTPUT);
    
// SD detect    
    gpio_reset_pin(SDD);
    gpio_set_direction(SDD, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SDD, GPIO_PULLUP_ONLY);   




//////////////////////////////////////////////////////////////
// leds test black -> green -> black
/////////////////////////////////////////////////////////////

   strip.Begin();

//////////////////////////////////////////////////////////////
// buttons test
//////////////////////////////////////////////////////////////

  strip.SetPixelColor(ONled, WHITEL);
  strip.Show();

  while(gpio_get_level(BT) == 1)delay(50);
  strip.SetPixelColor(ONled, BLUEL);
  strip.Show();

////////////////////////////////////////////////////////////////
//Micro test
////////////////////////////////////////////////////////////////

// init enable and gain pins
  gpio_set_level(PW, 1);
//gpio_set_level(GS, 0);

////////////////////////////////////////////////////////////////
// test SD, speaker and microphone
// 1- plays "500hz44100Mono.wav" Sd file (a 500hz sine signal)
// 2- records it on SD ("record.wav")
// 3- Checks that que recorded signal is OK
///////////////////////////////////////////////////////////////////

//I2S port0 init:   TX, RX, mono , 16bits, 44100hz
i2s_driver_install(I2SR, &i2s_configR,0,NULL);
i2s_set_pin(I2SR, &pin_configR);
i2s_stop(I2SR);
//////////////////////////////////////////////////////////////
//up to  5 tests
/////////////////////////////////////////////////////////////
for(int i=0;i<5;i++)
{
i2s_start(I2SR);
testOK = false;
// record task
xTaskCreatePinnedToCore(recordAudio, "recordAudio", 40000, NULL, 10, NULL,0);
delay(10);
// play task
xTaskCreatePinnedToCore(playAudio, "playAudio", 20000, NULL, 10, NULL,1);
delay(2000);
i2s_stop(I2SR);
if(testOK == true) break;
}

if(testOK == true)
{
//////////////////////////////////////////////////////////////////// 
//test write/read on SD
////////////////////////////////////////////////////////////////////  
 char b[15];

 SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
 delay(500);
 if(!SD.begin(SD_CS))
  {
    printf("init SD failed!\n");
    testOK = false;
  }  
 File f = SD.open("/record.wav", FILE_WRITE);
 f.write((uint8_t*)"MuseRosAndCO", 13);
 f.close(); 
 f = SD.open("/record.wav", FILE_READ);
 f.read ((uint8_t*)b, 13);
 f.close(); 
 if(strcmp(b, "MuseRosAndCO") != 0)
 {
 testOK = false;
 }
 SD.remove("/record.wav");
 if(testOK == false) strip.SetPixelColor(ONled, YELLOW);
 else strip.SetPixelColor(ONled, GREEN);
 strip.Show();
}
else
{
 strip.SetPixelColor(ONled, REDL);
 strip.Show(); 
}

}
 void loop(){
  
 }
