// TheGolem
//
// Peter Todd 25/6/2022
//
// Z80 <-> ESP32 interface through IO ports
//
// And Camputers Lynx external disk drive emulation (in the ESP32)
//
// External address decoding for ports 5x (or whatever your external decoding desires) read and write
//
// External decoding is required for the IO port address range.
//
// The external addressed signal (inverted) clocks in a /WAIT via a FF which MUST be cleared by the ESP otherwise the zombies will eat you all
//
// on Z80 writes - drops /WAIT (on both r and w, but w wait not needed if core polling is fast enough - done anyway (tested 200kBps without one-way))
//
// on Z80 reads - it drops /BUSREQ during the output of data back to the Z80 from the ESP32
//                then raises /WAIT to allow the IO request to complete (using the data presented by the ESP32 by outputting on the data bus)
//                *as soon as /IOREQ raises the bus can be released*
//                and then /BUSREQ is raised to allow the Z80 to continue
//
// There will be implications due to refresh of DRAM on the host Z80 system (doesn't occur during wait)
//
// In this example the interface emulates (the essential for Camputers Lynx) ports of a 1793 Floppy Disk Controller
//
// Sixteen ports in the &5x range are emulated. Some are not implemented, some are for the FDC, some are for control of the ESP32 side (e.g. changing currently selected disk image)
//
// Speeds are in the 3us per byte transfer depending upon RTOS intrusions/race conditions, the response loop is tight but ESP32 inputs from the pins seem slow - order 100ns?, 
// and the response to a dropped /ADDRESSED io port could vary between 100-200ns, sometimes more if the RTOS intervenes(?sprite_tm input req.).
//
// Z80 was running at 4MHz with a Muxed address bus low byte first coincident with dropped /mem req, not for IO (post rev A?)
//
// Circuit is a straight decode of the IO address (lower bits) ORed with /IOREQ (question should M1 high be included for interrupt reasons?????)
//
// This circuit and code does not work with the 128, circuit will maybe be the same, might require some interrupt stuff from FDC emulation
// or tweak for 6MHz
//
//
// IMPORTANT - This is example code, it works for me, on a breadboarded wire mess at present. Getting things wrong will do bad things to your Lynx. 
//             Testing signals with a scope before hooking things up is strongly advised.
//


// ------------------ INCLUDES  ------------------------
#include "soc/timer_group_struct.h"  // for feeding wdt
#include "soc/timer_group_reg.h"
#include "freertos/FreeRTOS.h"

#include "PALEDISK.H"


// ------------------ DEFINES  ------------------------
// ------------------ DEFINES  ------------------------

#define NOP() asm volatile ("nop")

#define DISPLAY_SERIAL_DEBUG_INITITALISATION
#define DISPLAY_SERIAL_DEBUG_REALTIME
//#define DISPLAY_SERIAL_DEBUG_ONCHANGE

//#define USE_WIFI   // needs psram for the disk buffers
#define USE_CORE0   // switching this off doesnt work at mo 

#ifdef USE_WIFI
      #include <WiFi.h>
      #include <WiFiClient.h>
      #include <WiFiAP.h>

      // Set these to your desired credentials.
      const char *ssid = "yourAP";
      const char *password = "yourPassword";
      
      WiFiServer server(80);
#endif

#define USE_SDCARD
//#define USE_SPIFFS

#ifdef USE_SDCARD
    #define HSPI_MISO   27
    #define HSPI_MOSI   26
    #define HSPI_SCLK   25
    #define HSPI_SS     33
    #include <SD.h>
    #include <SPI.h>
    #include "FS.h"
    static fs::FS* fileSystem;
    SPIClass *spi = NULL;
#endif

#ifdef USE_SPIFFS
    #include "SPIFFS.h"
#endif

// ------------------ PIN DEFINITIONS  ------------------------


#define PIN_ADRESSED 39  
#define PIN_ADRESSED_BIT BIT7  //of Upper pins  i.e. pin 39  
#define PIN_RD 2
#define PIN_RD_BIT BIT2
#define OUTPUT_PIN_CLEARWAIT 5 
#define OUTPUT_PIN_CLEARWAIT_BIT BIT5  
#define OUTPUT_PIN_BUSREQ 4 
#define OUTPUT_PIN_BUSREQ_BIT BIT4  

#define OUTPUT_PIN_DROM_ENABLE 21
#define OUTPUT_PIN_DROM_ENABLE_BIT BIT21

#define PIN_RESET 23
#define PIN_RESET_BIT BIT23

#define PIN_A0 34
#define PIN_A1 35
#define PIN_A2 36
#define PIN_A3 22
#define PIN_A3_BIT BIT22

#define MREQ_MASK PIN_ROM_ADRESSED_BIT 
#define IOREQ_MASK PIN_ADRESSED_BIT 
#define WRITE_MASK PIN_RD_BIT     // determine by write mask is RD bit being HIGH

#define DBUS 12   //  two-way databus is at pins 12 - 19



// ------------------ EXTERNS  ------------------------
// ------------------ EXTERNS  ------------------------




//
// ----------------------------------------------------------------------------------------
// ======================        GLOBALS       ============================================
// ======================        GLOBALS       ============================================
// ======================        GLOBALS       ============================================
// ======================        GLOBALS       ============================================
// ----------------------------------------------------------------------------------------

volatile unsigned int statusin = 0;
volatile byte address = 0;
volatile byte addressA3 = 0;
volatile unsigned int fullin = 0;
volatile unsigned int fullinHIGHpins = 0;
volatile unsigned int fullin_pre = 0;

volatile byte in_reset = 0;
volatile byte datain = 0;
volatile byte disk_rom_enable = 0;


portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

volatile byte recdata[8];   // io debug
volatile byte mrecdata[8]; // memory debug



const char* fmtMemCk = "Free: %d\tMaxAlloc: %d\t PSFree: %d\n";
#define MEMCK Serial.printf(fmtMemCk,ESP.getFreeHeap(),ESP.getMaxAllocHeap(),ESP.getFreePsram())


volatile unsigned char *dbuff_Array[DISKBUFFER_NOOF_SEGS]; // array of buffers to make one big 204800 buffer
                                                           // fragmented so esp can handle it on the heap
                                                           
int current_diskno = 1;
int dump_diskbuff_to_sdcard = 0;

volatile unsigned int df_ptr = 0;


// ------------------ Spiffs  ------------------------
// ------------------ Spiffs  ------------------------
// ------------------ Spiffs  ------------------------
// If using SPIFFS internal storage ...
#ifdef USE_SPIFFS

void readFile_spiffs(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    df_ptr = 0;
    while(file.available())
    {
        write_to_disk(df_ptr, file.read());
        df_ptr++;
    }
    Serial.print("Read bytes - ");
    Serial.println(df_ptr);
    df_ptr = 0;
}
#endif





// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------ UTILITY ROUTINES ------------------------
// ------------------ UTILITY ROUTINES ------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------


unsigned long IRAM_ATTR microsecs()
{
    return (unsigned long) (esp_timer_get_time());
}

void IRAM_ATTR delaythemicroseconds(uint32_t us)
{
    uint32_t m = microsecs();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(microsecs() > e){
                NOP();
            }
        }
        while(microsecs() < e){
            NOP();
        }
    }
}


void setup_initial_databus()    //IRAM attr not needed - slow stuff
{
  for (int i = 0; i < 8; i++) {
    pinMode(DBUS + i, INPUT);
  }
}

void IRAM_ATTR dbus_set_inputs(void) 
{
  REG_WRITE(GPIO_ENABLE_W1TC_REG, 0xFF << DBUS);
}

void IRAM_ATTR dbus_set_outputs(void)
{
  REG_WRITE(GPIO_ENABLE_W1TS_REG, 0xFF << DBUS);
}

uint8_t IRAM_ATTR dbus_read(void)
{
  uint32_t input = REG_READ(GPIO_IN_REG);
  return (input >> DBUS);
}

void IRAM_ATTR dbus_write(uint8_t value)
{
  uint32_t output =
    (REG_READ(GPIO_OUT_REG) & ~(0xFF << DBUS)) | (((uint32_t)value) << DBUS);
  REG_WRITE(GPIO_OUT_REG, output);
}


void IRAM_ATTR feedTheDog0(){
  // feed dog 0
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG0.wdt_feed=1;                       // feed dog
  TIMERG0.wdt_wprotect=0;                   // write protect
  // feed dog 1
//  TIMERG1.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
//  TIMERG1.wdt_feed=1;                       // feed dog
//  TIMERG1.wdt_wprotect=0;                   // write protect
}

// ------------------------------------------------------------------------------------
// --------------------- TIME CRITICAL RESPONSE ROUTINES ------------------------------
// --------------------- TIME CRITICAL RESPONSE ROUTINES ------------------------------
// --------------------- TIME CRITICAL RESPONSE ROUTINES ------------------------------
// --------------------- TIME CRITICAL RESPONSE ROUTINES ------------------------------
// --------------------- TIME CRITICAL RESPONSE ROUTINES ------------------------------
// --------------------- TIME CRITICAL RESPONSE ROUTINES ------------------------------
// --------------------- TIME CRITICAL RESPONSE ROUTINES ------------------------------
// --------------------- TIME CRITICAL RESPONSE ROUTINES ------------------------------
// ------------------------------------------------------------------------------------


void IRAM_ATTR do_clearwait()
{
  taskENTER_CRITICAL(&myMutex);  // ? does this help with just low pulse width but not the overall (extra pulse width) problem
            GPIO.out_w1tc = OUTPUT_PIN_CLEARWAIT_BIT;   // note polaity active low
            
            // Delay here needed -  too short and the loop goes round too quickly and sees IOREQ still low - hence doing a second write
            // too long and we intrude into any future write cycle occuring, ideally should be just about time wait ends            
            for(int f = 0; f < 5; f++)
            {
                //    delaythemicroseconds(1);  // this creates extra 2us jumps periodically?
                //                NOP();
               feedTheDog0();
               NOP();
            }  
            GPIO.out_w1ts = OUTPUT_PIN_CLEARWAIT_BIT;
taskEXIT_CRITICAL(&myMutex);

}

void IRAM_ATTR do_fdc_emu()
{     
      
        //globals used for speed
        if(address == 8)          // emulate the Lynx floppy control port, just one bit of it controls whether the LynxDOS ROM is enabled.
                                  // at boot the ROM is enabled then during boot the initialisation at E000 is called, the ROM is then disabled
                                  // to allow use of the RAM underneath
        {
              disk_rom_enable = (datain & 0x10) >> 4;
              if(disk_rom_enable == 1)  // 0 = enabled
                 GPIO.out_w1ts = OUTPUT_PIN_DROM_ENABLE_BIT;  // note  active low output
              else
                 GPIO.out_w1tc = OUTPUT_PIN_DROM_ENABLE_BIT;    // note  active low output
        }
        else
        {
              disk_outp(0x50+address, datain);
        }

}

                
// =============================================================================================
// ============================== MAIN CoreX Response ROUTINE for emulating IO ports ===========
// =============================================================================================

void IRAM_ATTR emuTask( void * parameter )
{
    byte recdata_ptr = 0;
    byte mrecdata_ptr = 0;
    while(1)
    {
         feedTheDog0();
         fullinHIGHpins = REG_READ(GPIO_IN1_REG);   // use GPIO_IN1_REG for pins over 31
         fullin = REG_READ(GPIO_IN_REG);  // lower pins 0 - 31


         // Test for RESET low here - if so then reset the DOS rom to ENABLED (Lynx will see this on boot, run
         //                           the init DOSROM code which will then disable the ROM shadow - 38556 bytes free
         if((fullin & (uint32_t)PIN_RESET_BIT) == 0)
         {
                 in_reset = 1;
                 disk_rom_enable = 0;
                 GPIO.out_w1tc = OUTPUT_PIN_DROM_ENABLE_BIT;    // note  active low output
         }
         else
                in_reset = 0;         

                  
         // IO REQUESTS ============================================================================================
         // IO REQUESTS ============================================================================================
         // IO REQUESTS ============================================================================================
         if((fullinHIGHpins & (uint32_t)IOREQ_MASK)  == (uint32_t)IOREQ_MASK) 
         {
            statusin = fullinHIGHpins ; // so that its captured only when IO request is done
            address = ((statusin & 0x1c) >> 2) | ((fullin & (uint32_t)PIN_A3_BIT) >> 19);
           
            if((fullin & (uint32_t)WRITE_MASK) == (uint32_t)WRITE_MASK)   // Receive data from Z80 writes
            {
                datain = fullin >> 12; 
                recdata[recdata_ptr++] = datain;
                if (recdata_ptr > 7)
                    recdata_ptr = 0;

                
                do_fdc_emu();
     
                do_clearwait();     
                
                if(address == 0xa)      // resets the data pointer on an emulated (serial byte) tape drive
                    df_ptr = 0;
                if(address == 0xb)      // changes the current disk image (results in new image being loaded from other core)
                    current_diskno = datain & 0x0F;
                if(address == 0xc)      // tell other core to dump the disk buffer back to sdcard
                    dump_diskbuff_to_sdcard = 1;
 }
            else  // we assume it was a read
            {   
                // Send data to Z80 input requests
                // Put data onto the z80 bus by switching to output mode on the bus pins
                // and put the requisite data on
         
                if(address == 9)          // read from &59 - an emulated serial tape drive
                {
                      dbus_write(dbuff_Array[0][df_ptr]);
                      df_ptr++;
                      if(df_ptr > 4090)
                        df_ptr = 0;
                }
                else
                {
                      dbus_write(disk_inp(0x50+address));
                }
                
                // Change ESP Data Bus pins to output mode
                dbus_set_outputs();
                //set z80 BUSREQ low
                GPIO.out_w1tc = OUTPUT_PIN_BUSREQ_BIT;    //lower busreq

                // Clear the WAIT condition
                do_clearwait();  // this will allow IOREQ to go HIGH latching in the data
                                 
                // I should wait here for ioreq to raise but no time for polling (maybe)
                // wait for ioreq (addressed) to raise
                //                while( (REG_READ(GPIO_IN1_REG) & (uint32_t)IOREQ_MASK)  == 0)
                //                {
                //                     feedTheDog0();
                //                     NOP();                
                //                } 
                                                
                 //set BUSREQ high
                GPIO.out_w1ts = OUTPUT_PIN_BUSREQ_BIT;    //set busreq high
          
                // Switch data bus mode back to input Hi-Z
                dbus_set_inputs();
               
                // yes wrong order ... if I had but the time... 
             }
         }
    }      
}

#ifdef USE_SPIFFS
void open_working_disk_spiffs(int x)
{
      File  lhandle;
      char lbl[30];
      sprintf(lbl,"/JD%d.LDF",x);
      readFile_spiffs(SPIFFS, lbl);
}
#endif

void alloc_disk_buffer()
{
    Serial.println("Allocating large disk buffer ARRAY");
    for(int f = 0; f < DISKBUFFER_NOOF_SEGS; f++)
    {
      MEMCK;
      dbuff_Array[f] = (volatile unsigned char *)calloc(DISKBUFFER_SEGMENT_SIZE,1);
    }
}


void  IRAM_ATTR setup()
{
   Serial.begin(115200);

  pinMode(OUTPUT_PIN_CLEARWAIT, OUTPUT);
  GPIO.out_w1ts = OUTPUT_PIN_CLEARWAIT_BIT;  
  pinMode(OUTPUT_PIN_BUSREQ, OUTPUT);  // z80 /BUSREQ  
  GPIO.out_w1ts = OUTPUT_PIN_BUSREQ_BIT;  

  pinMode(OUTPUT_PIN_DROM_ENABLE, OUTPUT);  
  GPIO.out_w1tc = OUTPUT_PIN_DROM_ENABLE_BIT;  // clear means DROM is enabled at boot
  pinMode(PIN_RESET, INPUT);  

  pinMode(PIN_ADRESSED, INPUT);   //39 always input anyway
 
  pinMode(PIN_RD, INPUT);  // z80 /RD  (and if not rd then assumed a write?)
  
#ifdef DISPLAY_SERIAL_DEBUG_INITIALISATION
    delay(2000);
    Serial.println("WELCOME   ");
    Serial.println("WELCOME   ");
    Serial.println("WELCOME   ");
    Serial.println("WELCOME   ");
    Serial.println("WELCOME   ");
    Serial.println("WELCOME   ");
#endif
  
  setup_initial_databus();

  pinMode(PIN_A0,INPUT);  //34 is always input anyway etc.
  pinMode(PIN_A1,INPUT);
  pinMode(PIN_A2,INPUT);
  pinMode(PIN_A3,INPUT);



#ifdef USE_SDCARD 
 
    pinMode(HSPI_SS, OUTPUT); // SS
    spi = new SPIClass(HSPI);
    spi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
    delay(100);
    SD.begin(HSPI_SS, *spi);
    
    fileSystem = &SD;
    delay(100);
    
    uint8_t cardType = SD.cardType();
    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }
    // open_working_disk_sdcard(current_diskno);
    
#endif



  
 Serial.print("Allocating big buffer  Free HEAP is ");
 Serial.println(ESP.getFreeHeap());
 alloc_disk_buffer();
 Serial.print("After allocation  Free HEAP is ");
 Serial.println(ESP.getFreeHeap());


#ifdef USE_SDCARD 
  open_working_disk_sdcard(current_diskno);
#endif

  
//  Serial.println("Verifying sdcard against disk buffer..");
//  verify_working_disk_sdcard(current_diskno);

#ifdef USE_SPIFFS
  if(!SPIFFS.begin(false)){
      Serial.println("SPIFFS Mount Failed");
      return;
  }
  open_working_disk_spiffs(current_diskno);
  //  readFile(SPIFFS, "/YNXVADERS.tap"); // uncomment if you want to test 'serial byte' tape upload - not implemented yet
  //  readFile(SPIFFS, "/vaders.bin");    // implemented stream without "header" bytes
 
#endif

Serial.println("Initialising disks and the 6k buffers");
  init_disks(); // allocates ?6k for track/sector buffers

  xTaskCreatePinnedToCore(
                    emuTask,   /* Function to implement the task */
                    "emuTask", /* Name of the task */
                    2048,      /* Stack size in words ,   I have no idea how much it needs , 128 seems to work, I am an ignoramus...   */
                    NULL,       /* Task input parameter */
                    20,          /* Priority of the task */
                    NULL,       /* Task handle. */
#ifdef USE_CORE0 
                    0);  /* Core where the task should run */
#else
                    1);  /* Core where the task should run */
#endif


#ifdef USE_WIFI
  Serial.println("Configuring access point...");

  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();

  Serial.println("Server started");
#endif


  
Serial.println("SETUP COMPLETE");
Serial.println("   APPLY Z80 NOW");
Serial.println("   APPLY Z80 NOW");
Serial.println("   APPLY Z80 NOW");
Serial.println(" ");
Serial.println(" ");

}


void  IRAM_ATTR loop() 
{
      static byte  last_diskno = current_diskno;


#ifdef USE_WIFI
 WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to turn ON the LED.<br>");
            client.print("Click <a href=\"/L\">here</a> to turn OFF the LED.<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

//        // Check to see if the client request was "GET /H" or "GET /L":
//        if (currentLine.endsWith("GET /H")) {
//          digitalWrite(LED_BUILTIN, HIGH);               // GET /H turns the LED on
//        }
//        if (currentLine.endsWith("GET /L")) {
//          digitalWrite(LED_BUILTIN, LOW);                // GET /L turns the LED off
//        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
#endif

      if(dump_diskbuff_to_sdcard == 1)
      {
              Serial.println("Doing disk buffer dump");
              save_working_disk_sdcard();
              dump_diskbuff_to_sdcard = 0;
      }


      //  if (Serial.available() > 0) {
      //    // get incoming byte:
      //    byte inByte = Serial.read();
      //    if(inByte == 'e')
      //          disk_rom_enable = 0;
      //    if(inByte == 'd')
      //          disk_rom_enable = 1;
      //}
       // Test if disk has been changed
       if(current_diskno != last_diskno)
       {
#ifdef USE_SPIFFS        
           open_working_disk_spiffs(current_diskno);
#endif
#ifdef USE_SDCARD        
           open_working_disk_sdcard(current_diskno);
#endif
       }
       last_diskno = current_diskno;

#ifdef DISPLAY_SERIAL_DEBUG_REALTIME
               Serial.print("Address: ");
               Serial.print(address);  // (statusin & 0x1c) >> 2  use statusin so it only captured on request  -  pins 34-36 are A0 - 2
               //Serial.print(statusin ,BIN);  // pins
               Serial.print(" Data: ");
               Serial.print(datain, HEX);
               Serial.print(" | DROMEN: ");
               Serial.print(disk_rom_enable);
               Serial.print(" INRESET: ");
               Serial.print(in_reset,DEC);
              //Serial.print(fullinHIGHpins, BIN);
               //Serial.print(fullin, BIN);
               //Serial.print(" ");
        //       Serial.print(" recdata: ");
        //       for(int f=0;f<8;f++)
        //       {
        //          Serial.write(recdata[f]);
        //       }
               Serial.print("     | DiskNo: ");
               Serial.print(current_diskno);
        
        
              Serial.print(" Track: ");
              Serial.print(disk_trackreg, DEC);
              Serial.print(" Sector: ");
              Serial.print(disk_sectreg, DEC);
        //      Serial.print(" Command: ");
        //      Serial.print(disk_comreg, HEX);
        //      Serial.print(" Status: ");
        //      Serial.print(disk_statusreg, HEX);
              Serial.println(" ");
 #endif
                               
              delay(100);
       
} 
