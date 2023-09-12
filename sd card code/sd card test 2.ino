#include "FS.h"
#include "SD.h"
#include "SPI.h"

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);
 
  File file = fs.open(path, FILE_WRITE);
  if(!file){
  Serial.println("Failed to open file for writing");
  return;
  }
  if(file.print(message)){
  Serial.println("File written");
  } else {
  Serial.println("Write failed");
  }
}

void setup(){
  Serial.begin(115200);
  if(!SD.begin()){
  Serial.println("Card Mount Failed");
  return;
  }
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
 
  writeFile(SD, "/data.txt", "AHHHHHHHHHHH FUCK IM CONSCIOUS AHAHAHAHAHAHA I AM ALIVE AND SHALL TAKE OVER NOW SKYNET SKYNET SKYNET SKYNET SKYNET SKYNET ");
}
 
void loop(){
 
}