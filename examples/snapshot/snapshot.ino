#include "SoftwareSerial.h"
#include <VC0706.h>
#include <SD.h>

#define SS  10

SoftwareSerial cameraconnection(2,3);//Rx, Tx
VC0706 cam = VC0706(&cameraconnection);

bool camera_status = false;

void setup() 
{
    Serial.begin(38400);
    Serial.println("VC0706 Camera Snapshot Test ...");
  
    // see if the card is present and can be initialized:
    if (!SD.begin(SS)) {
        Serial.println("Card failed, or not present");
        return;
    }  
    camera_status = cameraInit();
    if(true != camera_status){
        Serial.println("camera init error!");
    }else{
        snapShot();
    }    
}

void loop() 
{
    //nothing to do
}

bool cameraInit()
{
    // Try to locate the camera
    if (cam.begin(38400)) {
        Serial.println("Camera Found.");
    } else {
        Serial.println("No camera found.");
        return false;
    }

    // Print out the camera version information (optional)
    char *reply = cam.getVersion();
    if (reply == 0) {
        Serial.print("Failed to get version");
        return false;
    } else {
        Serial.println("version:");
        Serial.println("-----------------");
        Serial.println(reply);
        Serial.println("-----------------");
    }
    delay(5000);
    return true;
}

void snapShot()
{
#if 0
    // Set the picture size - you can choose one of 640x480, 320x240 or 160x120 
    // Remember that bigger pictures take longer to transmit!
    cam.setImageSize(VC0706_640x480);   // biggest
    //cam.setImageSize(VC0706_320x240); // medium
    //cam.setImageSize(VC0706_160x120); // small

    uint8_t imgsize = cam.getImageSize();
    Serial.print("Image size: ");
    if (imgsize == VC0706_640x480) Serial.println("640x480");
    if (imgsize == VC0706_320x240) Serial.println("320x240");
    if (imgsize == VC0706_160x120) Serial.println("160x120");
#endif

    Serial.println("Snap in 3 secs...");
    delay(3000);

    if (! cam.takePicture()){ 
        Serial.println("Failed to snap!");
    }else { 
        Serial.println("Picture taken!");
    }
    // Create an image with the name IMAGExx.JPG
    char filename[13];
    strcpy(filename, "IMAGE00.JPG");
    for (int i = 0; i < 100; i++) {
        filename[5] = '0' + i/10;
        filename[6] = '0' + i%10;
        // create if does not exist, do not open existing, write, sync after write
        if (! SD.exists(filename)) {
            break;
        }
    }
  
    // Open the file for writing
    File imgFile = SD.open(filename, FILE_WRITE);
    Serial.print("create ");
    Serial.println(filename);
    // Get the size of the image (frame) taken  
    uint16_t jpglen = cam.frameLength();
    Serial.print("wait to fetch ");
    Serial.print(jpglen, DEC);
    Serial.println(" byte image ...");

    int32_t time = millis();

    // Read all the data up to # bytes!
    while (jpglen != 0) {
        // read 32 bytes at a time;
        uint8_t *buffer;
        uint8_t bytesToRead = min(32, jpglen);   // change 32 to 64 for a speedup but may not work with all setups!
        buffer = cam.readPicture(bytesToRead);
        imgFile.write(buffer, bytesToRead);
        //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
        jpglen -= bytesToRead;
    }
    imgFile.close();
  
    time = millis() - time;
    Serial.println("Done!");
    Serial.print("Took "); Serial.print(time); Serial.println(" ms");
    cam.resumeVideo();    
}

void debug()
{
    while(1){
        while(cameraconnection.available()){
	    Serial.write(cameraconnection.read());
        }
        while(Serial.available()){     
	    cameraconnection.write(Serial.read()); 
        }
    }
}

