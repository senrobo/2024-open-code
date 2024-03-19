#include <Arduino.h>

#include "SPI.h"
#include <PacketSerial.h>
#include <Wire.h>
#include <iostream>
#include <math.h>

#define SERIAL_PORT_USBVIRTUAL Serial
#define SERIAL_PORT_MONITOR    Serial
#define SAMD21
using namespace std;

#ifdef SAMD21
    #define S0           9
    #define S1           8
    #define S2           1
    #define S3           2
    #define MuxInput1    3
    #define MuxInput2    4
    #define MuxInput3    5
    #define Solenoid_Pin 0
#endif

#ifdef ESP32
    #define S0           9
    #define S1           8
    #define S2           4
    #define S3           5
    #define MuxInput1    1
    #define MuxInput2    2
    #define MuxInput3    3
    #define Solenoid_Pin 0
#endif

#define LDRPINCOUNT 36
#define RadiusofLDR 1.0F

PacketSerial L1Serial;

int LDRPINMap[LDRPINCOUNT]{0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,
                           12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 24,
                           25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};

float LDRBearings[LDRPINCOUNT]{
    0.00,   10.00,  20.00,  30.00,  40.00,  50.00,  60.00,  70.00,
    80.00,  90.00,  100.00, 110.00, 120.00, 130.00, 140.00, 150.00,
    160.00, 170.00, 180.00, 190.00, 200.00, 210.00, 220.00, 230.00,
    240.00, 250.00, 260.00, 270.00, 280.00, 290.00, 300.00, 310.00,
    320.00, 330.00, 340.00, 350.00}; // assuming placement of ldrs is constant

float LDRThresholds[LDRPINCOUNT]{
    870.00, 873.50, 849.00, 862.00, 860.00, 850.00, 848.00, 857.50, 859.00,
    849.00, 847.00, 856.50, 861.50, 860.50, 868.00, 865.00, 867.00, 879.00,
    872.50, 863.00, 861.50, 869.00, 860.00, 871.50, 881.50, 879.50, 855.50,
    876.50, 866.00, 859.50, 868.50, 951.00, 867.00, 873.00, 865.50, 856.50};
// 26,27,29,13
double maxRecordedValue[LDRPINCOUNT]{
    881, 882, 863, 870, 874, 864, 857, 871, 868, 862, 861, 873,
    866, 871, 879, 872, 870, 890, 881, 873, 869, 881, 869, 883,
    891, 891, 864, 887, 876, 869, 877, 952, 874, 880, 876, 866};

double minRecordedValue[LDRPINCOUNT]{
    859, 865, 835, 854, 846, 836, 839, 844, 850, 836, 833, 840,
    857, 850, 857, 858, 864, 868, 864, 853, 854, 857, 851, 860,
    872, 868, 847, 866, 856, 850, 860, 950, 860, 866, 855, 847};

double calculatedthesholdValue[LDRPINCOUNT]{
    834, 833, 818, 822, 828, 821, 826, 820, 5,   821, 832, 822,
    830, 832, 825, 841, 825, 832, 838, 834, 829, 841, 843, 840,
    850, 846, 848, 843, 836, 840, 830, 831, 828, 844, 823, 827};

int RAWLDRVALUES[LDRPINCOUNT];
int kickpower = 0;

struct Solenoid {
    int kick;
};

struct L2toL1Payload {
    Solenoid solenoid;
};
Solenoid solenoid;

void receiveL1TxData(const byte *buf, size_t size) {
    L2toL1Payload payload;
    memcpy(&payload, buf, sizeof(payload));
    kickpower = payload.solenoid.kick;
    return;
}

struct linedata {
    int onLine = 1;
    float angleBisector = 0;
    float depthinLine = 0;
    int linetrackldr1 = 0;
    int linetrackldr2 = 0;
    int ballinCatchment = 1;
    // int SerialLDRID = 0;
    // int SerialLDRvalue = 0;
};
linedata SAMDlinedata;

typedef struct l1TxPayload {
    linedata SAMDlinedata;
} l1TxPayload;

void selectMUXChannel(uint8_t channel) {
    digitalWrite(S0, channel & 1);
    digitalWrite(S1, (channel >> 1) & 1);
    digitalWrite(S2, (channel >> 2) & 1);
    digitalWrite(S3, (channel >> 3) & 1);
}
int readMUXChannel(int index, int muxInput) {
    selectMUXChannel(index);
    return analogRead(muxInput);
}

void getValues() {
    // values from
    // https://docs.google.com/spreadsheets/d/14BAutKBhzy3ZVvEh6iIaexv4C6UqhUbg-NXU7_EWB44/edit#gid=0

    RAWLDRVALUES[0] = readMUXChannel(2, MuxInput2);
    RAWLDRVALUES[1] = readMUXChannel(1, MuxInput2);
    RAWLDRVALUES[2] = readMUXChannel(0, MuxInput2);
    RAWLDRVALUES[3] = readMUXChannel(7, MuxInput1);
    RAWLDRVALUES[4] = readMUXChannel(6, MuxInput1);
    RAWLDRVALUES[5] = readMUXChannel(5, MuxInput1);
    RAWLDRVALUES[6] = readMUXChannel(4, MuxInput1);
    RAWLDRVALUES[7] = readMUXChannel(3, MuxInput1);
    RAWLDRVALUES[8] = readMUXChannel(2, MuxInput1);
    RAWLDRVALUES[9] = readMUXChannel(1, MuxInput1);
    RAWLDRVALUES[10] = readMUXChannel(0, MuxInput1);
    RAWLDRVALUES[11] = readMUXChannel(15, MuxInput1);
    RAWLDRVALUES[12] = readMUXChannel(14, MuxInput1);
    RAWLDRVALUES[13] = readMUXChannel(13, MuxInput1);
    RAWLDRVALUES[14] = readMUXChannel(12, MuxInput1);
    RAWLDRVALUES[15] = readMUXChannel(11, MuxInput1);
    RAWLDRVALUES[16] = readMUXChannel(15, MuxInput2);
    RAWLDRVALUES[17] = readMUXChannel(14, MuxInput2);
    RAWLDRVALUES[18] = readMUXChannel(13, MuxInput2);
    RAWLDRVALUES[19] = readMUXChannel(12, MuxInput2);
    RAWLDRVALUES[20] = readMUXChannel(11, MuxInput2);
    RAWLDRVALUES[21] = readMUXChannel(12, MuxInput3);
    RAWLDRVALUES[22] = readMUXChannel(11, MuxInput3);
    RAWLDRVALUES[23] = readMUXChannel(10, MuxInput3);
    RAWLDRVALUES[24] = readMUXChannel(9, MuxInput3);
    RAWLDRVALUES[25] = readMUXChannel(8, MuxInput3);
    RAWLDRVALUES[26] = readMUXChannel(7, MuxInput3);
    RAWLDRVALUES[27] = readMUXChannel(6, MuxInput3);
    RAWLDRVALUES[28] = readMUXChannel(5, MuxInput3);
    RAWLDRVALUES[29] = readMUXChannel(4, MuxInput3);
    RAWLDRVALUES[30] = readMUXChannel(3, MuxInput3);
    RAWLDRVALUES[31] = readMUXChannel(2, MuxInput3);
    RAWLDRVALUES[32] = readMUXChannel(1, MuxInput3);
    RAWLDRVALUES[33] = readMUXChannel(0, MuxInput3);
    RAWLDRVALUES[34] = readMUXChannel(4, MuxInput2);
    RAWLDRVALUES[35] = readMUXChannel(3, MuxInput2);
    // ball catchment ldr
    SAMDlinedata.ballinCatchment = readMUXChannel(5, MuxInput2);
}
double highValues[LDRPINCOUNT] = {
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00};

void findLine() {
    getValues();
    // initialize essential variables
    int first_tmpldrangle = 0;
    int first_ldrPinout = 0;
    int second_tmpldrangle = 0;
    int second_ldrPinout = 0;
    double tmpanglediff = 0;
    double largestanglediff = 0;
    double tmprobotangle = 0;

    int final_ldrPinout1 = 0;
    int final_ldrPinout2 = 0;
    SAMDlinedata.onLine = 1;

    for (int pinNumber = 0; pinNumber < LDRPINCOUNT; pinNumber++) {
#ifdef DEBUG
        const auto printSerial = [](int value) { Serial.printf("%3d", value); };
        if (pinNumber == 35) {
            printSerial(calculatedthesholdValue[pinNumber]);
            Serial.println(" ");
        } else {
            printSerial(calculatedthesholdValue[pinNumber]);
            Serial.print(" , ");
        }
#endif

        for (int i = 0; i < LDRPINCOUNT; i++) {
            if (highValues[i] < RAWLDRVALUES[i]) {
                highValues[i] = RAWLDRVALUES[i];
            }

            if (RAWLDRVALUES[pinNumber] > LDRThresholds[pinNumber]) {
                SAMDlinedata.onLine = 2;
                first_ldrPinout = pinNumber;
                first_tmpldrangle = LDRBearings[pinNumber];

                calculatedthesholdValue[i] =
                    minRecordedValue[i] +
                    (maxRecordedValue[i] - minRecordedValue[i]) * 0.5;
                if (RAWLDRVALUES[i] > LDRThresholds[i]) {
                    second_ldrPinout = i;
                    second_tmpldrangle = LDRBearings[i];
                    tmpanglediff = abs(LDRBearings[second_ldrPinout] -
                                       LDRBearings[first_ldrPinout]);

                    if (tmpanglediff > 180) tmpanglediff = 360 - tmpanglediff;
                    if (tmpanglediff > largestanglediff) {
                        //(first_tmpldrangle > 180) ? tmprobotangle = 360 -
                        // first_tmpldrangle : first_tmpldrangle;
                        largestanglediff = tmpanglediff;
                        final_ldrPinout1 = first_ldrPinout;
                        final_ldrPinout2 = second_ldrPinout;
                    }
                }
            }
        }

        SAMDlinedata.linetrackldr1 = final_ldrPinout1;
        SAMDlinedata.linetrackldr2 = final_ldrPinout2;
        if (largestanglediff > 180) largestanglediff = 360 - largestanglediff;
        if (abs(LDRBearings[final_ldrPinout2] -
                LDRBearings[final_ldrPinout1]) <= 180) {
            SAMDlinedata.angleBisector =
                LDRBearings[final_ldrPinout1] + largestanglediff / 2;
        } else {
            SAMDlinedata.angleBisector =
                LDRBearings[final_ldrPinout2] + largestanglediff / 2;
        }
        SAMDlinedata.depthinLine =
            1.0 - cosf((largestanglediff / 2.0) / 180.0 * PI);
        //(1.0 - (cosf((90/2)/180 * PI) / 1.0));
    }
}

void printHighestValue(double thresholdValuePercentage) {
    // for (int j = LDRPINCOUNT; j < LDRPINCOUNT; j++){
    //   const auto printSerial = [](int value){
    //     Serial.printf("%3d",value);
    //   };
    //   if (j == 35) {
    //     printSerial(highValues[j]);
    //     Serial.println(" ");
    //   }
    //   else {
    //     printSerial(highValues[j]);
    //     Serial.print(" , ");
    //   }
    // }
    getValues();
}

void setup() {

    Serial1.begin(115200);
    Serial.begin(9600);

    L1Serial.setStream(&Serial1);
    L1Serial.setPacketHandler(&receiveL1TxData);
    analogWriteResolution(10);
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(MuxInput1, INPUT);
    pinMode(MuxInput2, INPUT);
    pinMode(MuxInput3, INPUT);

    pinMode(Solenoid_Pin, OUTPUT);
    // Serial1.begin(9600);
    // autoTuneThreshold(10000, 0.7);
}
int counter = 0;
int dontKillSamd21 = 1;
void loop() {
    // counter > 35 ? counter = 0 : counter = counter;
    // SAMDlinedata.linetrackldr1 = highValues[counter];
    // SAMDlinedata.linetrackldr2 = counter;

    L1Serial.update();
    // Serial.println(kickpower);
    if (kickpower == 1024) {
        digitalWrite(Solenoid_Pin, HIGH);
    } else {
        digitalWrite(Solenoid_Pin, LOW);
    }

    SAMDlinedata.onLine = 1;
    findLine();

    // printHighestValue(1);

    // autoTuneThreshold(5000, 0.7);
    Serial.print(SAMDlinedata.ballinCatchment);
    Serial.print(" , ");
    Serial.println(SAMDlinedata.onLine);

    // Serial.print("hei");

    byte buf[sizeof(l1TxPayload)];
    memcpy(buf, &SAMDlinedata, sizeof(SAMDlinedata));
    L1Serial.send(buf, sizeof(buf));
    // Serial1.write(2);

    counter++;
}
