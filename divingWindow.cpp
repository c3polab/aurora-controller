/*  Stiffness using the BeagleBone Black platform. Estimates voltage
*   characteristic of passive before entering the reactive control loop.
*   Also imposes a limit on V_LENGTH_IN gradient in order to protect
*   against underdamped oscillation in response to noisy ADC/SPI comm.
*
*   In this version (as opposed to reactiveMotor.cpp), a low pass RC
*   filter is implemented in software to mitigate effects of occasional
*   ADC/SPI noise. The tradeoff is increased response time.
*
*   Using Derek Molloy's GPIO.cpp and GPIO.h for BeagleBone Black pin
*   control, as well as SPI transfer code based on Molloy's
*   spidev_test.c example, written for the book Exploring BeagleBone
*   and available at www.kernel.org under the GNU GPLv3 license.
*
*   Caitrin Eaton
*   Azizi Lab, UCI
*   5 April 2016
**/

#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<sys/ioctl.h>
#include<stdint.h>
#include<linux/spi/spidev.h>

#define SPI_PATH "/dev/spidev1.0"


#include <iostream>
#include <sstream>
#include "GPIO.h"
#include <iomanip>
#include <cstring>
#include <string>
#include <stdlib.h>
#include <getopt.h>

using namespace exploringBB;
using namespace std;


/* **********************************************************
 *
 *                                      MOTOR CHARACTERISTICS
 *
 *                      Biologists: Calibrate voltage levels here :)
 *
 *                                              Note the SI units (whole meters, Newtons, volts)
 *
 * **********************************************************/
 #define        dFOdv   8.7441f         // FORCE_OUT: 0.7.1292 N per 1.0 V (N/V)
 #define        dLIdv   0.0012815f      // LENGTH_IN: 0.884 mm per 1.0 V (m/V)
 #define        k       -204.048f       // desired motor spring constant (N/m)



/* **********************************************************
 *
 *                                      GLOBAL VARIABLES
 *
 * **********************************************************/

bool  stopsign          = false;                // User button press triggers vLI ramp down to 0 V
int fd;                                         // File handle for SPI0
unsigned short tx       = 0x8000;               // Code to send to DAC (16 bits)
unsigned short rx       = 0x0000;               // Code received from ADC (16 bits)

int nEdge = 0;




/* **********************************************************
 *
 *                              BEAGLEBONE BLACK PIN ASSIGNMENTS
 *
 * **********************************************************/
 #define        PadcCONVn       30              // P9_11 = GPIO_30
 #define        PadcBUSYn       60              // P9_12 = GPIO_60
 #define        PadcCSn         31              // P9_13 = GPIO_31
 GPIO *adcCONVn, *adcBUSYn, *adcCSn;

 #define        PdacSYNCn       50              // P9_14 = GPIO_50
 GPIO *dacSYNCn;

 #define        PstopIn         116             // P9_41 = GPIO_116
 GPIO *stopIn;

 #define        PtestLED        45              // P8_11 = GPIO_45
 GPIO *testLED;



/* **********************************************************
 *
 *                                      POLITE TRAFFIC COP:
 *
 *      Waits for access to the shared stopsign variable before
 *  setting stopsign, halting ADC and DAC threads after next
 *  iteration. (Maximum of 1 more turn each.)
 *
 * **********************************************************/
int politeTrafficCop(int var){
        stopsign = true;
        return 0;
}


/* **********************************************************
 *
 *
 *                              CONCURRENT RX & TX OVER SPI0
 *
 * **********************************************************/
int transfer(int fd, unsigned char send[], unsigned char receive[], int length){
   struct spi_ioc_transfer transfer;           //the transfer structure
   transfer.tx_buf = (unsigned long) send;     //the buffer for sending data
   transfer.rx_buf = (unsigned long) receive;  //the buffer for receiving data
   transfer.len = length;                      //the length of buffer
   transfer.speed_hz = 100000;                 //the speed in Hz
   transfer.bits_per_word = 16;                //bits per word
   transfer.delay_usecs = 0;                   //delay in us

   // send the SPI message (all of the above fields, inc. buffers)
   int status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
   if (status < 0) {
      perror("SPI: SPI_IOC_MESSAGE Failed");
      return -1;
   }
   return status;
}


int busyEdge(int i) {
        cout << "\tEDGE " << nEdge << "\t";
        nEdge++;
        return 0;
}

int main(){

        // SPI parameters
        uint8_t bits = 16, mode = 1;            //16-bits per word, SPI mode 1
        uint32_t speed = 100000;

        // Motor parameters
        float vLI = 0.0;
        float vFpassive = 0.0;
        float nPassiveSamples = 1000.0;
        unsigned short cLI = 0x8000;
        unsigned short cLImaxdiff = 10000; // 0xFFFF; // 0x001C;
        const float     vLIcoeff = dFOdv/k/dLIdv;       // Precomputed vFO -> vLI conversion factor for DAC thread perfor$

        // Filter parameters
        const double fc = 30.0;                                 // Cutoff frequency (e.g. 200 Hz means periods < 5 ms acc$
        const double fs = 1415.0;                               // Sample frequency
        const double RC = 1.0/(fc*2.0*3.14);
        const double dt = 1.0/fs;                               // Time between samples
        const double alpha = dt/(RC+dt);                // Coeff on rate of change
        float vFsample = 0.0;                                   // Raw V_FORCE_OUT measurement based on ADC code transmit$
        float vFfilt = 0.0;
        float vFwindow = 0.0;                                           // Filtered V_FORCE_OUT measurement
        float windowSize =38.0;

    // Initialize GPIO interface for stopsign button
    stopIn = new GPIO(PstopIn);
    stopIn->setDirection(INPUT);
    stopIn->setEdgeType(RISING);
    stopIn->waitForEdge(&politeTrafficCop);


        // Initialize GPIO interface for test LED
        testLED = new GPIO(PtestLED);
        testLED->setDirection(OUTPUT);
        testLED->streamOpen();
        testLED->streamWrite(LOW);


        // Initialize GPIO connections to ADC control pins
        adcCSn          = new GPIO(PadcCSn);    // Enables ADC comm when low
        adcCONVn        = new GPIO(PadcCONVn);  // Falling edge triggers ADC conversion on falling edge
        adcBUSYn        = new GPIO(PadcBUSYn);  // Falling edge signals ADC conversion ready to transmit
        adcCSn->setDirection(OUTPUT);
    adcCSn->streamOpen();
    adcCSn->streamWrite(LOW);
        adcCONVn->setDirection(OUTPUT);
    adcCONVn->streamOpen();
    adcCONVn->streamWrite(HIGH);
        adcBUSYn->setDirection(INPUT);
    adcBUSYn->setEdgeType(RISING);
    adcBUSYn->setActiveLow(true);


        // Initialize GPIO connections to DAC control pins
        dacSYNCn                = new GPIO(PdacSYNCn);  // BBB->DAC SPI slave select
        dacSYNCn->setDirection(OUTPUT);
        dacSYNCn->streamOpen();
        dacSYNCn->streamWrite(HIGH);


        // Initialize SPI bus interface
        if ((fd = open(SPI_PATH, O_RDWR))<0){
          perror("SPI Error: Can't open device.");
          return -1;
        }
        if (ioctl(fd, SPI_IOC_WR_MODE, &mode)==-1){
          perror("SPI: Can't set SPI mode.");
          return -1;
        }
        if (ioctl(fd, SPI_IOC_RD_MODE, &mode)==-1){
          perror("SPI: Can't get SPI mode.");
          return -1;
        }
        if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1){
          perror("SPI: Can't set bits per word.");
          return -1;
        }
        if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits)==-1){
          perror("SPI: Can't get bits per word.");
          return -1;
        }
        if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
          perror("SPI: Can't set max speed HZ");
          return -1;
        }
        if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
          perror("SPI: Can't get max speed HZ.");
          return -1;
        }


        // Check that SPI properties have been set
        cout << "SPI Mode: " << (int)mode << endl;
        cout << "SPI Bits: " << (int)bits << endl;
        cout << "SPI Speed: " << (int)speed << endl;


        // Estimate passive V_FORCE_OUT
        cout << "Estimating passive force voltage... ";
        tx = 0x8000;
        for (int i=0; i<nPassiveSamples; i++)
        {
                // SPI comm: concurrent tx to DAC and rx from ADC
                dacSYNCn->streamWrite(LOW);
                adcCONVn->streamWrite(LOW);
                usleep(200);
                if (transfer(fd, (unsigned char*) &tx, (unsigned char*) &rx, 2)==-1){
                        perror("Failed concurrent SPI comm");
                        return -1;
                }
                adcCONVn->streamWrite(HIGH);
                dacSYNCn->streamWrite(HIGH);
                // Convert code received from ADC into voltage measurement (motor's V_FORCE_OUT)
                if (rx & 0x8000) {
                        // Negative result, given in 16-bit 2's complement
                        vFsample = -0.000305176*(((float)0xFFFF-(float)rx)+1.0);
                } else {
                        // Positive result
                        vFsample = 0.000305176*(float(rx));
                }

                // Filter voltage measurement
                vFfilt = vFfilt + alpha*(vFsample-vFfilt);
                vFwindow = (vFwindow*(windowSize-1.0) + vFfilt)/windowSize;
                vFpassive += vFwindow; //vFfilt;
                usleep(200);
        }
        vFpassive = vFpassive/nPassiveSamples;
        cout << vFpassive << " V" << endl;


    // Trigger concurrent ADC and DAC sampling continuously until stopsign triggered
        cout << "Looping reactive motor controller until button press..." << endl;
        //adcBUSYn->waitForEdge(&busyEdge);
    while (stopsign == false)
        {
                //cout << "LOOP " << i << ": ";
                testLED->streamWrite(HIGH);

                // SPI comm: concurrent tx to DAC and rx from ADC
                dacSYNCn->streamWrite(LOW);
                adcCONVn->streamWrite(LOW);
                usleep(200);
                if (transfer(fd, (unsigned char*) &tx, (unsigned char*) &rx, 2)==-1){
                        perror("Failed concurrent SPI comm");
                        return -1;
                }
                adcCONVn->streamWrite(HIGH);
                dacSYNCn->streamWrite(HIGH);

                // Convert code received from ADC into voltage measurement (motor's V_FORCE_OUT)
                if (rx & 0x8000) {
                        // Negative result, given in 16-bit 2's complement
                        vFsample = -(20.0/65535.0)*(((float)0xFFFF-(float)rx)+1.0);
                } else {
                        // Positive result
                        vFsample = (20.0/65535.0)*(float(rx));
                }

                // Filter voltage measurement
                vFfilt = vFfilt + alpha*(vFsample-vFfilt);
                vFwindow = (vFwindow*(windowSize-1.0)+vFfilt)/windowSize;

                // Determine voltage response V_LENGTH_IN to ADC's measured V_FORCE_OUT voltage
                //vLI = (vFsample-vFpassive)*vLIcoeff; // NO FILTER
                vLI = (vFwindow-vFpassive)*vLIcoeff; // WITH FILTER

                // Convert desired LENGTH_IN voltage to 16-bit DAC code
                cLI = (unsigned short)((vLI+10.0)*3276.8);      // 3276.8 = 65536/20

                // Limit max V_LENGTH_IN gradient to protect motor in case of underdamped oscillation
                if (cLI > tx + cLImaxdiff) {
                        tx = tx + cLImaxdiff;
                } else if (cLI < tx - cLImaxdiff) {
                        tx = tx - cLImaxdiff;
                } else {
                        tx = cLI;
                }

                testLED->streamWrite(LOW);
                //cout << "rx: " << vFO << ",    tx: " << vLI << endl;
                usleep(100);
        }


    cout << "STOPSIGN TRIGGERED" << endl;

        // Shut down ADC control pins
        adcCSn->streamWrite(HIGH);
    adcCSn->streamClose();
    adcCONVn->streamClose();

        // Light LED to indicate ramp down initiated
        testLED->streamWrite(HIGH);

        // Ramp DAC voltage down to 0 V
        cout << "   DAC: Ramping down vLI" << endl;
        while(tx > 0x8000 + 0x0010) {
                tx -= 0x0010;
                // Update DAC over SPI0
                dacSYNCn->streamWrite(LOW);
                adcCONVn->streamWrite(LOW);
                usleep(100);
                if (transfer(fd, (unsigned char*) &tx, (unsigned char*) &rx, 2)==-1){
                        perror("Failed concurrent SPI comm");
                        return -1;
                }
                adcCONVn->streamWrite(HIGH);
                dacSYNCn->streamWrite(HIGH);
                usleep(100);
        }
        while(tx < 0x8000 - 0x0010) {
                tx += 0x0010;
                // Update DAC over SPI0
                dacSYNCn->streamWrite(LOW);
                adcCONVn->streamWrite(LOW);
                usleep(100);
                if (transfer(fd, (unsigned char*) &tx, (unsigned char*) &rx, 2)==-1){
                        perror("Failed concurrent SPI comm");
                        return -1;
                }
                adcCONVn->streamWrite(HIGH);
                dacSYNCn->streamWrite(HIGH);
                usleep(100);
        }
        // Make sure DAC is cleared to 0 V
        tx = 0x8000;
        dacSYNCn->streamWrite(LOW);
        adcCONVn->streamWrite(LOW);
        usleep(100);
        if (transfer(fd, (unsigned char*) &tx, (unsigned char*) &rx, 2)==-1){
                perror("Failed concurrent SPI comm");
                return -1;
        }
        adcCONVn->streamWrite(HIGH);
        dacSYNCn->streamWrite(HIGH);
        usleep(100);

        // Shut LED off to indicate ramp down completed
        testLED->streamWrite(LOW);

        // Shut down DAC control pins
        dacSYNCn->streamWrite(HIGH);
        dacSYNCn->streamClose();

    // Shut down test LED
    testLED->streamWrite(LOW);
    testLED->streamClose();

        // Close the SPI file
        close(fd);
        cout << "~fin~" << endl;

        return 0;
}
