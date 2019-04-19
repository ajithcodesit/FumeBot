/*

 * Original code modified from SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include

****************************************
Modified for Lepton by:

Copyright (c) 2014, Pure Engineering LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*******************************************
July 2017
Modified by Luke Van Horn for Lepton 3
 
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License.

May 2018
Modified by Ajith Thomas for Lepton3 python interfacing

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License.
 */


#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
    perror(s);
    abort();
}

static int fd;
static int ret;

static char device[50] = "/dev/spidev0.0";
static uint8_t mode = SPI_CPOL | SPI_CPHA;
static uint8_t bits = 8;
static uint32_t speed = 16000000; //Normally set to 16000000 Hz
static uint16_t delay = 65535; //Normally set to 65535;
static uint8_t status_bits = 0;

int8_t last_packet = -1;

#define VOSPI_FRAME_SIZE (164)
#define LEP_SPI_BUFFER (118080) //(118080)39360
/* modify /boot/cmdline.txt to include spidev.bufsiz=131072 */

static uint8_t rx_buf[LEP_SPI_BUFFER] = {0};  // Buffer used to store the incoming SPI data from Lepton3
static unsigned int lepton_image[240][80];  // The image is stores in 240x80 array and is then converted into the 160x120 array

// Actual lepton 3 image in 160x120 format in a 2D array
#define LEP3_IMG_WIDTH (160)
#define LEP3_IMG_HEIGHT (120)
static unsigned int lepton3_image[LEP3_IMG_HEIGHT][LEP3_IMG_WIDTH];

static bool debugging=false;  // To enable or disable debugging 

//Threading variables
static pthread_t pth_id;

void lepton3_image_120x160_arr(unsigned int *img_arr)  // Function to get the image in a 160x120 array exactly like the image
{
    int i=0;
    int j=0;

    //120 x 160 (Height x Width) Image

    for(i=0; i < 240; i += 2)
    {
        /* First 80 pixels in row */
        for(j = 0; j < 80; j++)
        {
            img_arr[j+(80*i)] = lepton_image[i][j];
        }

        /* Second 80 pixels in row */
        for(j = 0; j < 80; j++)
        {
            img_arr[(j+80)+(80*i)] = lepton_image[i + 1][j];
        }        
    }
}

int lepton3_transfer(int fd)  // Function that communicates with the Lepton 3 Module over the SPI bus
{
    int ret;
    int i;
    int ip;
    uint8_t packet_number = 0;
    uint8_t segment = 0;
    uint8_t current_segment = 0;
    int packet = 0;
    int state = 0;  //set to 1 when a valid segment is found
    int pixel = 0;
    
    struct spi_ioc_transfer tr = {  //Properties for the SPI to communicate with Lepton module
        .tx_buf = (unsigned long)NULL,
        .rx_buf = (unsigned long)rx_buf,  // The buufer that is used to store the incoming frame
        .len = LEP_SPI_BUFFER,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits
    };    
    
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr); //Reading the SPI bus (Returns 118080 most of the times)
    if (ret < 1) { 
        pabort("Cannot read SPI data");  // If the read failed
    }
    
    //The first data stream might be a part of the previous frame and the next frame and we need to find where the frame begins
    
    for(ip = 0; ip < (ret / VOSPI_FRAME_SIZE); ip++) {
        packet = ip * VOSPI_FRAME_SIZE;
        
        //check for invalid packet number
        if((rx_buf[packet] & 0x0f) == 0x0f) {
            state = 0;
            continue; //Below steps are not executed if the last 4 bits of the elements in the array is 0x0F
        }

        //Packet_number goes from 0 to 59 (60 packets)
        packet_number = rx_buf[packet + 1]; // Packet number is next element to the packet validity checking above
        
        if(packet_number > 0 && state == 0) {  // If the packet number is not zero the below steps are executed and the state has to be zero
            continue;
        }
        
        if(state == 1 && packet_number == 0) {
            state = 0;  //reset for new segment
        }
        
        //look for the start of a segment
        if(state == 0 && packet_number == 0 && (packet + (20 * VOSPI_FRAME_SIZE)) < ret) {
            /* 
            Bit[6:4] carries the segment [Bit shifted by 4 to get the segment] 
            (3280+packet[used as offset to get to the right location in a long array] location of buffer has the segment number)
            */
            segment = (rx_buf[packet + (20 * VOSPI_FRAME_SIZE)] & 0x70) >> 4; 
            
            /*
            If rx_buf location 3280+packet[used as offset]+1 has 20 and the segment number is in 
            range of 1 to 4 then we are sure a new segment was received
            */
            if(segment > 0 && segment < 5 && rx_buf[packet + (20 * VOSPI_FRAME_SIZE) + 1] == 20) {  
                state = 1; //State is set to true
                current_segment = segment; //Update to the latest received segment
                if(debugging) printf("New segment: %x \n", segment);
            } 
        }
        
        // The continue is not executed if a new segment is found
        if(!state) {  //State has to be 1 to execute the below steps
            continue;
        }

        for(i = 4; i < VOSPI_FRAME_SIZE; i+=2) // 80 iteratation VOSPI_FRAME_SIZE-4(Since starting at 4) / 2 (Since incrementing by 2) = 80 iterations 
        {
            //The current segment will ensure that the all the 240 lines are filled (Think of it like pages in EEPROM with 60 locations per page and 4 pages)
            pixel = packet_number + ((current_segment - 1) * 60); // 60 locations x 4 segments makes 240 lines with (164-4)/2=80 elements per line 
            lepton_image[pixel][(i - 4) / 2] = (rx_buf[packet + i] << 8 | rx_buf[packet + (i + 1)]); //Each pixel is 16-Bits in size (Loaction is offset by packets)
        }
        
        if(packet_number == 59) {  // If the last packet_number is reached
            //set the segment status bit
            status_bits |= ( 0x01 << (current_segment - 1)); // Bit shifted according to the segment and or'ed with the status bits (If all the 4 segments are read 0x0f is the status)
        }      
    }
    
    return status_bits;
}

void setup_lepton3(const char* spi_device, uint32_t spi_speed, uint16_t spi_delay,bool enable_debug)  //Function to set the speed and device of SPI 
{
	memset(device,'\0',sizeof(device));
	strcpy(device,spi_device);

	speed=spi_speed;
    delay=spi_delay;

	debugging=enable_debug;

	if(debugging)
	{
		printf("SPI device set to= %s\n",device);
		printf("SPI speed set to= %u\n",speed);
        printf("SPI delay set to= %u\n",delay);
	}
}

void open_lepton3(void)  // Open the file descriptor
{
    fd = open(device, O_RDWR);
    if (fd < 0)
    {
        pabort("Cannot open device");
    }

    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1)
    {
        pabort("Cannot set SPI mode");
    }

    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1)
    {
        pabort("Cannot get SPI mode");
    }

    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
    {
        pabort("Cannot set bits per word");
    }

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
    {
        pabort("Cannot get bits per word");
    }

    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
    {
        pabort("Cannot set max speed Hz");
    }

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
    {
        pabort("Cannot get max speed Hz");
    }

    if(debugging)
    {
	    printf("SPI mode: %d\n", mode);
	    printf("Bits per word: %d\n", bits);
	    printf("Max speed: %d Hz (%d KHz)\n", speed, speed/1000);
	}
}

void lepton3_capture(unsigned int *image_array_buf)
{
	//Loop until one image is formed
	while(status_bits != 0x0f) { lepton3_transfer(fd); }  // If the status bits is 0x0f all the 4 segments were read
   	
   	//Convert the image into the right format
    lepton3_image_120x160_arr(image_array_buf);

    status_bits=0; //Reset the status bits for the next frame
}

void close_lepton3(void)  // Close the file descriptor
{
	close(fd);

    if(debugging)
    {
        printf("Closed file descriptor\n");
    }
}

void *lepton3_stream_thread()  // Ran in a thread to capture image from Lepton 3
{
    while(1)  //Inifinite loop
    {
        //Loop until one image is formed
        while(status_bits != 0x0f) { lepton3_transfer(fd); }  // If the status bits is 0x0f all the 4 segments were read
        status_bits=0; //Reset the status bits for the next frame
    }

    return NULL;  //Nor reached here
}

void lepton3_stream_thread_start(void)  //Function to start the thread
{
    pthread_create(&pth_id,NULL,lepton3_stream_thread,NULL);

    if(debugging)
    {
        printf("Lepton 3 stream thread started\n");
    }
}

void open_lepton3_stream(void)  //Function to open and start the thread
{
    open_lepton3(); // To setup the SPI
    lepton3_stream_thread_start();

    if(debugging)
    {
        printf("Opening stream done\n");
    }
}

void close_lepton3_stream(void)  //Function to close or stop the thread
{
    int ret=pthread_cancel(pth_id);
    if(debugging)
    {
        printf("Closed stream with return: %u\n",ret);
    }

    close_lepton3(); //To close the file descriptor
}

void read_lepton3_stream(unsigned int *image_array_buf)  //Function to read the image from the stream thread
{
    //The convert the 240x80 array to the correct 120x160 frame
    lepton3_image_120x160_arr(image_array_buf);
}

//The main is only used for debuuging  
int main(int argc, char *argv[])
{
	setup_lepton3("/dev/spidev0.0",16000000,65535,true);

    open_lepton3();

    lepton3_capture(*lepton3_image);

    close_lepton3();
    
    //For Debug

    printf("-----Image start-----\n\n");

    for(int m=0; m < LEP3_IMG_HEIGHT; m++)
    {
        for(int n=0; n < LEP3_IMG_WIDTH; n++)
        {
            printf(" %u ",lepton3_image[m][n]);
        }
        printf("\n");
    }

    printf("-----Image end-----\n\n");

    //save_pgm_file();

    return ret;
}
