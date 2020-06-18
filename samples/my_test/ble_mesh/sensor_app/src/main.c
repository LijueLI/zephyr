#include <sys/printk.h>
#include <settings/settings.h>
#include <nrf.h>
#include <sys/byteorder.h>
#include <device.h>

#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include  <bluetooth/mesh.h>

#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include <drivers/sensor.h>

#include "pherphs.h"
#include "mesh.h"

void main(void){
    int err;
    err = pheriphs_init();
    if(err){
        printk("Failed to get device binding\n");
    }
    err = mesh_init();
    if(err){
        printf("Bluetooth init failed(err %d)\n",err);
    }
}