#include <stdlib.h>
#include <modbus.h>
#include <signal.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <stdio.h>

#define DEVICE_NAME             ("/dev/ttyUSB0")
#define BAUDRATE                (9600u)
#define PARITY                  ('N')
#define DATA_BITS               (8u)
#define STOP_BITS               (1u)
#define TIME_MS_MODBUS          (2u)

typedef union {

	float f;
	struct
	{
		uint32_t mantissa : 23;
		uint32_t exponent : 8;
		uint32_t sign : 1;

	} raw;
} myfloat;
myfloat var;
static void codeData(float data, uint16_t *dataHigh, uint16_t *dataLow);


int main()
{
    modbus_mapping_t *mapping = modbus_mapping_new(0, 0, 200, 0);
    if (!mapping) 
    {
        fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
        exit(1);
    }

    codeData(-2.25, &mapping->tab_registers[0], &mapping->tab_registers[1]);
    codeData(256.9, &mapping->tab_registers[2], &mapping->tab_registers[3]);
    codeData(-32.25, &mapping->tab_registers[4], &mapping->tab_registers[5]);
    codeData(-432.247, &mapping->tab_registers[6], &mapping->tab_registers[7]);
    codeData(-26.25, &mapping->tab_registers[8], &mapping->tab_registers[9]);
    codeData(45.8465, &mapping->tab_registers[10], &mapping->tab_registers[11]);


    codeData(-5.3, &mapping->tab_registers[52], &mapping->tab_registers[53]);
    codeData(0.345, &mapping->tab_registers[54], &mapping->tab_registers[55]);
    codeData(34.0, &mapping->tab_registers[56], &mapping->tab_registers[57]);
    codeData(-78.78, &mapping->tab_registers[58], &mapping->tab_registers[59]);
    codeData(-146.56, &mapping->tab_registers[60], &mapping->tab_registers[61]);
    codeData(-2.0, &mapping->tab_registers[62], &mapping->tab_registers[63]);
    codeData(80.345, &mapping->tab_registers[64], &mapping->tab_registers[65]);
    codeData(-795.29, &mapping->tab_registers[66], &mapping->tab_registers[67]);


    codeData(208.22425, &mapping->tab_registers[248], &mapping->tab_registers[249]);
    codeData(25.2655, &mapping->tab_registers[250], &mapping->tab_registers[251]);


    codeData(-452.257, &mapping->tab_registers[344], &mapping->tab_registers[345]);


    modbus_t *ctx = modbus_new_rtu(DEVICE_NAME, BAUDRATE, PARITY, DATA_BITS, STOP_BITS);
    if (!ctx) 
    {
        fprintf(stderr, "Failed to create the context: %s\n", modbus_strerror(errno));
        exit(1);
    }

    modbus_set_slave(ctx, 0);


    if (modbus_connect(ctx) == -1) 
    {
        fprintf(stderr, "Unable to connect: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        exit(1);
    }


    uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH];
    int len;

    while(1) 
    {
        len = modbus_receive(ctx, req);
        if (len == -1) break;

        len = modbus_reply(ctx, req, len, mapping);
        if (len == -1) break;
    }
    printf("Exit the loop: %s\n", modbus_strerror(errno));

    modbus_mapping_free(mapping);
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}


static void codeData(float data, uint16_t *dataHigh, uint16_t *dataLow)
{
    var.f = data;
    *dataHigh = var.raw.sign <<15;
    *dataHigh |= var.raw.exponent << 7;
    *dataHigh |= var.raw.mantissa >> 16;
    *dataLow = var.raw.mantissa & 0xFFFF;
}
