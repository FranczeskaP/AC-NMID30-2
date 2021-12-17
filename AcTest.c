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

modbus_t *slave;
static uint8_t schedulerCalled = 0u;
static struct timespec WaitTimeModbus = {
    .tv_sec = 0,
    .tv_nsec = TIME_MS_MODBUS * 1000000
};
static struct timeval response_timeout = {
    .tv_sec = 1,
    .tv_usec = 0
};
static myfloat var;

static void SchedulerCalled(int signum);
static void initAcModbus(void);
static void modbusDeInit(void);
static uint32_t convertToInt(uint32_t* arr, uint32_t low, uint32_t high);
static void toArray(uint16_t high, uint16_t low, uint32_t *arr);
static void decodeData(uint16_t high, uint16_t low);

int main()
{
    signal(SIGALRM, SchedulerCalled);
    alarm(5);
    initAcModbus();
    while(1)
    {
        if(1u == schedulerCalled)
        {
            schedulerCalled = 0u;
            uint16_t receivedData[12u];
            int numOfReadRegs = modbus_read_input_registers(slave, 0u, 12u, receivedData);
            if (numOfReadRegs != 12) 
            {
                fprintf(stderr, "Failed to read: %s\n", modbus_strerror(errno));
            }
            nanosleep(&WaitTimeModbus, &WaitTimeModbus);
            uint16_t receivedData1[16u];
            numOfReadRegs = modbus_read_input_registers(slave, 0x34u, 16u, receivedData1);
            if (numOfReadRegs != 16) 
            {
                fprintf(stderr, "Failed to read: %s\n", modbus_strerror(errno));
            }
            nanosleep(&WaitTimeModbus, &WaitTimeModbus);
            uint16_t receivedData2[4u];
            numOfReadRegs = modbus_read_input_registers(slave, 0xF8u, 4u, receivedData2);
            if (numOfReadRegs != 4) 
            {
                fprintf(stderr, "Failed to read: %s\n", modbus_strerror(errno));
            }
            nanosleep(&WaitTimeModbus, &WaitTimeModbus);
            uint16_t receivedData3[2u];
            numOfReadRegs = modbus_read_input_registers(slave, 0x0158u, 2u, receivedData3);
            if (numOfReadRegs != 2) 
            {
                fprintf(stderr, "Failed to read: %s\n", modbus_strerror(errno));
            }
            nanosleep(&WaitTimeModbus, &WaitTimeModbus);
            decodeData(receivedData[0], receivedData[1]);
	        printf("Napicie fazy pierwszej: %f", var.f);
            decodeData(receivedData[2], receivedData[3]);
	        printf("Napicie fazy drugiej: %f", var.f);
            decodeData(receivedData[4], receivedData[5]);
	        printf("Napicie fazy trzeciej: %f", var.f);
            decodeData(receivedData[6], receivedData[7]);
	        printf("Prad fazy pierwszej: %f", var.f);
            decodeData(receivedData[8], receivedData[9]);
	        printf("Prad fazy drugiej: %f", var.f);
            decodeData(receivedData[10], receivedData[11]);
	        printf("Prad fazy trzeciej: %f", var.f);
            decodeData(receivedData1[0], receivedData1[1]);
	        printf("Moc czynna łączna: %f", var.f);
            decodeData(receivedData1[2], receivedData1[3]);
	        printf("Moc pozorna łączna: %f", var.f);
            decodeData(receivedData1[4], receivedData1[5]);
	        printf("Moc Bierna łączna: %f", var.f);
            decodeData(receivedData1[6], receivedData1[7]);
	        printf("współczynnik mocy cos: %f", var.f);
            decodeData(receivedData1[10], receivedData1[11]);
	        printf("częstotliwość sieci: %f", var.f);
            decodeData(receivedData1[12], receivedData1[13]);
	        printf("Energia czynna pobierana: %f", var.f);
            decodeData(receivedData1[14], receivedData1[14]);
	        printf("Energia czynna oddawana: %f", var.f);
            decodeData(receivedData2[0], receivedData2[1]);
	        printf("THD napięcia: %f", var.f);
            decodeData(receivedData2[2], receivedData2[3]);
	        printf("THD prądu: %f", var.f);
            decodeData(receivedData3[0], receivedData3[1]);
	        printf("Energia bierna: %f", var.f);
        }
        else
        {
            printf(".\n");
        }
        sleep(1);
    }
    modbusDeInit();
    return 0;
}

static void SchedulerCalled(int signum)
{
    schedulerCalled = 1u;
    alarm(10);
}

static void modbusDeInit(void)
{
    modbus_close(slave);
    modbus_free(slave);
}

static void initAcModbus(void)
{
    slave = modbus_new_rtu(DEVICE_NAME, BAUDRATE, PARITY, DATA_BITS, STOP_BITS);
    if (!slave) 
    {
        fprintf(stderr, "Failed to create the context: %s\n", modbus_strerror(errno));
        exit(1);
    }

    if (modbus_connect(slave) == -1) 
    {
        fprintf(stderr, "Unable to connect: %s\n", modbus_strerror(errno));
        modbus_free(slave);
        exit(1);
    }
    modbus_set_response_timeout(slave, response_timeout.tv_sec, response_timeout.tv_usec);
    modbus_set_slave(slave, 0u);
}

static int32_t convertReceivedData(uint16_t high, uint16_t low)
{
    uint8_t znak = (high & 0x8000u) >> 15u;
    uint16_t wykladnik = ((high & 0x7F80u) >> 7u);
    uint32_t liczba = ((high & 0x7Fu) << 16u) | low | 0x800000;
    uint32_t czescCalkowita = 0u;
    uint32_t czescUlamkowa = 0u;
    if(znak == 0u)
    {
        czescCalkowita = liczba >> (24u - (wykladnik + 1u));
    }

}

static uint32_t convertToInt(uint32_t* arr, uint32_t low, uint32_t high)
{
	uint32_t f = 0, i;
	for (i = high; i >= low; i--) {
		f = f + arr[i] * pow(2, high - i);
	}
	return f;
}

static void toArray(uint16_t high, uint16_t low, uint32_t *arr)
{
    uint32_t x = (high << 16) | low;
    int j = 0;
    for(int i = 31; i >= 0; i--)
    {
        arr[i] = (x >> j) & 0x1;
        j++;
    }
}

static void decodeData(uint16_t high, uint16_t low)
{
    uint32_t valInArr[32];
	toArray(high, low, valInArr);
	uint32_t f = convertToInt(valInArr, 9, 31);
	var.raw.mantissa = f;
	f = convertToInt(valInArr, 1, 8);
	var.raw.exponent = f;
	var.raw.sign = valInArr[0];
}
