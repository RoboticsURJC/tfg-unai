#include <Wire.h>

#define I2C_ADDRESS 0x4

int g_iOnRequestActionCode = 0;
unsigned long g_lSecondsSinceStart = 0;

void setup() 
{
    Wire.begin(I2C_ADDRESS);
    Wire.onRequest(sendDataOverI2CGateway);
    Wire.onReceive(defineOnRequestAction);
}

int counter = 0;
void loop() 
{
    counter++;
}



void sendOperationTimeDataOverI2C()
{
    unsigned long longInt = g_lSecondsSinceStart;
    byte size = sizeof(longInt);

    byte arr[size];
    for (int i = 0; i < size; i++)
    {
        int iBitShift = 8 * (size - i - 1);
        if (iBitShift >= 8)
            arr[i] = ((longInt >> iBitShift) & 0xFF);
        else
            arr[i] = (longInt & 0xFF);
    }
    Wire.write(arr, size);
}

void sendDataOverI2CGateway()
{
  int number = 0;
  Wire.write(number);
}

void defineOnRequestAction(int iBuffer) 
{
    while (Wire.available())
    {
        g_iOnRequestActionCode = Wire.read();
    }
}
