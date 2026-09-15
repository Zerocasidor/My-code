#include <stdio.h>
void IsTempHigh(float temperature) 
{
    if (temperature > 30.0) {
        printf("Temperature %.1f is high.\n", temperature);
    }
}

int main(void)
{
    float temp[] = {30.1, 28.7, 29.5, 31.2, 32.8};
    int size = sizeof(temp) / sizeof(temp[0]);

    for (int i = 0; i < size; i++) {
        IsTempHigh(temp[i]);
    }
}