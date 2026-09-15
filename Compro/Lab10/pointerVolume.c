#include <stdio.h>
#include <math.h>
void calAreaVol(float w, float l, float h, float *a, float *v) 
{
    *a = 2 * (w * l + w * h + l * h);
    *v = w * l * h;
}

int main(void)
{
    float area, volume;
    calAreaVol(5.0, 10.0, 15.0, &area, &volume);
    
    printf("Area of cuboid: %.2f\n", area);
    printf("Volume of cuboid: %.2f\n", volume);
}