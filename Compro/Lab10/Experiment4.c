#include <stdio.h>
#include <math.h>
void calCylinderVol(float r, float h, float *v) 
{
    *v = M_PI * r * r * h;
}
int main(void)
{
    float volume;
    calCylinderVol(5.0, 10.0, &volume);
    printf("Volume of cylinder: %.2f\n", volume);
}