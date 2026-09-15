#include <stdio.h>
float calAverageWeight(float t[]) 
{
    float sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += t[i];
    }
    return sum / 5;
}
int main(void)
{
    float weight[5];

    for (int i = 0; i < 5; i++) {
        printf("Enter weight %d: ", i + 1);
        scanf("%f", &weight[i]);
    }
    printf("Average weight: %.2f\n", calAverageWeight(weight));
}