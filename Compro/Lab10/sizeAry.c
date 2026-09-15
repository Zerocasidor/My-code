#include <stdio.h>
float findAverage(float ary[], int size) 
{
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += ary[i];
    }
    return sum / size;
}
void main(void)
{
    float score[3] = {30.1, 28.7, 29.5};
    printf("Average score: %.2f\n", findAverage(score, sizeof(score) / sizeof(score[0])));
}