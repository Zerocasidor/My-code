#include <stdio.h>

void calsum(float id, float weight) {
    float sum = id + weight;
    printf("Sum is %.2f\n", sum);
}

void main(void) {
    calsum(83, 44.5);
}
