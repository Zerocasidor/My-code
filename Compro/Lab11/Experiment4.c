#include <stdio.h>
int main(void)
{
    typedef struct {
        int n; //numerator
        int d; //denominator
    } fraction;

    fraction first = {1, 9}; // 1/9 (change 0 to 9)
    fraction second = {8, 3}; // 8/3
    fraction result;
    result.n = first.n * second.d + second.n * first.d; // 1*3 + 8*9 = 75
    result.d = first.d * second.d; // 9*3 = 27

    printf("Result: %d/%d\n", result.n, result.d); // 75/27

    return 0;
}