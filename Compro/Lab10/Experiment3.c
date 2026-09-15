#include <stdio.h>
void count(int *p) 
{
    *p = *p+3; //add last digit my id
}
int main(void)
{
    int x = 0;
    for (int i = 0; i < 10; i++) {
        count(&x);
    }
    printf("x: %d\n", x);
}