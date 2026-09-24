#include <stdio.h>
int main(void)
{
    enum favorite_animal { DOG, CAT, MOUSE, SNAKE, RABBIT };
    
    enum favorite_animal my_animal = DOG;

    printf("My favorite animal is: %d\n", my_animal);
    
    return 0;
}