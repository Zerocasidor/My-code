#include <stdio.h>
int main(void)
{
    enum color { RED, GREEN, BLUE };
    enum month { JANUARY = 1, FEBRUARY, MARCH, APRIL, MAY, JUNE, JULY, AUGUST, SEPTEMBER, OCTOBER, NOVEMBER, DECEMBER };
    
    enum direction { NORTH, EAST, SOUTH, WEST };
    enum direction my_direction = WEST;
    printf("My direction is: %d\n", my_direction);
    
    return 0;
}