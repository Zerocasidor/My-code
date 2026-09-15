#include <stdio.h>

int printArray(int data[], int size) 
{
    for (int i = 0; i < size; i++) {
        printf("%d ", data[i]);
    }
    printf("\n");
    return 0;
}

int swap(int data[], int size) 
{
    printArray(data, size);
    int temp, i;
    for (i = 0; i < size / 2; i++) {
        temp = data[i];
        data[i] = data[size - i - 1];
        data[size - i - 1] = temp;
        printArray(data, size);

    }
    return 0;
}

int main(void)
{
    int data[] = {6, 6, 5, 4, 1, 1, 0};
    int size = sizeof(data) / sizeof(data[0]);
    
    swap(data, size);
    return 0;
}