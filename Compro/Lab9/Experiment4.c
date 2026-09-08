#include <stdio.h>

float getStudentID(void) {
    double id;
    printf("Enter student ID: ");
    scanf("%lf", &id);
    id = ((long int)(id))%1000000;
    return id;
}

void main(void) {
    printf("Student ID(last 6 digits): %06.0f\n", getStudentID());
}