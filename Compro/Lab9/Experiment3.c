#include <stdio.h>

char grading(int score) {
    if (score >= 85 && score <= 100) {
        return 'A';
    } else if (score >= 70) {
        return 'B';
    } else if (score >= 55) {
        return 'C';
    } else {
        return 'F';
    }
}

void main(void) {
    printf("%c\n", grading(95));
}
