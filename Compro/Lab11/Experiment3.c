#include <stdio.h>
int main(void)
{
    struct student {
        char name[20];
        char id[14];
        char dep[5];
        int age;
    };

    struct student students[3];
    for (int i = 0; i < 3; i++) {
        printf("Enter student %d's name: ", i + 1);
        scanf("%s", students[i].name);
        printf("Enter student %d's ID: ", i + 1);
        scanf("%s", students[i].id);
        printf("Enter student %d's Department: ", i + 1);
        scanf("%s", students[i].dep);
        printf("Enter student %d's age: ", i + 1);
        scanf("%d", &students[i].age);

        printf("\n---Student %d's information---\n", i + 1);
        printf("Name: %s\n", students[i].name);
        printf("ID: %s\n", students[i].id);
        printf("Department: %s\n", students[i].dep);
        printf("Age: %d\n", students[i].age);
    }

    return 0;
}