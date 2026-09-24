#include <stdio.h>
int main(void)
{
    struct book {
        char title[20];
        char author[20];
        int price;
        int page;
    };

    struct book myBook = {"Black Holes", "Ajavorong Chantamas", 230, 80};

    printf("title:  %s\n", myBook.title);
    printf("author: %s\n", myBook.author);
    printf("price:  %d\n", myBook.price);
    printf("page:   %d\n", myBook.page);
    return 0;
}