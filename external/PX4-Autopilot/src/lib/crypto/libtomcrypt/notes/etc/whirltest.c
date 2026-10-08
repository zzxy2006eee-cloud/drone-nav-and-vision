#include <stdio.h>

int main(void)
{
   char buf[4096];
   int x;
   
   while (fgets(buf, sizeof(buf)-2, stdin) != NULL) {
        for (x = 0; x < 128; ) {
            printf("0x%c%c, ", buf[x], buf[x+1]);
            if (!((x += 2) & 31)) printf("\n");
        }
   }
}


/* ref:         px4, develop */
/* git commit:  673f5ce29015a9bba3c96792920a10601b5b0718 */
/* commit time: 2021-06-04 18:51:41 +0200 */
