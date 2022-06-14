// C program to compare two files and report
// mismatches by displaying line number and
// position of line.
#include<stdio.h>
#include<string.h>
#include<stdlib.h>

void compareFiles(FILE *fp1, FILE *fp2)
{
    // fetching character of two file
    // in two variable ch1 and ch2
    char ch1 = getc(fp1);
    char ch2 = getc(fp2);
    int temp_bit_error = 0;
  
    // error keeps track of number of errors
    // pos keeps track of position of errors
    // line keeps track of error line
    int error = 0, pos = 0, line = 1, bit_error = 0;
  
    // iterate loop till end of file
    while (ch1 != EOF && ch2 != EOF)
    {
        pos++;
  
        // if both variable encounters new
        // line then line variable is incremented
        // and pos variable is set to 0
        if (ch1 == '\n' && ch2 == '\n')
        {
            line++;
            pos = 0;
        }
  
        // if fetched data is not equal then
        // error is incremented
        if (ch1 != ch2)
        {
            temp_bit_error = 0;
            error++;
            for (int i = 0; i < 8; i++) {
 
            // right shift both the numbers by 'i' and
            // check if the bit at the 0th position is different
            if (((ch1 >> i) & 1) != ((ch2 >> i) & 1)) {
                temp_bit_error++;
            }
        }
            bit_error += temp_bit_error;
            printf("Line Number : %d \tError"
               " Position : %d \tBit errors : %d\n", line, pos, temp_bit_error);
            
        }
  
        // fetching character until end of file
        ch1 = getc(fp1);
        ch2 = getc(fp2);
    }
  
    printf("Total Errors : %d\t", error);
    printf("Total Bit errors : %d\t", bit_error);
}

// Driver code
int main(int argc, char **argv)
{
    // opening both file in read only mode
    FILE *fp1 = fopen(argv[1], "r");
    FILE *fp2 = fopen(argv[2], "r");
  
    if (fp1 == NULL || fp2 == NULL)
    {
       printf("Error : Files not open");
       exit(0);
    }
  
    compareFiles(fp1, fp2);
  
    // closing both file
    fclose(fp1);
    fclose(fp2);
    return 0;
}
