/*
 * database.c
 *
 *  Created on: 14 Oct 2022
 *      Author: Calvert
 */

/* Standard Library*/
#include <stdio.h>
#include <string.h>
#include "database.h"

char findCharacter(char *input)
{
  //Populate mydata with numbers
    if(strcmp("101000111011101", input) == 0)
        return '0';
    if(strcmp("111010001010111", input) == 0)
        return '1';
    if(strcmp("101110001010111", input) == 0)
        return '2';
    if(strcmp("111011100010101", input) == 0)
        return '3';
    if(strcmp("101000111010111", input) == 0)
        return '4';
    if(strcmp("111010001110101", input) == 0)
        return '5';
    if(strcmp("101110001110101", input) == 0)
        return '6';
    if(strcmp("101000101110111", input) == 0)
        return '7';
    if(strcmp("111010001011101", input) == 0)
        return '8';
    if(strcmp("101110001011101", input) == 0)
        return '9';

    //Input symbols for mydata
    if(strcmp("100010101110111", input) == 0)
        return '-';
    if(strcmp("111000101011101", input) == 0)
        return '.';
    if(strcmp("100011101011101", input) == 0)
        return ' ';
    if(strcmp("100010001000101", input) == 0)
        return '$';
    if(strcmp("100010001010001", input) == 0)
        return '/';
    if(strcmp("100010100010001", input) == 0)
        return '+';
    if(strcmp("101000100010001", input) == 0)
        return '%';
    if(strcmp("100010111011101", input) == 0)
        return '*';

    //Populate mydata with alphabets
    if(strcmp("111010100010111", input) == 0)
        return 'A';
    if(strcmp("101110100010111", input) == 0)
        return 'B';
    if(strcmp("111011101000101", input) == 0)
        return 'C';
    if(strcmp("101011100010111", input) == 0)
        return 'D';
    if(strcmp("111010111000101", input) == 0)
        return 'E';
    if(strcmp("101110111000101", input) == 0)
        return 'F';
    if(strcmp("101010001110111", input) == 0)
        return 'G';
    if(strcmp("111010100011101", input) == 0)
        return 'H';
    if(strcmp("101110100011101", input) == 0)
        return 'I';
    if(strcmp("101011100011101", input) == 0)
        return 'J';
    if(strcmp("111010101000111", input) == 0)
        return 'K';
    if(strcmp("101110101000111", input) == 0)
        return 'L';
    if(strcmp("111011101010001", input) == 0)
        return 'M';
    if(strcmp("101011101000111", input) == 0)
        return 'N';
    if(strcmp("111010111010001", input) == 0)
        return 'O';
    if(strcmp("101110111010001", input) == 0)
        return 'P';
    if(strcmp("101010111000111", input) == 0)
        return 'Q';
    if(strcmp("111010101110001", input) == 0)
        return 'R';
    if(strcmp("101110101110001", input) == 0)
        return 'S';
    if(strcmp("101011101110001", input) == 0)
        return 'T';
    if(strcmp("111000101010111", input) == 0)
        return 'U';
    if(strcmp("100011101010111", input) == 0)
        return 'V';
    if(strcmp("111000111010101", input) == 0)
        return 'W';
    if(strcmp("100010111010111", input) == 0)
        return 'X';
    if(strcmp("111000101110101", input) == 0)
        return 'Y';
    if(strcmp("100011101110101", input) == 0)
        return 'Z';

    return '.';
}
