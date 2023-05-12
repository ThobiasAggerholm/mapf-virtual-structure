#include "../inc/common.hpp"

int factorial(int n)
{
    int fact = 1, i;
   
   for(i=1; i<=n; i++)
    fact = fact * i;
   
   return fact;
}

int get_combinations(int a, int b)
{
    return double(factorial(a))/double(factorial(a-b)*factorial(b));
}

int get_direction(int a, int b)// right, left, down, up
{
    if(a+1 == b) // right
        return 0;
    else if(a-1 == b) //left
        return 1;
    else if(a < b) //  down. Heuristic works for square maps
        return 2;
    else if(a > b) // up. Heuristic works for square maps
        return 3;
    else
        return -1;
}
