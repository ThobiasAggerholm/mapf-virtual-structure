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