#include <stdio.h>
#include <math.h>

#define VAR 2.0

double mean = 2;

double gauss(double theta)
{
	return exp((-1 * (theta - mean) * (theta - mean)) / (VAR * VAR));
}

double integral(double(*f)(double x), double a, double b, int n)
{
	double step = (b - a) / n;
	double area = 0.0;
	int i = 0;
	for (i = 0; i < n; i++)
	{
		area += f(a + (i + 0.5) * step) * step;
	}
	return area;
}

int main()
{
	printf("%lf\n", integral(gauss, 0, 2 * M_PI, 1000));
}
