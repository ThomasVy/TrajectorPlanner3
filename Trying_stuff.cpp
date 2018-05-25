#include <iostream>
using namespace std;
int main(int argc, char **argv)
{
	int a = 7;
	int &b =a;
	int c =b;
	cout<<a<<" "<<b<<" "<<c<<endl;	
	c++;
	cout<<a<<" "<<b<<" "<<c<<endl;	
	return 0;
}

