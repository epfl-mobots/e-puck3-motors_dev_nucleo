#include "test.py.h"

const __attribute__((section(".py_flash"))) char myarray[] = {
"\r\n"
"import gc\r\n"
"\r\n"
"def divide(a,b):\r\n"
"	return a/b\r\n"
"\r\n"
"DIX = const(10)\r\n"
"\r\n"
"def print_div():\r\n"
"	for i in range(10):\r\n"
"		if i%2 == 0:\r\n"
"			print('le nombre ',i,' est pair et ',i,' divisé par ', x, ' = ', divide(i,DIX))\r\n"
"		else:\r\n"
"			print('le nombre ',i,' est impair et ',i,' divisé par ', x, ' = ', divide(i,DIX))\r\n"
"\r\n"
"def print_tab(a):\r\n"
"	for i in a:\r\n"
"		type_i = str(type(i)) \r\n"
"		if type_i == \"<class 'int'>\":\r\n"
"			print(i,' is an int')\r\n"
"		elif type_i == \"<class 'str'>\":\r\n"
"			print(i,' is a string')\r\n"
"		else:\r\n"
"			print(\"I don't know the type \", type_i)\r\n"
"\r\n"
"tab = [1,2,3,4,5,6,'test']\r\n"
};

