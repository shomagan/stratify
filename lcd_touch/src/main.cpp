
#include <cstdio>
#include <sapi/sys.hpp>
#include <sapi/hal.hpp>
#include "lcd_test.h"

int main(int argc, char * argv[]){
	Cli cli(argc, argv);
	cli.set_publisher("Stratify Labs, Inc");
	cli.handle_version();
    lcd_test();
    return 0;
}
