
#include <cstdio>
#include <sapi/sys.hpp>
#include <sapi/hal.hpp>
#include "qspi_flash.h"

int main(int argc, char * argv[]){
	Cli cli(argc, argv);
	cli.set_publisher("Stratify Labs, Inc");
	cli.handle_version();
    qspi_flash();
    return 0;
}
