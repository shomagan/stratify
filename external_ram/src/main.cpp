
#include <cstdio>
#include <sapi/sys.hpp>
#include <sapi/hal.hpp>
#include "external_ram.h"

int main(int argc, char * argv[]){
	Cli cli(argc, argv);
	cli.set_publisher("Stratify Labs, Inc");
	cli.handle_version();
    external_ram();
    return 0;
}
