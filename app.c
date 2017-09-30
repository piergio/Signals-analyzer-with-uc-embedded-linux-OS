#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define clear()	printf("\33[2J")
//#define DEBUG 1

int main(int argc, char **argv){
	
	unsigned int period = 0;
	float freq = 0;
	char *app_name = argv[0];
	char *dev_name = "/dev/sample";
	int fd = -1;
	int ret = 0;
	
	if ((fd = open(dev_name, O_RDWR)) < 0) {
		fprintf(stderr, "%s: unable to open %s: %s\n", app_name, dev_name, strerror(errno));
		return( 1 );
	}
	
	while(1){
		
		usleep(1000000);
		ret = read(fd, &period, sizeof(period));	// reads the period value from the kernel space

		#ifdef DEBUG	
		if(ret<0){
			printf("Read error\n");		
		}
		#endif

		freq = (1/(float)period)*1000000;
		clear();
		fprintf(stdout, "\n\t\t## Measuring ##\n");
		fprintf(stdout, "\nPeriod: %f ms - Frequency: %f Hz\n", (float)period/1000, freq);
		fflush(stdout);				
	}

	if (fd >= 0) {
		close(fd);
	}
	return( 0 );
}
