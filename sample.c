/*
 * sample.c - The simplest loadable kernel module.
 * Intended as a template for development of more
 * meaningful kernel modules.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>

/*
 * Definition of the GPIOs we want to use
*/
#define	BTN	139		// User button, PI.11
#define LED	129		// User led, PI.1

/*
 * Definition of ioctl commands
*/
#define	INVALID		-1
#define WRITE_HPERIOD	0
#define	WRITE_BLINK	1

//#define DEBUG 1

#define TIM12_BASEADDRESS	0x40001800
#define TIM12_CR1			0x00
#define TIM12_SMCR			0x08
#define TIM12_SR			0x10
#define TIM12_DIER			0x0C
#define TIM12_EGR			0x14
#define TIM12_CCMR1			0x18
#define TIM12_CCER			0x20
#define TIM12_PSC			0x28
#define TIM12_COUNTER_VAL	0x34	//TIM12_CCR1 REGISTER for the counter value
#define TIM12_CHECK_UIF		0x01
#define TIM12_CHECK_CC1IF	0x02

#define PORTH_BASEADDRESS	0x40021C00
#define PORTH6_MODER		0x00		
#define PORTH6_AFRL			0x20

#define RCC_BASEADDRESS		0x40023800
#define RCC_AHB1ENR			0x30
#define RCC_APB1ENR			0x40

/* Pointers to RCC registers */
unsigned int volatile	* const rcc_ah = (unsigned int *)(RCC_BASEADDRESS+RCC_AHB1ENR);	
unsigned int volatile	* const rcc_ap = (unsigned int *)(RCC_BASEADDRESS+RCC_APB1ENR);		

/* Pointers to GPIO registers */
unsigned int volatile	* const portH_MODER = (unsigned int *)(PORTH_BASEADDRESS+PORTH6_MODER);
unsigned int volatile	* const portH_AFRL = (unsigned int *)(PORTH_BASEADDRESS+PORTH6_AFRL);

/* Pointers to TIM12 registers */
unsigned short	volatile * const tim12_PSC = (unsigned short *)(TIM12_BASEADDRESS+TIM12_PSC);
unsigned short	volatile * const tim12_CR1 = (unsigned short *)(TIM12_BASEADDRESS+TIM12_CR1);
unsigned short	volatile * const tim12_SMCR = (unsigned short *)(TIM12_BASEADDRESS+TIM12_SMCR);
unsigned short	volatile * const tim12_SR = (unsigned short *)(TIM12_BASEADDRESS+TIM12_SR);
unsigned short	volatile * const tim12_DIER = (unsigned short *)(TIM12_BASEADDRESS+TIM12_DIER);
unsigned short	volatile * const tim12_EGR = (unsigned short *)(TIM12_BASEADDRESS+TIM12_EGR);
unsigned short	volatile * const tim12_CCMR1 = (unsigned short *)(TIM12_BASEADDRESS+TIM12_CCMR1);
unsigned short	volatile * const tim12_CCER = (unsigned short *)(TIM12_BASEADDRESS+TIM12_CCER);
unsigned short	volatile * const tim12_COUNTER_VAL = (unsigned short *)(TIM12_BASEADDRESS+TIM12_COUNTER_VAL);

unsigned int 	volatile value = 0;
unsigned short	volatile high = 0;

/*
 * Device major number
 */
static uint module_major = 166;

/*
 * Device name
 */
static char * module_name = "sample";

/*
 * Device access lock. Only one process can access the driver at a time
 */
static int sample_lock = 0;

/*
 * Device variables
*/
static volatile char btn_value; // BTN value
static int	blk_hperiod;	// blinking semi-period in milliseconds
int		mode = INVALID;	// device mode: INVALID = do nothink
				// WRITE_HPERIOD = next write sets the blinking hperiod
				// WRITE_BLINK   = next write will blink the led for the give number of times 

/*
 * Declare the workqueue
 */
static struct workqueue_struct *my_wq;

typedef struct {
	struct work_struct my_work;
	int	delay, n_cycles;
} my_work_t;

static my_work_t work;

/*
 * Work function
 */
static void my_wq_function( struct work_struct *work ){

	int	delay, i, n_cycles;
	my_work_t *my_work;

	my_work = (my_work_t *)work;
	delay = my_work->delay;
	n_cycles = my_work->n_cycles;

	gpio_set_value( LED, 0 );
	for( i = 0; i < n_cycles; i++ )
	{
		gpio_set_value( LED, 1 );
		msleep( blk_hperiod );
		gpio_set_value( LED, 0 );
		msleep( blk_hperiod );
	}
}

/*
 * TIMER12 and GIOPH registers setup
*/

static void setup_registers(){

/* --------------------------------------- RCC --------------------------------------- */
	
		*rcc_ah			|=	0x80;		// Enables GPIOH CLOCK
		#ifdef DEBUG	
		printf("rcc_gpioH: %d\n\n\n", *rcc_ah);
		#endif

		*rcc_ap			|=	0x40;		// Enables TIM12 CLOCK
		#ifdef DEBUG	
		printf("rcc_tim12: %d\n", *rcc_ap);
		#endif

/* ------------------------------------- TIMER12 ------------------------------------- */
	
		//	clk_counter = f_clock / ( psc[15:0] + 1 )
		//	16 bits prescaler

		*tim12_PSC		|=	0x63; 	// clock_counter = 1MHz
		#ifdef DEBUG	
		printk(KERN_INFO "psc: %d\n", *tim12_PSC);
		#endif
	
		*tim12_CR1		|=	0x05;	//	Enables counter and sets update interrupt as overflow
		#ifdef DEBUG
		printk(KERN_INFO "cr1: %d\n", *tim12_CR1);
		#endif
	
		*tim12_CCMR1	|=	0x01;	//	Maps IC1 on TI1
		#ifdef DEBUG
		printk(KERN_INFO "ccmr1: %d\n", *tim12_CCMR1);
		#endif

		*tim12_CCER		|=	0x01;	//	Enables the capture
		#ifdef DEBUG
		printk(KERN_INFO "ccer: %d\n", *tim12_CCER);
		#endif

		*tim12_SMCR		|=	0x54;	//	Autoresets counter on rising event
		#ifdef DEBUG
		printk(KERN_INFO "smcr: %d\n", *tim12_smcr);
		#endif

		*tim12_DIER		|=	0x03;	//	Enables update and capture interrupts
		#ifdef DEBUG
		printf("dier: %d\n", *tim12_DIER);
		#endif

/* --------------------------------------- GPIOH --------------------------------------- */

		*portH_MODER	|=	0x2000;	//	Sets alternatate-mode-function on PH.6
		#ifdef DEBUG
		printk(KERN_INFO "moder: %d\n", *portH_MODER);
		#endif
	
		*portH_AFRL		|=	0x9000000;	//	Sets multiplexer as AF9
		#ifdef DEBUG
		printk(KERN_INFO "afrl: %d\n", *portH_AFRL);
		#endif
}

/*
 * TIMER12 interrupt handler
*/

static irq_handler_t tim12_handler( unsigned int irq, struct pt_regs *regs ){
	
	if((*tim12_SR) & TIM12_CHECK_UIF){			// if it has been an overflow
	
		*tim12_SR &= 0x00;						// clears update interrupt flag
		high++;									// increments a variable
		
	}else{
		
		if((*tim12_SR) & TIM12_CHECK_CC1IF){	// if the counter value has been captured	
			
			value = (high << 16) | *tim12_COUNTER_VAL;		// stores the counter shifting the variable by 16 and adding to it the value inside the CCR1 register 
			high = 0;										
		}
	}		
	return (irq_handler_t)IRQ_HANDLED;
}

/*
 * Device open
 */
static int sample_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	/*
	 * One process at a time
	 */
	if (sample_lock > 0) 
	{
		ret = -EBUSY;
	}
	else
	{
		sample_lock++;

		/*
 	 	* Increment the module use counter
 	 	*/
		try_module_get(THIS_MODULE);

		#ifdef SAMPLE_DEBUG 
			printk( KERN_INFO "%s: %s\n", module_name, __func__ ); 
		#endif
	}

	return( ret );
}

/*
 * Device close
 */
static int sample_release(struct inode *inode, struct file *file)
{
	/*
 	 * Release device
 	 */
	sample_lock = 0;

	/*
 	 * Decrement module use counter
 	 */
	module_put(THIS_MODULE);

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( 0 );
}

/* 
 * Device read
 */
static ssize_t sample_read(struct file *filp, char *buffer,
			 size_t length, loff_t * offset)
{
	int ret = 1;

	memcpy(buffer, &value, sizeof(value));		//copy sizeof(value) bytes from value to buffer
 
	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( ret );
}

/* 
 * Device write
 */
static ssize_t sample_write(struct file *filp, const char *buffer,
			  size_t length, loff_t * offset)
{
	int ret = 0;
	int n_cycles;

	switch( mode )
	{
		case WRITE_HPERIOD:
			memcpy( &blk_hperiod, buffer, sizeof( blk_hperiod) );
			ret = 1;

			printk( KERN_INFO "%s: %s got blk_hperiod = %d msec\n", module_name, __func__, blk_hperiod );

			break;
		case WRITE_BLINK:
			n_cycles = (int)*buffer;

			work.delay = blk_hperiod;
			work.n_cycles = n_cycles;

			queue_work( my_wq, (struct work_struct *)&work );
			ret = n_cycles;
			break;
		case INVALID:
			ret = 0;
			break;
	}

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( ret );
}

static ssize_t sample_ioctl(struct inode *inode, struct file *filep, 
			    const unsigned int cmd, const unsigned long arg)
{
	int ret = 0;
#define WRITE_HPERIOD	0
#define	WRITE_BLINK	1

	switch( cmd )
	{
		case WRITE_HPERIOD:
			mode = WRITE_HPERIOD;	// The next write will set the blinking semiperiod
			break;
		case WRITE_BLINK:
			mode = WRITE_BLINK;	// The next write will start blinking for a given number of times
						// with the set blinking semiperiod;
			break;
		default:
			mode = INVALID;
			break;
	}

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( ret );
}


/*
 * Device operations
 */
static struct file_operations sample_fops = {
	.read = sample_read,
	.write = sample_write,
	.open = sample_open,
	.release = sample_release,
	.ioctl = sample_ioctl
};

static int __init sample_init_module(void)
{
	/*
 	 * Register device
 	 */
	int	ret;

	ret = register_chrdev(module_major, module_name, &sample_fops);
	if (ret < 0) {
		printk(KERN_INFO "%s: registering device %s with major %d failed with %d\n",
		       __func__, module_name, module_major, module_major );
		return( ret );
	}
	else
	{
		printk(KERN_INFO "%s: registering device %s with major %d\n",
		       __func__, module_name, module_major );
		
		if( request_irq(43, (irq_handler_t) tim12_handler, 0, module_name, NULL ) < 0 ){	//	Installs the interrupt handler for timer12 through its irq number 

			printk( KERN_INFO "%s: %s unable to register TIM12 irq\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}
		
		setup_registers();		// configures the registers needed

		my_wq = create_workqueue( "my_queue" );
		if( my_wq )
		{
			INIT_WORK( (struct work_struct *)&work, my_wq_function );
		}
	}
	
	return( ret );
}

static void __exit sample_cleanup_module(void){

	/*
	 * Unregister device
	 */
	unregister_chrdev(module_major, module_name);

	printk(KERN_INFO "%s: unregistering %s done\n", __func__, module_name );
}

module_init(sample_init_module);
module_exit(sample_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Massimo Violante, massimo.violante@polito.it");
MODULE_DESCRIPTION("Device Driver Example 1");

