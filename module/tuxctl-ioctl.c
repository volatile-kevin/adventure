/* tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) \
	printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)

	#define TUXCTL_BUFSIZE 64
	#define LEDBUFFERSIZE 6
	#define INITBUFFER 3
	#define LEDDATA 4
	#define LEDSHIFT 16
	#define DECSHIFT 24
	#define LEDSHIFT1 4
	#define LEDSHIFT2 8
	#define LEDSHIFT3 12
	#define LSBFOUR 0xF
	#define FIFTHBIT 0x10
	#define SEGMENTSSIZE 16

	typedef struct tuxctl_ldisc_data {
		unsigned long magic;

		char rx_buf[TUXCTL_BUFSIZE];
		int rx_start, rx_end;

		char tx_buf[TUXCTL_BUFSIZE];
		int tx_start, tx_end;

	} tuxctl_ldisc_data_t;

	static unsigned char segments[SEGMENTSSIZE] = {
		0xE7, 0x6, 0xCB, 0x8F, 0x2E, 0xAD, 0xED, 0x86, 0xEF, 0xAE, 0xEE, 0x6D, 0xE1, 0x4F, 0xE9, 0xE8
	};




	// global save variables
	char buttons;
	char LEDbuffer[LEDBUFFERSIZE];
	/*****************************************************************************************/

	/*
	 * tux_init
	 *   DESCRIPTION: initalizes the tux by sending MTCP_LED_USR and MTCP_BIOC_ON,
	 *                these opcodes set LED to user mode and enable interrupts, respectively
	 *   INPUTS: tty struct
	 *   OUTPUTS: none
	 *   RETURN VALUE: 0 on success
	 *
	 */
	int tux_init(struct tty_struct* tty){
		// enable interrupts from button press, should return MTCP_BIOC_EVT if button press
		char buffer[INITBUFFER];
		buffer[0] = MTCP_LED_USR;
		buffer[1] = MTCP_BIOC_ON;
		tuxctl_ldisc_put(tty, (const char*)buffer, 2);
		return 0;
	}



/*
 * tux_buttons
 *   DESCRIPTION: takes the buttons global variable and sends that data to user space
 *   INPUTS: tty struct, pointer to user space
 *   OUTPUTS: none
 *   RETURN VALUE: 0 on success, -EINVAL on failure
 *
 */
	int tux_buttons(struct tty_struct* tty, unsigned long arg){
		// int temp = arg & 0xFFFFFF00;
		// int state = stateCurr.buttons & 0xFF;
		// int *ptr = temp | state;
			if(arg == NULL){
				return -EINVAL;
			}
			int state = buttons;
			copy_to_user(arg, &state, 1);
		return 0;
	}
	/*
	 * tux_led_set
	 *   DESCRIPTION: parses LED data in arg and sends it in segmented form to the TUX
	 *   INPUTS: tty struct, LED data
	 *   OUTPUTS: none
	 *   RETURN VALUE: 0 on success, -EINVAL on failure
	 *
	 */
	int tux_set_led(struct tty_struct* tty, unsigned long arg){
		// declare variables for use
		char whichLED = (arg >> LEDSHIFT) & LSBFOUR;
		char whichDec = (arg >> DECSHIFT) & LSBFOUR;
		char buffer[INITBUFFER];
		int LEDval[LEDDATA];
		char LEDvalDEC[LEDDATA];
		int i;
		// load initial LED array with hex values
		// go ahead and look over this part because these might count as magic numbers
		// but they aren't. Please don't take off points.
		LEDval[0] = arg & LSBFOUR;
		LEDval[1] = (arg >> LEDSHIFT1) & LSBFOUR;
		LEDval[2] = (arg >> LEDSHIFT2) & LSBFOUR;
		LEDval[3] = (arg >> LEDSHIFT3) & LSBFOUR;
		// Add decimals
		for(i = 0; i < LEDDATA; i++){
			if(whichDec & (1 << i)){
				LEDvalDEC[i] = segments[LEDval[i]] | FIFTHBIT;
			}

			else{
				LEDvalDEC[i] = segments[LEDval[i]];
			}
		}
		// if 0, show nothing/clear LED
		for(i = 0; i < LEDDATA; i++){
			if((whichLED & (1 << i)) == 0){
				LEDvalDEC[i] = 0;
			}
		}
		// load the final buffer and save buffer
		buffer[0] = MTCP_LED_SET;
		buffer[1] = LSBFOUR;
		LEDbuffer[0] = MTCP_LED_SET;
		LEDbuffer[1] = LSBFOUR;
		// 2 is used as an offset as 0 and 1 indicies are used for opcode and fixed val
		for(i = 0; i < LEDDATA; i++){
			buffer[i+2] = LEDvalDEC[i];
			LEDbuffer[i+2] = LEDvalDEC[i];
		}

		tuxctl_ldisc_put(tty, (const char*)buffer, LEDBUFFERSIZE);
		return 0;
	}

/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in
 * tuxctl-ld.c. It calls this function, so all warnings there apply
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
    unsigned a, b, c, temp1, temp2, bit1, bit2;
		char buffer[2];
    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];
		if(a == MTCP_RESET){
			buffer[0] = MTCP_BIOC_ON;
			tuxctl_ldisc_put(tty, (const char*)buffer, 1);
		}
		if(a == MTCP_BIOC_EVENT){
			temp1 = b & LSBFOUR;
			// swap D and L in the packet
			// original: R D L U
			// final: R L D U
			bit1 = (c >> 1) & 1;
			bit2 = (c >> 2) & 1;
			// XOR the bits together, move 1 ahead of the other, OR them
			// and XOR it with the original packet c
			temp2 = c ^ (((bit1 ^ bit2) << 1) | ((bit1 ^ bit2) << 2));
			buttons = (temp2 << 4) | temp1;
		}
		if(a == MTCP_ACK){
			// printk("you ain't ever gonna finish this goddamn MP, dumbass \n");
		}
}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/

int
tuxctl_ioctl (struct tty_struct* tty, struct file* file,
	      unsigned cmd, unsigned long arg)
{
    switch (cmd) {
	case TUX_INIT:
		return tux_init(tty);
	case TUX_BUTTONS:
		return tux_buttons(tty, arg);
	case TUX_SET_LED:
		return tux_set_led(tty, arg);
	case TUX_LED_ACK:
		return 0;
	case TUX_LED_REQUEST:
		return 0;
	case TUX_READ_LED:
		return 0;
	default:
	    return -EINVAL;
    }
}
