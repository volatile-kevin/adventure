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
	typedef struct tuxctl_ldisc_data {
		unsigned long magic;

		char rx_buf[TUXCTL_BUFSIZE];
		int rx_start, rx_end;

		char tx_buf[TUXCTL_BUFSIZE];
		int tx_start, tx_end;

	} tuxctl_ldisc_data_t;

	static unsigned char segments[16] = {
		0xE7, 0x6, 0xCB, 0x8F, 0x2E, 0xAD, 0xED, 0x86, 0xEF, 0xAE, 0xEE, 0x6D, 0xE1, 0x4F, 0xE9, 0xE8
	};




	// static saveState statePrev;
	char buttons;
	char LEDbuffer[6];

	int tux_init(struct tty_struct* tty){
		// enable interrupts from button press, should return MTCP_BIOC_EVT if button press
		char buffer[3];
		buffer[0] = MTCP_LED_USR;
		buffer[1] = MTCP_BIOC_ON;
		tuxctl_ldisc_put(tty, (const char*)buffer, 2);
		return 0;
	}



/*****************************************************************************************/
	int tux_buttons(struct tty_struct* tty, unsigned long arg){
		// int temp = arg & 0xFFFFFF00;
		// int state = stateCurr.buttons & 0xFF;
		// int *ptr = temp | state;
			int state = buttons;
			copy_to_user(arg, &state, 1);
		return 0;
	}

	int tux_set_led(struct tty_struct* tty, unsigned long arg){
		char whichLED = (arg >> 16) & 0xF;
		char whichDec = (arg >> 24) & 0xF;
		printk("%x \n", whichLED);
		char buffer[6];
		int LEDval[4];
		char LEDvalDEC[4];
		int i;
		LEDval[0] = arg & 0xF;
		LEDval[1] = (arg >> 4) & 0xF;
		LEDval[2] = (arg >> 8) & 0xF;
		LEDval[3] = (arg >> 12) & 0xF;
		printk("0, 1, 2, 3: %d, %d, %d, %d \n", LEDval[0], LEDval[1], LEDval[2], LEDval[3]);
		for(i = 0; i < 4; i++){
			if(whichDec & (1 << i)){
				LEDvalDEC[i] = segments[LEDval[i]] | 0x10;
			}

			else{
				LEDvalDEC[i] = segments[LEDval[i]];
				// printk("bitstring: %d \n", segments[LEDval[i]]);
			}
		}
		for(i = 0; i < 4; i++){
			if((whichLED & (1 << i)) == 0){
				LEDvalDEC[i] = 0;
			}
		}
		buffer[0] = MTCP_LED_SET;
		buffer[1] = 0xF;
		buffer[2] = LEDvalDEC[0];
		buffer[3] = LEDvalDEC[1];
		buffer[4] = LEDvalDEC[2];
		buffer[5] = LEDvalDEC[3];
		printk("bitstring: %x \n", LEDvalDEC[0]);
		printk("bitstring: %x \n", LEDvalDEC[1]);
		printk("bitstring: %x \n", LEDvalDEC[2]);
		printk("bitstring: %x \n", LEDvalDEC[3]);

		LEDbuffer[0] = MTCP_LED_SET;
		LEDbuffer[1] = whichLED;
		LEDbuffer[2] = LEDvalDEC[0];
		LEDbuffer[3] = LEDvalDEC[1];
		LEDbuffer[4] = LEDvalDEC[2];
		LEDbuffer[5] = LEDvalDEC[3];
		tuxctl_ldisc_put(tty, (const char*)buffer, 6);
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
		// printk("packet: %d, %d, %d \n", a, b, c);
		if(a == MTCP_RESET){
			// printk("asdlkjfhasd;kfjal;sdkjfla;skjsdfj!!!\n");
			buffer[0] = MTCP_BIOC_ON;
			tuxctl_ldisc_put(tty, (const char*)buffer, 1);
		}
		if(a == MTCP_BIOC_EVENT){
			temp1 = b & 0xF;
			// swap D and L in the packet
			// original: R D L U
			// final: R L D U
			bit1 = (c >> 1) & 1;
			bit2 = (c >> 2) & 1;
			temp2 = c ^ (((bit1 ^ bit2) << 1) | ((bit1 ^ bit2) << 2));
			buttons = (temp2 << 4) | temp1;
			// printk("buttons string: %x, %x, %x \n", stateCurr.buttons, temp1);
		}
		if(a == MTCP_ACK){
			printk("you ain't ever gonna finish this goddamn MP, dumbass \n");
		}
    // printk("packet : %x %x %x\n", a, b, c);
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
