/* Simple Raw HID functions for Linux - for use with Teensy RawHID example
 * http://www.pjrc.com/teensy/rawhid.html
 * Copyright (c) 2009 PJRC.COM, LLC
 *
 *  rawhid_open - open 1 or more devices
 *  rawhid_recv - receive a packet
 *  rawhid_send - send a packet
 *  rawhid_close - close a device
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above description, website URL and copyright notice and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

#include "hid.h"

// On Linux there are several options to access HID devices.
//
// libusb 0.1 - the only way that works well on all distributions
// libusb 1.0 - someday will become standard on most distributions
// hidraw driver - relatively new, not supported on many distributions (yet)
// hiddev driver - old, ubuntu, fedora, others dropping support
// usbfs - low level usb API: http://www.kernel.org/doc/htmldocs/usb.html#usbfs
//
// This code uses libusb 1.0, which provides better performance and a more modern API.
// It offers both synchronous and asynchronous transfers, and kernel-level buffering
// for better performance.
//

#define printf(...)  // comment this out for lots of info

// a list of all opened HID devices, so the caller can
// simply refer to them by number
typedef struct hid_struct hid_t;
static hid_t *first_hid = NULL;
static hid_t *last_hid = NULL;
struct hid_struct {
	libusb_device_handle *handle;
	int open;
	int iface;
	int ep_in;
	int ep_out;
	struct hid_struct *prev;
	struct hid_struct *next;
};


// private functions, not intended to be used from outside this file
static void add_hid(hid_t *h);
static hid_t * get_hid(int num);
static void free_all_hid(void);
static void hid_close(hid_t *hid);
static int hid_parse_item(uint32_t *val, uint8_t **data, const uint8_t *end);

//  rawhid_recv - receive a packet
//	Inputs:
//	num = device to receive from (zero based)
//	buf = buffer to receive packet
//	len = buffer's size
//	timeout = time to wait, in milliseconds
//	Output:
//	number of bytes received, or -1 on error
//
int rawhid_recv(int num, void *buf, int len, int timeout)
{
	hid_t *hid;
	int transferred;
	int r;

	hid = get_hid(num);
	if (!hid || !hid->open) return -1;

	r = libusb_interrupt_transfer(hid->handle, hid->ep_in | LIBUSB_ENDPOINT_IN,
								buf, len, &transferred, timeout);
								
	if (r == 0) return transferred;
	if (r == LIBUSB_ERROR_TIMEOUT) return 0;
	return -1;
}

//  rawhid_send - send a packet
//	Inputs:
//	num = device to transmit to (zero based)
//	buf = buffer containing packet to send
//	len = number of bytes to transmit
//	timeout = time to wait, in milliseconds
//	Output:
//	number of bytes sent, or -1 on error
//
int rawhid_send(int num, void *buf, int len, int timeout)
{
	hid_t *hid;
	int transferred;
	int r;

	hid = get_hid(num);
	if (!hid || !hid->open) return -1;

	if (hid->ep_out) {
		r = libusb_interrupt_transfer(hid->handle, hid->ep_out,
									buf, len, &transferred, timeout);
	} else {
		r = libusb_control_transfer(hid->handle,
								  LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
								  0x09, 0x200, hid->iface,
								  buf, len, timeout);
		transferred = r;
	}
	
	return (r >= 0) ? transferred : -1;
}

/**
 * Scans for the given vid and pid, and returns the number of devices found
 */
int rawhid_scan(int vid, int pid)
{
	libusb_device **devs;
	libusb_device *dev;
	struct libusb_device_descriptor desc;
	ssize_t cnt;
	int count = 0;
	int i = 0;
	
	libusb_init(NULL);
	cnt = libusb_get_device_list(NULL, &devs);
	if (cnt < 0) return 0;

	while ((dev = devs[i++]) != NULL) {
		if (libusb_get_device_descriptor(dev, &desc) < 0)
			continue;

		if (vid > 0 && desc.idVendor != vid) continue;
		if (pid > 0 && desc.idProduct != pid) continue;
		
		if (desc.bNumConfigurations > 0) {
			printf("device: vid=%04X, pid=%04X\n",
				   desc.idVendor, desc.idProduct);
			count++;
		}
	}

	libusb_free_device_list(devs, 1);
	return count;
}

//  rawhid_open - open 1 or more devices
//
//	Inputs:
//	max = maximum number of devices to open
//	vid = Vendor ID, or -1 if any
//	pid = Product ID, or -1 if any
//	usage_page = top level usage page, or -1 if any
//	usage = top level usage number, or -1 if any
//	Output:
//	actual number of devices opened
//
int rawhid_open(int max, int vid, int pid, int usage_page, int usage)
{
	libusb_device **devs;
	libusb_device *dev;
	libusb_device_handle *handle;
	struct libusb_device_descriptor desc;
	struct libusb_config_descriptor *conf_desc;
	const struct libusb_interface *iface;
	const struct libusb_interface_descriptor *iface_desc;
	const struct libusb_endpoint_descriptor *ep;
	ssize_t cnt;
	int i, j, k;
	int count = 0;
	uint8_t buf[1024];
	int len;
	uint32_t parsed_usage_page = 0, parsed_usage = 0;
	hid_t *hid;

	if (first_hid) free_all_hid();
	printf("rawhid_open, max=%d\n", max);
	if (max < 1) return 0;

	libusb_init(NULL);
	cnt = libusb_get_device_list(NULL, &devs);
	if (cnt < 0) return 0;

	for (i = 0; devs[i] != NULL && count < max; i++) {
		dev = devs[i];
		if (libusb_get_device_descriptor(dev, &desc) < 0)
			continue;

		if (vid > 0 && desc.idVendor != vid) continue;
		if (pid > 0 && desc.idProduct != pid) continue;

		printf("device: vid=%04X, pid=%04X, with %d iface\n",
			   desc.idVendor, desc.idProduct, desc.bNumInterfaces);

		if (libusb_get_config_descriptor(dev, 0, &conf_desc) < 0)
			continue;

		for (j = 0; j < conf_desc->bNumInterfaces && count < max; j++) {
			iface = &conf_desc->interface[j];
			for (k = 0; k < iface->num_altsetting; k++) {
				iface_desc = &iface->altsetting[k];

				printf("  type %d, %d, %d\n", iface_desc->bInterfaceClass,
					   iface_desc->bInterfaceSubClass, iface_desc->bInterfaceProtocol);

				if (iface_desc->bInterfaceClass != LIBUSB_CLASS_HID)
					continue;
				if (iface_desc->bInterfaceSubClass != 0)
					continue;
				if (iface_desc->bInterfaceProtocol != 0)
					continue;

				int ep_in = 0, ep_out = 0;
				for (int e = 0; e < iface_desc->bNumEndpoints; e++) {
					ep = &iface_desc->endpoint[e];
					if (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
						if (!ep_in) ep_in = ep->bEndpointAddress & 0x7F;
						printf("	IN endpoint %d\n", ep_in);
					} else {
						if (!ep_out) ep_out = ep->bEndpointAddress;
						printf("	OUT endpoint %d\n", ep_out);
					}
				}

				if (!ep_in) continue;

				if (libusb_open(dev, &handle) < 0) {
					printf("  unable to open device\n");
					continue;
				}

				printf("  hid interface (generic)\n");

				// Detach kernel driver if necessary
				if (libusb_kernel_driver_active(handle, j)) {
					printf("  in use by driver, attempting to detach\n");
					if (libusb_detach_kernel_driver(handle, j) < 0) {
						printf("  unable to detach from kernel\n");
						libusb_close(handle);
						continue;
					}
				}

				if (libusb_claim_interface(handle, j) < 0) {
					printf("  unable claim interface %d\n", j);
					libusb_close(handle);
					continue;
				}

				// Get HID report descriptor
				len = libusb_control_transfer(handle,
					LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_INTERFACE,
					LIBUSB_REQUEST_GET_DESCRIPTOR,
					(LIBUSB_DT_REPORT << 8), j,
					buf, sizeof(buf), 1000);

				printf("  descriptor, len=%d\n", len);

				if (len < 0) {
					libusb_release_interface(handle, j);
					libusb_close(handle);
					continue;
				}

				// Parse HID report descriptor
				uint8_t *p = buf;
				uint32_t val;
				while (p < buf + len) {
					int tag = hid_parse_item(&val, &p, buf + len);
					if (tag < 0) break;
					printf("  tag: %X, val %X\n", tag, val);
					if (tag == 4) parsed_usage_page = val;
					if (tag == 8) parsed_usage = val;
					if (parsed_usage_page && parsed_usage) break;
				}

				if ((!parsed_usage_page) || (!parsed_usage) ||
					(usage_page > 0 && parsed_usage_page != usage_page) ||
					(usage > 0 && parsed_usage != usage)) {
					libusb_release_interface(handle, j);
					libusb_close(handle);
					continue;
				}

				// Create and initialize HID device structure
				hid = (hid_t *)malloc(sizeof(hid_t));
				if (!hid) {
					libusb_release_interface(handle, j);
					libusb_close(handle);
					continue;
				}

				hid->handle = handle;
				hid->iface = j;
				hid->ep_in = ep_in;
				hid->ep_out = ep_out;
				hid->open = 1;
				add_hid(hid);
				count++;
			}
		}
		libusb_free_config_descriptor(conf_desc);
	}

	libusb_free_device_list(devs, 1);
	return count;
}

//  rawhid_close - close a device
//
//	Inputs:
//	num = device to close (zero based)
//	Output
//	(nothing)
//
// cppcheck-suppress unusedFunction
void rawhid_close(int num)
{
	hid_t *hid;

	hid = get_hid(num);
	if (!hid || !hid->open) return;
	hid_close(hid);
}

// Chuck Robey wrote a real HID report parser
// (chuckr@telenix.org) chuckr@chuckr.org
// http://people.freebsd.org/~chuckr/code/python/uhidParser-0.2.tbz
// this tiny thing only needs to extract the top-level usage page
// and usage, and even then is may not be truly correct, but it does
// work with the Teensy Raw HID example.
static int hid_parse_item(uint32_t *val, uint8_t **data, const uint8_t *end)
{
	const uint8_t *p = *data;
	uint8_t tag;
	const int table[4] = {0, 1, 2, 4};
	int len;

	if (p >= end) return -1;
	if (p[0] == 0xFE) {
		// long item, HID 1.11, 6.2.2.3, page 27
		if (p + 5 >= end || p + p[1] >= end) return -1;
		tag = p[2];
		*val = 0;
		len = p[1] + 5;
	} else {
		// short item, HID 1.11, 6.2.2.2, page 26
		tag = p[0] & 0xFC;
		len = table[p[0] & 0x03];
		if (p + len + 1 >= end) return -1;
		switch (p[0] & 0x03) {
			case 3: *val = p[1] | (p[2] << 8) | (p[3] << 16) | (p[4] << 24); break;
			case 2: *val = p[1] | (p[2] << 8); break;
			case 1: *val = p[1]; break;
			case 0: *val = 0; break;
		}
	}
	*data += len + 1;
	return tag;
}


static void add_hid(hid_t *h)
{
	if (!first_hid || !last_hid) {
		first_hid = last_hid = h;
		h->next = h->prev = NULL;
		return;
	}
	last_hid->next = h;
	h->prev = last_hid;
	h->next = NULL;
	last_hid = h;
}


static hid_t * get_hid(int num)
{
	hid_t *p;
	for (p = first_hid; p && num > 0; p = p->next, num--) ;
	return p;
}


static void free_all_hid(void)
{
    // cppcheck-suppress variableScope
	hid_t *p, *q;

	for (p = first_hid; p; p = p->next) {
		hid_close(p);
	}
	p = first_hid;
	while (p) { 
		q = p;
		p = p->next;
		free(q);
	}
	first_hid = last_hid = NULL;
}

static void hid_close(hid_t *hid)
{
	if (!hid->handle) return;
	
	libusb_release_interface(hid->handle, hid->iface);
	libusb_close(hid->handle);
	hid->handle = NULL;
}
