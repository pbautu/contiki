/*
 * Copyright (c) 2013, Institute of Computer Aided Automation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \file
 *      IoTSyS example server. Most parts are based on the er-example-server by M. Kovatsch
 * \author
 *      Markus Jung <mjung@auto.tuwien.ac.at>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"

#include "erbium.h"

#define RES_TEMP 1
#define RES_ACC 0
#define RES_BUTTON 0
#define RES_LEDS 0
#define RES_BATTERY 1

#define GROUP_COMM_ENABLED 1
#define UDP_PORT 5683

#define TEMP_ONLY_SEND_WHEN_NEW_VALUE 0

#if RES_TEMP
  /* Z1 temperature sensor */
  #include "dev/i2cmaster.h"
  #include "dev/tmp102.h"
#endif // RES_TEMP

#if RES_ACC || RES_BUTTON
/* Z1 accelorometer */
#include "dev/adxl345.h"
#endif // RES_ACC

#if RES_LEDS
/* Z1 leds */
#include "dev/leds.h"
#endif // RES_LEDS

#if RES_BATTERY
#include "dev/battery-sensor.h"
#endif

/* For CoAP-specific example: not required for normal RESTful Web service. */
#if WITH_COAP == 3
#include "er-coap-03.h"
#elif WITH_COAP == 7
#include "er-coap-07.h"
#elif WITH_COAP == 12
#include "er-coap-12.h"
#elif WITH_COAP == 13
#include "er-coap-13.h"
#include "er-coap-13-engine.h"
#else
#warning "IoTSyS server example"
#endif /* CoAP-specific example */

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]",(lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3],(lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif

#define CHUNKS_TOTAL        1024

#define TEMP_MSG_MAX_SIZE       140   // more than enough right now
#define TEMP_BUFF_MAX           7     // -234.6\0
#define BUTTON_MSG_MAX_SIZE     140   // more than enough right now
#define LED_MSG_MAX_SIZE        240   // more than enough right now
#define BUTTON_BUFF_MAX         6     // true\0 false\0
#define ACC_MSG_MAX_SIZE        140   // more than enough right now
#define ACC_BUFF_MAX            11    // freefall\0 activity\0 inactivity\0
#define BATTERY_MSG_MAX_SIZE    140   // more than enough right now
#define BATTERY_BUFF_MAX        4     // 0\0 ... 100\0

// Group communication definition
#define MAX_GC_HANDLERS 2
#define MAX_GC_GROUPS 5

typedef void (*gc_handler) (char*);

#define PUT_BUFFER_SIZE 140
/******************************************************************************/
/* typedefs, enums  ***********************************************************/
/******************************************************************************/
#if RES_ACC
typedef enum {
	ACC_INACTIVITY, ACC_ACTIVITY, ACC_FREEFALL
} acceleration_t;
#endif

// Data structure for storing group communication assignments.
// It is intended to store only the group identifier
// of a transient link-local scope multicast address (FF:12::XXXX)
typedef struct {
	int group_identifier;
	gc_handler handlers[MAX_GC_HANDLERS];
} gc_handler_t;

/******************************************************************************/
/* globals ********************************************************************/
/******************************************************************************/
#if GROUP_COMM_ENABLED
	static struct simple_udp_connection broadcast_connection;
#endif

#if RES_TEMP
char tempstring[TEMP_BUFF_MAX];
#endif


#if RES_BUTTON
char buttonstring[BUTTON_BUFF_MAX];
uint8_t virtual_button;
uint8_t acc_register_tap;
process_event_t event_tap;
#endif // RES_BUTTON

#if RES_ACC
char accstring[ACC_BUFF_MAX];
acceleration_t acc;
uint8_t acc_register_acc;
process_event_t event_acc;
#endif // RES_ACC

char payload_buffer[PUT_BUFFER_SIZE];

gc_handler_t gc_handlers[MAX_GC_GROUPS];

#if RES_LEDS
int led_red = 0;
int led_blue = 0;
int led_green = 0;
#endif

#if RES_BATTERY
char batterystring[BATTERY_BUFF_MAX];
#endif

/******************************************************************************/
/* helper functions ***********************************************************/
/******************************************************************************/

// creates an IPv6 address from the provided string
// note: the provided string is manipulated.
void get_ipv6_multicast_addr(char* input, uip_ip6addr_t* address){
	// first draft, assume an IPv6 address with explicit notation like FF12:0000:0000:0000:0000:0000:0000:0001

	// in this case the address shall be an char array with 32 (hex chars) + 7 (: delim) + 1 string delimiter
	// replace all : with a whitespace
	char* curChar;
	char* pEnd;
	int addr1, addr2,addr3, addr4, addr5, addr6, addr7, addr8;

	// move to the beginning of the IPv6 address --> assume it starts with FF
	input = strstr(input, "FF");

	curChar = strchr(input, ':');

	while (curChar != NULL)
	{
	   *curChar = ' '; // replace : with space
	   curChar=strchr(curChar+1,':');
	}


	addr1 = strtol(input,&pEnd,16); // FF12 block
	addr2 = strtol(pEnd, &pEnd,16); // 0000 block
	addr3 = strtol(pEnd, &pEnd,16); // 0000 block
	addr4 = strtol(pEnd, &pEnd,16); // 0000 block
	addr5 = strtol(pEnd, &pEnd,16); // 0000 block
	addr6 = strtol(pEnd, &pEnd,16); // 0000 block
	addr7 = strtol(pEnd, &pEnd,16); // 0000 block
	addr8 = strtol(pEnd, &pEnd,16); // 0000 block

	// create ipv6 address with 16 bit words
	uip_ip6addr(address,addr1, addr2, addr3, addr4, addr5, addr6, addr7, addr8); // 0001 block
}


int get_bool_value_obix(char* obix_object){
	PRINTF("Obix object is: %s\n", obix_object);
	// value can either be true or false
	if(strstr(obix_object, "true") != NULL){
		PRINTF("obix value is true\n");
		return 1;
	}
	PRINTF("obix value is false\n");
	return 0;
}

void send_message(const char* message, const uint16_t size_msg, void *request,
		void *response, uint8_t *buffer, uint16_t preferred_size,
		int32_t *offset) {
	PRINTF("Send Message: Size = %u, Offset = %ld\n", size_msg, *offset);
	PRINTF("Preferred Size: %d\n", preferred_size);

	uint16_t length;
	char *err_msg;

	length = size_msg - *offset;

	printf("length is: %d\n", length);

	if (length <= 0) {
		PRINTF("AHOYHOY?!\n");
		REST.set_response_status(response, REST.status.INTERNAL_SERVER_ERROR);
		err_msg = "calculation of message length error";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	if (preferred_size < 0 || preferred_size > REST_MAX_CHUNK_SIZE) {
		preferred_size = REST_MAX_CHUNK_SIZE;
		PRINTF(
				"Preferred size set to REST_MAX_CHUNK_SIZE = %d\n", preferred_size);
	}

	if (length > preferred_size) {
		PRINTF("Message still larger then preferred_size, truncating...\n");
		length = preferred_size;
		PRINTF("Length is now %u\n", length);

		memcpy(buffer, message + *offset, length);

		/* Truncate if above CHUNKS_TOTAL bytes. */
		if (*offset + length > CHUNKS_TOTAL) {
			PRINTF("Reached CHUNKS_TOTAL, truncating...\n");
			length = CHUNKS_TOTAL - *offset;
			PRINTF("Length is now %u\n", length);
			PRINTF("End of resource, setting offset to -1\n");
			*offset = -1;
		} else {
			/* IMPORTANT for chunk-wise resources: Signal chunk awareness to REST engine. */
			*offset += length;
			PRINTF("Offset refreshed to %ld\n", *offset);
		}
	} else {
		memcpy(buffer, message + *offset, length);
		*offset = -1;
	}

	PRINTF(
			"Sending response chunk: length = %u, offset = %ld\n", length, *offset);

	REST.set_header_etag(response, (uint8_t *) &length, 1);
	REST.set_response_payload(response, buffer, length);
}

#if GROUP_COMM_ENABLED
coap_packet_t request;

void send_coap_multicast(char* payload, size_t msgSize, uip_ip6addr_t* mc_address){
	 coap_init_message(&request, COAP_TYPE_NON, COAP_PUT, 0 );
	 coap_set_payload(&request, (uint8_t *)payload, msgSize);
	 coap_set_header_uri_path(&request, "");
	 coap_simple_request(mc_address, 5683, &request);
	 printf("\n--Done--\n");
}

void send_group_update(char* payload, size_t msgSize, gc_handler handler ){
	printf("sending group update\n");
	int i,l=0;
	uip_ip6addr_t gc_address;

	for(i = 0; i < MAX_GC_GROUPS; i++){
		// adding gc handler
		for(l=0; l < MAX_GC_HANDLERS; l++){
			if(gc_handlers[i].handlers[l] == handler ){
				printf("Sending update to group identifier %d", gc_handlers[i].group_identifier);
				uip_ip6addr(&gc_address, 0xff12, 0, 0, 0, 0, 0, 0, gc_handlers[i].group_identifier);
				send_coap_multicast(payload, msgSize, &gc_address);
			}
		}

	}
}

void extract_group_identifier(uip_ip6addr_t* ipv6Address, uint16_t* groupIdentifier ){
	*groupIdentifier = 0;
	*groupIdentifier =  ((uint8_t *)ipv6Address)[14];
	*groupIdentifier <<= 8;
	*groupIdentifier += ((uint8_t *)ipv6Address)[15];
}

void join_group(int groupIdentifier, gc_handler handler  ){
	int i,l=0;
	// use last 32 bits
	for(i = 0; i < MAX_GC_GROUPS; i++){
		if(gc_handlers[i].group_identifier == 0 || gc_handlers[i].group_identifier == groupIdentifier){ // free slot or same slot

			gc_handlers[i].group_identifier = groupIdentifier;
			//gc_handlers[i].group_identifier &= (groupAddress.u16[6] << 16);
			PRINTF("Assigned slot: %d\n", gc_handlers[i].group_identifier);

			// adding gc handler
			for(l=0; l < MAX_GC_HANDLERS; l++){
				if(gc_handlers[i].handlers[l] == NULL ||  gc_handlers[i].handlers[l] == &handler ){
					gc_handlers[i].handlers[l] = handler;
					PRINTF("(Re-)Assigned callback on slot %d\n", l);
					break;
				}
			}
			break;
		}
	}
}

void leave_group(int groupIdentifier, gc_handler handler){
	int i,l=0;
	for(i = 0; i < MAX_GC_GROUPS; i++){
		if(gc_handlers[i].group_identifier == groupIdentifier){ // free slot or same slot

			gc_handlers[i].group_identifier = groupIdentifier;
			//gc_handlers[i].group_identifier &= (groupAddress.u16[6] << 16);
			PRINTF("Found slot: %d\n", gc_handlers[i].group_identifier);

			// adding gc handler
			for(l=0; l < MAX_GC_HANDLERS; l++){

				if(gc_handlers[i].handlers[l] == handler ){
					gc_handlers[i].handlers[l] = NULL;
					PRINTF("Removed callback from slot %d\n", l);
					break;
				}
			}
			break;
		}
	}
}
#endif // GROUP_COMM_ENABLED

#if RES_TEMP
int temp_to_buff(char* buffer) {
	int16_t tempint;
	uint16_t tempfrac;
	int16_t raw;
	uint16_t absraw;
	int16_t sign = 1;

	/* get temperature */
	raw = tmp102_read_temp_raw();
	absraw = raw;

	if (raw < 0) {
		// Perform 2C's if sensor returned negative data
		absraw = (raw ^ 0xFFFF) + 1;
		sign = -1;
	}

	tempint = (absraw >> 8) * sign;
	tempfrac = ((absraw >> 4) % 16) * 625; // Info in 1/10000 of degree
	tempfrac = ((tempfrac) / 1000); // Round to 1 decimal

	return snprintf(buffer, TEMP_BUFF_MAX, "%d.%1d", tempint, tempfrac);
}

int temp_to_default_buff() {
	return temp_to_buff(tempstring);
}

uint8_t create_response_datapoint_temperature(char *buffer,	int asChild) {
	size_t size_temp;
	int size_msgp1, size_msgp2;
	const char *msgp1, *msgp2;
	uint8_t size_msg;

	if (asChild) {
		msgp1 =
				"<real href=\"temp/value\" units=\"obix:units/celsius\" val=\"";
		size_msgp1 = 56;
    msgp2 = "\"/>";
    size_msgp2 = 3;
	} else {
		msgp1 = "<real href=\"value\" units=\"obix:units/celsius\" val=\"";
		size_msgp1 = 51;
    msgp2 = "\"/>\0";
    size_msgp2 = 4;
	}


	if ((size_temp = temp_to_default_buff()) < 0) {
		PRINTF("Error preparing temperature string!\n");
		return 0;
	}

	size_msg = size_msgp1 + size_msgp2 + size_temp;

	memcpy(buffer, msgp1, size_msgp1);
	memcpy(buffer + size_msgp1, tempstring, size_temp);
	memcpy(buffer + size_msgp1 + size_temp, msgp2, size_msgp1);

	return size_msg;
}

uint8_t create_response_object_temperature(char *buffer) {
	size_t size_datapoint;
	int size_msgp1, size_msgp2;
	const char *msgp1, *msgp2;
	uint8_t size_msg;

	msgp1 = "<obj href=\"temp\" is=\"iot:TemperatureSensor\">";
	msgp2 = "</obj>\0";
	size_msgp1 = 44;
	size_msgp2 = 7;

	memcpy(buffer, msgp1, size_msgp1);
	// creates real data point and copies content to message buffer
	size_datapoint = create_response_datapoint_temperature(buffer + size_msgp1, 1);

	memcpy(buffer + size_msgp1 + size_datapoint, msgp2, size_msgp2);

	size_msg = size_msgp1 + size_msgp2 + size_datapoint;

	return size_msg;
}

/*
 * Example for an oBIX temperature sensor.
 */
RESOURCE(temp, METHOD_GET, "temp", "title=\"Temperature Sensor\"");

void temp_handler(void* request, void* response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset) {
	PRINTF(
			"temp_handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);
	/* Save the message as static variable, so it is retained through multiple calls (chunked resource) */
	static char message[TEMP_MSG_MAX_SIZE];
	static uint8_t size_msg;

	const uint16_t *accept = NULL;
	int num = 0;
	char *err_msg;

	/* Check the offset for boundaries of t        he resource data. */
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		/* A block error message should not exceed the minimum block size (16). */
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	/* compute message once */
	if (*offset <= 0) {
		/* decide upon content-format */
		num = REST.get_header_accept(request, &accept);

		REST.set_header_content_type(response, REST.type.APPLICATION_XML);

		if ((size_msg = create_response_object_temperature(message)) <= 0) {
			PRINTF("ERROR while creating message!\n");
			REST.set_response_status(response,
					REST.status.INTERNAL_SERVER_ERROR);
			err_msg = "ERROR while creating message :\\";
			REST.set_response_payload(response, err_msg, strlen(err_msg));
			return;
		}
	}

	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}

/*
 * Example for an oBIX temperature sensor.
 */
PERIODIC_RESOURCE(value, METHOD_GET, "temp/value",
		"title=\"Temperature Value;obs\"", 30*CLOCK_SECOND);

void value_handler(void* request, void* response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset) {

	PRINTF(
			"temp_value_handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);
	/* Save the message as static variable, so it is retained through multiple calls (chunked resource) */
    char message[TEMP_MSG_MAX_SIZE];
	uint8_t size_msg;

	const uint16_t *accept = NULL;
	char *err_msg;

	/* Check the offset for boundaries of the resource data. */
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		/* A block error message should not exceed the minimum block size (16). */
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	REST.set_header_content_type(response, REST.type.APPLICATION_XML);

	if ((size_msg = create_response_datapoint_temperature(message, 0)) <= 0) {
		PRINTF("ERROR while creating message!\n");
		REST.set_response_status(response,
				REST.status.INTERNAL_SERVER_ERROR);
		err_msg = "ERROR while creating message :\\";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}

/*
 * Additionally, a handler function named [resource name]_handler must be implemented for each PERIODIC_RESOURCE.
 * It will be called by the REST manager process with the defined period.
 */
void value_periodic_handler(resource_t *r) {

	static char new_value[TEMP_BUFF_MAX];
	static char buffer[TEMP_MSG_MAX_SIZE];
	static uint8_t obs_counter = 0;
	size_t size_msg;

	if (temp_to_buff(new_value) <= 0) {
		PRINTF("ERROR while creating message!\n");
		return;
	}

#if TEMP_ONLY_SEND_WHEN_NEW_VALUE
	if (strncmp(new_value, tempstring, TEMP_BUFF_MAX) != 0) {
			PRINTF("INFO current temperature value equals previous temperature value; returning!\n");
			return;
		}
#endif

	if ((size_msg = create_response_datapoint_temperature( buffer, 0)) <= 0) {
		PRINTF("ERROR while creating message!\n");
		return;
	}

		/* Build notification. */
		coap_packet_t notification[1]; /* This way the packet can be treated as pointer as usual. */
		coap_init_message(notification, COAP_TYPE_NON, CONTENT_2_05, 0);
		coap_set_payload(notification, buffer, size_msg);

		/* Notify the registered observers with the given message type, observe option, and payload. */
		REST.notify_subscribers(r, obs_counter, notification);
}


#endif // ENABLE TEMP

#if RES_BUTTON
int button_to_buff(char* buffer) {
	if (virtual_button) {
		return snprintf(buffer, BUTTON_BUFF_MAX, "true");
	}
	return snprintf(buffer, BUTTON_BUFF_MAX, "false");
}

int button_to_default_buff() {
	return button_to_buff(buttonstring);
}

uint8_t create_response_datapoint_button(char *buffer, int asChild) {
	size_t size_button;
	int size_msgp1, size_msgp2;
	const char *msgp1, *msgp2;
	uint8_t size_msg;

	msgp1 = "<bool val=\"";
	size_msgp1 = 11;
	msgp2 = "\"/>\0";
	size_msgp2 = 4;

	if ((size_button = button_to_default_buff()) < 0) {
		PRINTF("Error preparing button string!\n");
		return 0;
	}

	size_msg = size_msgp1 + size_msgp2 + size_button;

	memcpy(buffer, msgp1, size_msgp1);
	memcpy(buffer + size_msgp1, buttonstring, size_button);
	memcpy(buffer + size_msgp1 + size_button, msgp2, size_msgp2);

	return size_msg;
}

uint8_t create_response_object_button(char *buffer) {
	size_t size_datapoint;
	int size_msgp1, size_msgp2;
	const char *msgp1, *msgp2;
	uint8_t size_msg;

	msgp1 = "<obj href=\"button\" is=\"iot:PushButton\">";
	msgp2 = "</obj>\0";
	size_msgp1 = 39;
	size_msgp2 = 7;

	memcpy(buffer, msgp1, size_msgp1);
	// creates bool data point and copies content to message buffer
	size_datapoint = create_response_datapoint_button(buffer + size_msgp1, 1);

	memcpy(buffer + size_msgp1 + size_datapoint, msgp2, size_msgp2);

	size_msg = size_msgp1 + size_msgp2 + size_datapoint;

	return size_msg;
}

/*
 * Handles group communication updates for the button.
 */
void button_group_commhandler(char* payload){
	// this is just a place holder function.
	// the function pointer will be used by the acc driver for the button
	// to find out the IPv6 address to which an update should be sent
}

/*
 * Example for an oBIX button sensor.
 */
RESOURCE(button, METHOD_GET , "button", "title=\"Button Sensor\"");

void button_handler(void* request, void* response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset) {
	PRINTF(
			"button_handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);
	/* Save the message as static variable, so it is retained through multiple calls (chunked resource) */
	static char message[BUTTON_MSG_MAX_SIZE];
	static uint8_t size_msg;

	const uint16_t *accept = NULL;
	int num = 0;
	char *err_msg;


	/* Check the offset for boundaries of the resource data. */
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		/* A block error message should not exceed the minimum block size (16). */
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	REST.set_header_content_type(response, REST.type.APPLICATION_XML);

	if ((size_msg = create_response_object_button(message))
			<= 0) {
		PRINTF("ERROR while creating message!\n");
		REST.set_response_status(response,
				REST.status.INTERNAL_SERVER_ERROR);
		err_msg = "ERROR while creating message :\\";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}

/*
 * Example for an event resource.
 */
EVENT_RESOURCE(button_value, METHOD_GET , "button/value",
		"title=\"VButton Value\";obs");
SUB_RESOURCE(button_value_gc, METHOD_POST | METHOD_GET | HAS_SUB_RESOURCES, "button/value/groupComm", "", button_value);

void button_value_handler(void* request, void* response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset) {
	PRINTF("button_value_handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);
	/* Save the message as static variable, so it is retained through multiple calls (chunked resource) */
	static char message[BUTTON_MSG_MAX_SIZE];
	static uint8_t size_msg;

	char *err_msg;

#if GROUP_COMM_ENABLED
	const uint8_t *incoming = NULL;
	static size_t payload_len = 0;
	int newVal = 0;
	uip_ip6addr_t groupAddress;
	gc_handler handler = &button_group_commhandler;

	int16_t groupIdentifier = 0;

	const char *uri_path = NULL;
	int len = REST.get_url(request, &uri_path);

	// for PUT and POST request we need to process the payload content
	if( REST.get_method_type(request) == METHOD_POST){
		payload_len = REST.get_request_payload(request, &incoming);
		memcpy(payload_buffer, incoming, payload_len);
	}

	if(strstr(uri_path, "joinGroup") && REST.get_method_type(request) == METHOD_POST ){
		printf("Join group called.\n");
		get_ipv6_multicast_addr(payload_buffer, &groupAddress);
		PRINT6ADDR(&groupAddress);
		extract_group_identifier(&groupAddress, &groupIdentifier);
		printf("\n group identifier: %d\n", groupIdentifier);
		join_group(groupIdentifier, handler);
	}
	else if(strstr(uri_path, "leaveGroup") && REST.get_method_type(request) == METHOD_POST){
		printf("Leave group called.\n");
		get_ipv6_multicast_addr(payload_buffer, &groupAddress);
		PRINT6ADDR(&groupAddress);
		extract_group_identifier(&groupAddress, &groupIdentifier);
		printf("\n group identifier: %d\n", groupIdentifier);
		leave_group(groupIdentifier,  handler);
	}
#endif // GROUP_COMM_ENABLED

	/* Check the offset for boundaries of t        he resource data. */
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		/* A block error message should not exceed the minimum block size (16). */
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	/* compute message once */
	if (*offset <= 0) {
		REST.set_header_content_type(response, REST.type.APPLICATION_XML);

		if ((size_msg = create_response_datapoint_button( message, 0)) <= 0) {
			PRINTF("ERROR while creating message!\n");
			REST.set_response_status(response,
					REST.status.INTERNAL_SERVER_ERROR);
			err_msg = "ERROR while creating message :\\";
			REST.set_response_payload(response, err_msg, strlen(err_msg));
			return;
		}
	}


	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}

/* Additionally, a handler function named [resource name]_event_handler must be implemented for each PERIODIC_RESOURCE defined.
 * It will be called by the REST manager process with the defined period. */
void button_value_event_handler(resource_t *r) {
	static char buffer[BUTTON_MSG_MAX_SIZE];
	size_t size_msg;
	static uint8_t button_presses = 0;

	if (!(acc_register_tap & ADXL345_INT_TAP)) {
		return;
	}
	virtual_button = !virtual_button;

	if ((size_msg = create_response_datapoint_button( buffer, 0)) <= 0) {
		PRINTF("ERROR while creating message!\n");
		return;
	}

	/* Build notification. */
	coap_packet_t notification[1]; /* This way the packet can be treated as pointer as usual. */
	coap_init_message(notification, COAP_TYPE_NON, CONTENT_2_05, 0);
	coap_set_payload(notification, buffer, size_msg);

	/* Notify the registered observers with the given message type, observe option, and payload. */
	REST.notify_subscribers(r, button_presses, notification);

#if GROUP_COMM_ENABLED
	// check for registered group communication variables
	send_group_update(buffer, size_msg, &button_group_commhandler);

#endif
}
#endif //RES_BUTTON

#if RES_ACC
int acc_to_buff(char* buffer) {
	if (acc == ACC_INACTIVITY) {
		return snprintf(buffer, ACC_BUFF_MAX, "inactivity");
	} else if (acc == ACC_ACTIVITY) {
		return snprintf(buffer, ACC_BUFF_MAX, "activity");
	}
	return snprintf(buffer, ACC_BUFF_MAX, "freefall");
}

int acc_to_default_buff() {
	return acc_to_buff(accstring);
}

uint8_t create_response_datapoint_acc(char *buffer, int asChild) {
	size_t size_acc;
	int size_msgp1, size_msgp2;
	const char *msgp1, *msgp2;
	uint8_t size_msg;


	if (asChild) {
		msgp1 = "<bool href=\"acc/active\" val=\"";
		size_msgp1 = 29;
    msgp2 = "\"/>";
    size_msgp2 = 3;
	} else {
		msgp1 = "<bool href=\"active\" val=\"";
		size_msgp1 = 25;
    msgp2 = "\"/>\0";
    size_msgp2 = 4;
	}


	if ((size_acc = acc_to_default_buff()) < 0) {
		PRINTF("Error preparing acc string!\n");
		return 0;
	}

	size_msg = size_msgp1 + size_msgp2 + size_acc;

	memcpy(buffer, msgp1, size_msgp1);
	memcpy(buffer + size_msgp1, accstring, size_acc);
	memcpy(buffer + size_msgp1 + size_acc, msgp2, size_msgp2);

	return size_msg;
}

uint8_t create_response_object_acc(char *buffer) {
	size_t size_datapoint;
	int size_msgp1, size_msgp2;
	const char *msgp1, *msgp2;
	uint8_t size_msg;

	msgp1 = "<obj href=\"acc\" is=\"iot:ActivitySensor\">";
	msgp2 = "</obj>\0";
	size_msgp1 = 40;
	size_msgp2 = 7;

	memcpy(buffer, msgp1, size_msgp1);
	// creates data point and copies content to message buffer
	size_datapoint = create_response_datapoint_acc(buffer + size_msgp1, 1);

	memcpy(buffer + size_msgp1 + size_datapoint, msgp2, size_msgp2);

	size_msg = size_msgp1 + size_msgp2 + size_datapoint;

	return size_msg;
}

/*
 * Example for an oBIX acceleration sensor.
 */
RESOURCE(acc, METHOD_GET, "acc", "title=\"Acceleration Sensor\"");

void acc_handler(void* request, void* response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset) {
	PRINTF(
			"acc_handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);
	/* Save the message as static variable, so it is retained through multiple calls (chunked resource) */
	static char message[BUTTON_MSG_MAX_SIZE];
	static uint8_t size_msg;

	const uint16_t *accept = NULL;
	int num = 0;
	char *err_msg;

	/* Check the offset for boundaries of t        he resource data. */
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		/* A block error message should not exceed the minimum block size (16). */
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	/* compute message once */
	if (*offset <= 0) {
		/* decide upon content-format */
		num = REST.get_header_accept(request, &accept);

		REST.set_header_content_type(response, REST.type.APPLICATION_XML);

		if ((size_msg = create_response_object_acc(message))
				<= 0) {
			PRINTF("ERROR while creating message!\n");
			REST.set_response_status(response,
					REST.status.INTERNAL_SERVER_ERROR);
			err_msg = "ERROR while creating message :\\";
			REST.set_response_payload(response, err_msg, strlen(err_msg));
			return;
		}
	}

	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}

/*
 * Accelerometer.
 */
EVENT_RESOURCE(event_acc, METHOD_GET, "acc/value",
		"title=\"Acceleration Value\";obs");

void event_acc_handler(void* request, void* response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset) {
	PRINTF(
			"event_acc_handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);
	/* Save the message as static variable, so it is retained through multiple calls (chunked resource) */
	static char message[ACC_MSG_MAX_SIZE];
	static uint8_t size_msg;

	char *err_msg;

	/* Check the offset for boundaries of t        he resource data. */
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		/* A block error message should not exceed the minimum block size (16). */
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}


	/* decide upon content-format */
	REST.set_header_content_type(response, REST.type.APPLICATION_XML);

	if ((size_msg = create_response_datapoint_acc(message, 0)) <= 0) {
		PRINTF("ERROR while creating message!\n");
		REST.set_response_status(response,
				REST.status.INTERNAL_SERVER_ERROR);
		err_msg = "ERROR while creating message :\\";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}

/* Additionally, a handler function named [resource name]_event_handler must be implemented for each PERIODIC_RESOURCE defined.
 * It will be called by the REST manager process with the defined period. */
void event_acc_event_handler(resource_t *r) {
	static char buffer[ACC_MSG_MAX_SIZE];
	size_t size_msg;
	static uint8_t acc_events = 0;

	if (acc_register_acc & ADXL345_INT_INACTIVITY) {
		acc = ACC_INACTIVITY;
	} else if (acc_register_acc & ADXL345_INT_FREEFALL) {
		acc = ACC_FREEFALL;
	} else if (acc_register_acc & ADXL345_INT_ACTIVITY) {
		acc = ACC_ACTIVITY;
	} else {
		return;
	}

	if ((size_msg = create_response_datapoint_acc(buffer, 0)) <= 0) {
		PRINTF("ERROR while creating message!\n");
		return;
	}

	/* Build notification. */
	coap_packet_t notification[1]; /* This way the packet can be treated as pointer as usual. */
	coap_init_message(notification, COAP_TYPE_NON, CONTENT_2_05, 0);
	coap_set_payload(notification, buffer, size_msg);

	/* Notify the registered observers with the given message type, observe option, and payload. */
	REST.notify_subscribers(r, acc_events, notification);
}
#endif //RES_ACC

#if RES_LEDS
/* Leds */
uint8_t create_response_datapoint_led(char *buffer,
		int asChild, int color) {
	int size_msgp1, size_msgp2, size_msgp3, size_color;
	const char *msgp1, *msgp2, *msgp3, *msgp_red, *msgp_blue, *msgp_green;
	char *msgp_color; // will point to red, blue or green
	int value = 0; // on or off, depending on led
	const char *msg_true;
	const char *msg_false;
	char *msgp_value;
	int size_msgp_value = 0;

	msg_true = "true";
	msg_false = "false";

	msgp_red = "red";
	msgp_blue = "blue";
	msgp_green = "green";

	uint8_t size_msg;

	PRINTF("Creating response datapoint led asChild: %d color: %d\n", asChild, color);


	if (asChild) {
		msgp1 =	"<bool href=\"leds/";
		size_msgp1 = 17;

	} else {
		msgp1 = "<bool href=\"";
		size_msgp1 = 12;
	}
	msgp2 = "\" val=\"";
	size_msgp2 = 7;
	msgp3 = "\"/>\0";
	size_msgp3 = 4;

	memcpy(buffer, msgp1, size_msgp1);

	if(color == 0){ // red
		msgp_color = msgp_red;
		size_color = 3;
		if(led_red == 1){
			value = 1;
		}
	} else if(color == 1){
		msgp_color = msgp_blue;
		size_color = 4;
		if(led_blue == 1){
		  value = 1;
		}
	} else if(color == 2){
		msgp_color = msgp_green;
		size_color = 5;

		if(led_green == 1){
			value = 1;
		}
	}

	if( value == 1){
		msgp_value = msg_true;
		size_msgp_value = 4;
	}
	else{
		msgp_value = msg_false;
		size_msgp_value = 5;
	}
	memcpy(buffer + size_msgp1, msgp_color, size_color);
	memcpy(buffer + size_msgp1 + size_color, msgp2, size_msgp2);

	memcpy(buffer + size_msgp1 + size_color + size_msgp2, msgp_value, size_msgp_value);

	memcpy(buffer + size_msgp1 + size_color + size_msgp2 + size_msgp_value, msgp3, size_msgp3);

	size_msg = size_msgp1 + size_msgp2 + size_msgp_value + size_color + size_msgp3;

	return size_msg;
}

uint8_t create_response_object_led(char *buffer) {
	int size_datapoint_red;
	int size_datapoint_green;
	int size_datapoint_blue;
	int size_msgp1, size_msgp2;
	const char *msgp1, *msgp2;
	uint8_t size_msg;

	PRINTF("Creating response object led called\n");

	msgp1 = "<obj href=\"leds\" is=\"iot:LedsActuator\">";
	msgp2 = "</obj>\0";
	size_msgp1 = 39;
	size_msgp2 = 7;

	memcpy(buffer, msgp1, size_msgp1);
	// creates bool data point and copies content to message buffer
	size_datapoint_red = create_response_datapoint_led(buffer + size_msgp1, 0, 0);
	size_datapoint_green = create_response_datapoint_led(buffer + size_msgp1 + size_datapoint_red, 0,1);

	size_datapoint_blue = create_response_datapoint_led(buffer + size_msgp1 + size_datapoint_red + size_datapoint_blue + size_datapoint_green, 0,2);

	memcpy(buffer + size_msgp1 + size_datapoint_red + size_datapoint_green + size_datapoint_blue, msgp2, size_msgp2);

	size_msg = size_msgp1 + size_msgp2 + size_datapoint_red + size_datapoint_green + size_datapoint_blue;

	return size_msg;
}

RESOURCE(leds, METHOD_GET | METHOD_PUT , "leds", "title=\"Leds Actuator\"");

void
leds_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	PRINTF(
			"leds handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);

	char message[LED_MSG_MAX_SIZE];
	uint8_t size_msg;

	//const uint16_t *accept = NULL;
	int num = 0;
	char *err_msg;

	/* Check the offset for boundaries of t        he resource data. */
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		/* A block error message should not exceed the minimum block size (16). */
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	// due to memory constraints --> compute message for all requests
	REST.set_header_content_type(response, REST.type.APPLICATION_XML);

	if ((size_msg = create_response_object_led(message))
			<= 0) {
		PRINTF("ERROR while creating message!\n");
		REST.set_response_status(response,
				REST.status.INTERNAL_SERVER_ERROR);
		err_msg = "ERROR while creating message :\\";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}

/*
 * Red led
 */
RESOURCE(led_red, METHOD_PUT | METHOD_GET, "leds/red",
		"title=\"Red led\"");

void led_red_handler(void* request, void* response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset) {
	PRINTF(
			"led_red_handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);
	// Save the message as static variable, so it is retained through multiple calls (chunked resource)
	char message[BUTTON_MSG_MAX_SIZE];
	uint8_t size_msg;

	char *err_msg;

	int payload_len = 0;
	int newVal = 0;
	const uint8_t *incoming;

	if( REST.get_method_type(request) == METHOD_PUT){
		payload_len = REST.get_request_payload(request, &incoming);
		memcpy(payload_buffer, incoming, payload_len);
		newVal = get_bool_value_obix(payload_buffer);
		if(newVal){
			leds_on(LEDS_RED);
		}
		else{
			leds_off(LEDS_RED);
		}
	}

	// Check the offset for boundaries of the resource data.
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		// A block error message should not exceed the minimum block size (16).
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	// compute message once
	if (*offset <= 0) {
		REST.set_header_content_type(response, REST.type.APPLICATION_XML);

		if ((size_msg = create_response_datapoint_led(message, 0, 0)) <= 0) {
			PRINTF("ERROR while creating message!\n");
			REST.set_response_status(response,
					REST.status.INTERNAL_SERVER_ERROR);
			err_msg = "ERROR while creating message :\\";
			REST.set_response_payload(response, err_msg, strlen(err_msg));
			return;
		}
	}

	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}

/*
 * Green led
 */
RESOURCE(led_green, METHOD_PUT | METHOD_GET, "leds/green",
		"title=\"Green led\"");

void led_green_handler(void* request, void* response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset) {
	PRINTF(
			"led_green_handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);
	// Save the message as static variable, so it is retained through multiple calls (chunked resource)
	char message[BUTTON_MSG_MAX_SIZE];
	uint8_t size_msg;

	char *err_msg;
	const uint8_t *incoming = NULL;

	int payload_len = 0;
	int newVal = 0;

	if( REST.get_method_type(request) == METHOD_PUT){
		payload_len = REST.get_request_payload(request, &incoming);
		memcpy(payload_buffer, incoming, payload_len);
		newVal = get_bool_value_obix(payload_buffer);
		if(newVal){
			leds_on(LEDS_GREEN);
		}
		else{
			leds_off(LEDS_GREEN);
		}
	}

	// Check the offset for boundaries of the resource data.
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		// A block error message should not exceed the minimum block size (16).
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	// compute message once
	if (*offset <= 0) {
		REST.set_header_content_type(response, REST.type.APPLICATION_XML);

		if ((size_msg = create_response_datapoint_led(message, 0, 2)) <= 0) {
			PRINTF("ERROR while creating message!\n");
			REST.set_response_status(response,
					REST.status.INTERNAL_SERVER_ERROR);
			err_msg = "ERROR while creating message :\\";
			REST.set_response_payload(response, err_msg, strlen(err_msg));
			return;
		}
	}

	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}


/*
 * Handles group communication updates.
 */
void led_blue_groupCommHandler(char* payload){
	int newVal;
	newVal = get_bool_value_obix(payload);
	if(newVal){
		leds_on(LEDS_BLUE);
	}
	else{
		leds_off(LEDS_BLUE);
	}
}



/*
 * Blue led
 */
RESOURCE(led_blue, METHOD_PUT | METHOD_GET | HAS_SUB_RESOURCES | METHOD_POST, "leds/blue",
		"title=\"Blue led\"");

void led_blue_handler(void* request, void* response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset) {
	PRINTF(
			"led_blue_handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);
	// Save the message as static variable, so it is retained through multiple calls (chunked resource)
	static char message[BUTTON_MSG_MAX_SIZE];
	static uint8_t size_msg;
	int newVal = 0;
	const uint8_t *incoming = NULL;
	static size_t payload_len = 0;

	const char *uri_path = NULL;
	int len = REST.get_url(request, &uri_path);

	// for PUT and POST request we need to process the payload content
	if( REST.get_method_type(request) == METHOD_PUT || REST.get_method_type(request) == METHOD_POST){
		payload_len = REST.get_request_payload(request, &incoming);
		memcpy(payload_buffer, incoming, payload_len);
	}

#if GROUP_COMM_ENABLED
	uip_ip6addr_t groupAddress;
	gc_handler leb_blue_handler = &led_blue_groupCommHandler;

	int16_t groupIdentifier = 0;

    if(strstr(uri_path, "joinGroup") && REST.get_method_type(request) == METHOD_POST ){
    	PRINTF("Join group called.\n");
    	get_ipv6_multicast_addr(payload_buffer, &groupAddress);
    	PRINT6ADDR(&groupAddress);
    	extract_group_identifier(&groupAddress, &groupIdentifier);
    	PRINTF("\n group identifier: %d\n", groupIdentifier);
    	join_group(groupIdentifier, leb_blue_handler);


    }
    else if(strstr(uri_path, "leaveGroup") && REST.get_method_type(request) == METHOD_POST){
    	PRINTF("Leave group called.\n");
    	get_ipv6_multicast_addr(payload_buffer, &groupAddress);
    	PRINT6ADDR(&groupAddress);
    	extract_group_identifier(&groupAddress, &groupIdentifier);
    	PRINTF("\n group identifier: %d\n", groupIdentifier);
    	leave_group(groupIdentifier,  leb_blue_handler);
    }
#endif

	char *err_msg;

	if( REST.get_method_type(request) == METHOD_PUT){
		newVal = get_bool_value_obix(payload_buffer);
		if(newVal){
			leds_on(LEDS_BLUE);
		}
		else{
			leds_off(LEDS_BLUE);
		}
	}

	// Check the offset for boundaries of the resource data.
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		// A block error message should not exceed the minimum block size (16).
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	// compute message once
	if (*offset <= 0) {
		REST.set_header_content_type(response, REST.type.APPLICATION_XML);

		if ((size_msg = create_response_datapoint_led(message, 0, 1)) <= 0) {
			PRINTF("ERROR while creating message!\n");
			REST.set_response_status(response,
					REST.status.INTERNAL_SERVER_ERROR);
			err_msg = "ERROR while creating message :\\";
			REST.set_response_payload(response, err_msg, strlen(err_msg));
			return;
		}
	}

	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}
#endif // RES_LEDS


#if RES_BATTERY
int battery_to_buff(char* buffer) {
	/* get adc value for battery */
	uint32_t battery = battery_sensor.value(0);

  /* calculate percental value */
  battery = (battery * 100) / 2457;
  battery = battery > 100 ? 100 : battery;

	return snprintf(buffer, BATTERY_BUFF_MAX, "%ld", battery);
}

int battery_to_default_buff() {
	return battery_to_buff(batterystring);
}

uint8_t create_response_datapoint_battery(char *buffer,	int asChild) {
	size_t size_battery;
	int size_msgp1, size_msgp2;
	const char *msgp1, *msgp2;
	uint8_t size_msg;

	if (asChild) {
		msgp1 =
				"<int href=\"battery/value\" units=\"obix:units/percent\" val=\"";
		size_msgp1 = 58;
    msgp2 = "\"/>";
    size_msgp2 = 3;
	} else {
		msgp1 = "<int href=\"value\" units=\"obix:units/percent\" val=\"";
		size_msgp1 = 50;
    msgp2 = "\"/>\0";
    size_msgp2 = 4;
	}


	if ((size_battery = battery_to_default_buff()) < 0) {
		PRINTF("Error preparing battery string!\n");
		return 0;
	}

	size_msg = size_msgp1 + size_msgp2 + size_battery;

	memcpy(buffer, msgp1, size_msgp1);
	memcpy(buffer + size_msgp1, batterystring, size_battery);
	memcpy(buffer + size_msgp1 + size_battery, msgp2, size_msgp2);

	return size_msg;
}

uint8_t create_response_object_battery(char *buffer) {
	size_t size_datapoint;
	int size_msgp1, size_msgp2;
	const char *msgp1, *msgp2;
	uint8_t size_msg;

	msgp1 = "<obj href=\"battery\" is=\"iot:Battery\">";
	msgp2 = "</obj>\0";
	size_msgp1 = 37;
	size_msgp2 = 7;

	memcpy(buffer, msgp1, size_msgp1);
	// creates int data point and copies content to message buffer
	size_datapoint = create_response_datapoint_battery(buffer + size_msgp1, 1);

	memcpy(buffer + size_msgp1 + size_datapoint, msgp2, size_msgp2);

	size_msg = size_msgp1 + size_msgp2 + size_datapoint;

	return size_msg;
}
#endif

/*
 * Battery
 */
#if RES_BATTERY 
RESOURCE(battery, METHOD_GET, "battery", "title=\"Battery Sensor\"");
void
battery_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	PRINTF(
			"battery_handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);
	/* Save the message as static variable, so it is retained through multiple calls (chunked resource) */
	static char message[BATTERY_MSG_MAX_SIZE];
	static uint8_t size_msg;

	const uint16_t *accept = NULL;
	int num = 0;
	char *err_msg;

	/* Check the offset for boundaries of the resource data. */
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		/* A block error message should not exceed the minimum block size (16). */
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	/* compute message once */
	if (*offset <= 0) {
		/* decide upon content-format */
		num = REST.get_header_accept(request, &accept);

		REST.set_header_content_type(response, REST.type.APPLICATION_XML);

		if ((size_msg = create_response_object_battery(message)) <= 0) {
			PRINTF("ERROR while creating message!\n");
			REST.set_response_status(response,
					REST.status.INTERNAL_SERVER_ERROR);
			err_msg = "ERROR while creating message :\\";
			REST.set_response_payload(response, err_msg, strlen(err_msg));
			return;
		}
	}

	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}

RESOURCE(battery_value, METHOD_GET, "battery/value", "title=\"Battery Value\"");
void
battery_value_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	PRINTF(
			"battery_value_handler called - preferred size: %u, offset:%ld,\n", preferred_size, *offset);
	/* Save the message as static variable, so it is retained through multiple calls (chunked resource) */
	static char message[BATTERY_MSG_MAX_SIZE];
	static uint8_t size_msg;

	const uint16_t *accept = NULL;
	int num = 0;
	char *err_msg;

	/* Check the offset for boundaries of the resource data. */
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		/* A block error message should not exceed the minimum block size (16). */
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	/* compute message once */
	if (*offset <= 0) {
		/* decide upon content-format */
		num = REST.get_header_accept(request, &accept);

		REST.set_header_content_type(response, REST.type.APPLICATION_XML);

		if ((size_msg = create_response_datapoint_battery(message, 0)) <= 0) {
			PRINTF("ERROR while creating message!\n");
			REST.set_response_status(response,
					REST.status.INTERNAL_SERVER_ERROR);
			err_msg = "ERROR while creating message :\\";
			REST.set_response_payload(response, err_msg, strlen(err_msg));
			return;
		}
	}

	send_message(message, size_msg, request, response, buffer, preferred_size,
			offset);
}
#endif // RES_BATTERY














#if GROUP_COMM_ENABLED
static void
group_comm_handler(const uip_ipaddr_t *sender_addr,
         const uip_ipaddr_t *receiver_addr,
         const uint8_t *data,
         uint16_t datalen)
{
	uint16_t groupIdentifier;
	PRINT6ADDR(sender_addr);
	PRINT6ADDR(receiver_addr);
	uint8_t i,l=0;

	groupIdentifier =  ((uint8_t *)receiver_addr)[14];
    groupIdentifier <<= 8;
    groupIdentifier += ((uint8_t *)receiver_addr)[15];
    printf("\n######### Data received on group comm handler with length %d for group identifier %d\n",
		 datalen, groupIdentifier);

    for(i = 0; i < MAX_GC_GROUPS; i++){
    			if(gc_handlers[i].group_identifier == groupIdentifier){ // free slot or same slot
    				for(l=0; l < MAX_GC_HANDLERS; l++){
    					if(gc_handlers[i].handlers[l] != NULL){
    						gc_handlers[i].handlers[l](data);
    					}
    				}
    			}
        	}
}
#endif

PROCESS(iotsys_server, "IoTSyS");
AUTOSTART_PROCESSES(&iotsys_server);

#if RES_ACC
/* Accelerometer acceleration detection callback */
void accm_cb_acc(uint8_t reg) {
	acc_register_acc = reg;
	process_post(&iotsys_server, event_acc, NULL);
}
#endif // RES_ACC

#if RES_BUTTON
/* Accelerometer tap detection callback */
void accm_cb_tap(uint8_t reg) {
	acc_register_tap = reg;
	process_post(&iotsys_server, event_tap, NULL);
}
#endif // RES_BUTTON

PROCESS_THREAD(iotsys_server, ev, data) {
	uip_ipaddr_t addr;
	uip_ds6_maddr_t *maddr;
	PROCESS_BEGIN()	;

	 	uip_ip6addr(&addr, 0xff12, 0, 0, 0, 0, 0, 0, 0x1);
	 	maddr = uip_ds6_maddr_add(&addr);
	 	  if(maddr == NULL){
	 		  PRINTF("NULL returned.");
	 	  }
	 	  else{
	 		  PRINTF("Something returned.");
	 		  PRINTF("Is used: %d", maddr->isused);
	 		  PRINT6ADDR(&(maddr->ipaddr));
	 	  }
		PRINTF("Starting IoTSyS Server\n");

#if GROUP_COMM_ENABLED
		PRINTF("### Registering group comm handler.\n");
		//coap_init_connection(uip_htons(5683));
		coap_rest_implementation.set_group_comm_callback(group_comm_handler);
		/*simple_udp_register(&broadcast_connection, UDP_PORT,
		                      NULL, UDP_PORT,
		                      receiver);
		*/
#endif

#ifdef RF_CHANNEL
		PRINTF("RF channel: %u\n", RF_CHANNEL);
#endif
#ifdef IEEE802154_PANID
		PRINTF("PAN ID: 0x%04X\n", IEEE802154_PANID);
#endif

		PRINTF("uIP buffer: %u\n", UIP_BUFSIZE);
		PRINTF("LL header: %u\n", UIP_LLH_LEN);
		PRINTF("IP+UDP header: %u\n", UIP_IPUDPH_LEN);
		PRINTF("REST max chunk: %u\n", REST_MAX_CHUNK_SIZE);
		/* Initialize the REST engine. */
		rest_init_engine();

		/* Activate the application-specific resources. */
#if RES_TEMP
		rest_activate_resource(&resource_temp);
		rest_activate_periodic_resource(&periodic_resource_value);
#endif
#if RES_ACC
		rest_activate_resource(&resource_acc);
		rest_activate_event_resource(&resource_event_acc);
#endif
#if RES_BUTTON
		rest_activate_resource(&resource_button);
		rest_activate_event_resource(&resource_button_value);
		rest_activate_resource(&resource_button_value_gc);
#endif
#if RES_LEDS
		rest_activate_resource(&resource_leds);
		rest_activate_resource(&resource_led_red);
		rest_activate_resource(&resource_led_green);
		rest_activate_resource(&resource_led_blue);
#endif
#if RES_BATTERY
    SENSORS_ACTIVATE(battery_sensor);
    rest_activate_resource(&resource_battery);
    rest_activate_resource(&resource_battery_value);
#endif

		/* Setup events. */
#if RES_BUTTON
		event_tap = process_alloc_event();
#endif
#if RES_ACC
		event_acc = process_alloc_event();
#endif
		// activate temperature
#if RES_TEMP
		tmp102_init();
#endif

		/* Start and setup the accelerometer with default values, eg no interrupts enabled. */
#if RES_ACC || RES_BUTTON
		accm_init();
#endif
		/* Register the callback functions for each interrupt */
#if RES_ACC
		ACCM_REGISTER_INT1_CB(accm_cb_acc);
#endif
#if RES_BUTTON
		ACCM_REGISTER_INT2_CB(accm_cb_tap);
#endif
		/* Set what strikes the corresponding interrupts. Several interrupts per pin is
		 possible. For the eight possible interrupts, see adxl345.h and adxl345 datasheet. */
#if RES_ACC
		accm_set_irq(
				ADXL345_INT_FREEFALL | ADXL345_INT_INACTIVITY
						| ADXL345_INT_ACTIVITY, ADXL345_INT_TAP);
#endif
		//accm_set_irq(ADXL345_INT_FREEFALL,  ADXL345_INT_TAP);



		/* Define application-specific events here. */
		while (1) {
			PROCESS_WAIT_EVENT();
#if RES_BUTTON
			if (ev == event_tap) {
				printf("Tap event occured.\n");
				button_value_event_handler(&button_value_handler);
			}
#endif
#if RES_ACC
			if (ev == event_acc) {
				printf("Acc event occured.\n");
				event_acc_event_handler(&resource_event_acc);
			}
#endif
		} /* while (1) */

	PROCESS_END();
}
