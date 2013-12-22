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

/* Z1 temperature sensor */
#include "dev/i2cmaster.h"
#include "dev/tmp102.h"

/* For CoAP-specific example: not required for normal RESTful Web service. */
#if WITH_COAP == 3
#include "er-coap-03.h"
#elif WITH_COAP == 7
#include "er-coap-07.h"
#elif WITH_COAP == 12
#include "er-coap-12.h"
#elif WITH_COAP == 13
#include "er-coap-13.h"
#else
#warning "IoTSyS server without CoAP-specifc functionality"
#endif /* CoAP-specific example */

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]",(lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3],(lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif

#define CHUNKS_TOTAL      1024
#define TEMP_MSG_MAX_SIZE 140   // more than enough right now
#define TEMP_BUFF_MAX     7     // -234.6\0
/******************************************************************************/
/* globals ********************************************************************/
/******************************************************************************/
char tempstring[TEMP_BUFF_MAX];

/******************************************************************************/
/* helper functions ***********************************************************/
/******************************************************************************/

void send_message(const char* message, const uint16_t size_msg, void *request,
		void *response, uint8_t *buffer, uint16_t preferred_size,
		int32_t *offset) {
	PRINTF("Send Message: Size = %d, Offset = %d\n", size_msg, *offset);
	PRINTF("Preferred Size: %d\n", preferred_size);

	uint16_t length;
	char *err_msg;
	const char* len;

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
		PRINTF("Preferred size set to REST_MAX_CHUNK_SIZE = %d\n", preferred_size);
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
			PRINTF("Length is now %u\n", length); PRINTF("End of resource, setting offset to -1\n");
			*offset = -1;
		} else {
			/* IMPORTANT for chunk-wise resources: Signal chunk awareness to REST engine. */
			*offset += length;
			PRINTF("Offset refreshed to %u\n", *offset);
		}
	} else {
		memcpy(buffer, message + *offset, length);
		*offset = -1;
	}

	PRINTF("Sending response chunk: length = %u, offset = %d\n", length, *offset);

	REST.set_header_etag(response, (uint8_t *) &length, 1);
	REST.set_response_payload(response, buffer, length);
}

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

uint8_t create_response(int num, int accept, char *buffer) {
	size_t size_temp;
	int size_msgp1, size_msgp2;
	const char *msgp1, *msgp2;
	uint8_t size_msg;

	if (num && accept == REST.type.APPLICATION_XML) {
		msgp1 =
				"<obj href=\"temp\"><real href=\"value\" units=\"obix:units/celsius\" val=\"";
		msgp2 = "\"/></obj>\0";
		/* hardcoded length, ugly but faster and necc. for exi-answer */
		size_msgp1 = 68;
		size_msgp2 = 10;
	} else {
		PRINTF("Unsupported encoding!\n");
		return -1;
	}

	if ((size_temp = temp_to_default_buff()) < 0) {
		PRINTF("Error preparing temperature string!\n");
		return -1;
	}

	/* Some data that has the length up to REST_MAX_CHUNK_SIZE. For more, see the chunk resource. */
	/* we assume null-appended strings behind msgp2 and tempstring */
	size_msg = size_msgp1 + size_msgp2 + size_temp + 1;

	if (size_msg > TEMP_MSG_MAX_SIZE) {
		PRINTF("Message too big!\n");
		return -1;
	}

	memcpy(buffer, msgp1, size_msgp1);
	memcpy(buffer + size_msgp1, tempstring, size_temp);
	memcpy(buffer + size_msgp1 + size_temp, msgp2, size_msgp2 + 1);

	return size_msg;
}

/*
 * Example for an oBIX temperature sensor.
 */
RESOURCE(temp, METHOD_GET, "temp", "title=\"Temperature Sensor\"");

void temp_handler(void* request, void* response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset) {
	PRINTF("temp_handler called - preferred size: %u, offset:%d,\n", preferred_size, *offset);
	/* we save the message as static variable, so it is retained through multiple calls (chunked resource) */
	static char message[TEMP_MSG_MAX_SIZE];
	static uint8_t size_msg;

	const uint16_t *accept = NULL;
	int num = 0, length = 0;
	char *err_msg;

	/* Check the offset for boundaries of t	he resource data. */
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

		if ((size_msg = create_response(num, accept[0], message)) <= 0) {
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
PERIODIC_RESOURCE(temp_value, METHOD_GET, "temp/value",
		"title=\"Temperature value", 5*CLOCK_SECOND);

void temp_value_handler(void* request, void* response, uint8_t *buffer,
		uint16_t preferred_size, int32_t *offset) {

	PRINTF("temp value called\n");
	REST.set_header_content_type(response, REST.type.TEXT_PLAIN);

	/* Usually, a CoAP server would response with the resource representation matching the periodic_handler. */
	const char *msg = "It's periodic!";
	REST.set_response_payload(response, msg, strlen(msg));

	/* A post_handler that handles subscriptions will be called for periodic resources by the REST framework. */
}

/*
 * Additionally, a handler function named [resource name]_handler must be implemented for each PERIODIC_RESOURCE.
 * It will be called by the REST manager process with the defined period.
 */
void temp_value_periodic_handler(resource_t *r) {
	static uint16_t obs_counter = 0;
	static char content[11];

	++obs_counter;

	PRINTF("TICK %u for /%s\n", obs_counter, r->url);

	/* Build notification. */
	coap_packet_t notification[1]; /* This way the packet can be treated as pointer as usual. */
	coap_init_message(notification, COAP_TYPE_NON, REST.status.OK, 0);
	coap_set_payload(notification, content,
			snprintf(content, sizeof(content), "TICK %u", obs_counter));

	/* Notify the registered observers with the given message type, observe option, and payload. */
	REST.notify_subscribers(r, obs_counter, notification);
}

PROCESS(iotsys_server, "IoTSyS example server");
AUTOSTART_PROCESSES(&iotsys_server);

PROCESS_THREAD(iotsys_server, ev, data) {
	PROCESS_BEGIN();

		PRINTF("Starting IoTSyS Server\n");

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
		rest_activate_resource(&resource_temp);

		rest_activate_resource(&resource_temp_value);

		// activate temperature
		tmp102_init();

		/* Define application-specific events here. */
		while (1) {
			PROCESS_WAIT_EVENT();
		} /* while (1) */

	PROCESS_END();
}
