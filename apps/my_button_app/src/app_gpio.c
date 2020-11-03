/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

/* Bluetooth: Mesh Generic OnOff, Generic Level, Lighting & Vendor Models
 *
 * Copyright (c) 2018 Vikrant More
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp/bsp.h"
#include "console/console.h"
#include "hal/hal_gpio.h"
#include "app_gpio.h"
#include <custom_coap/custom_coap.h>        //  For sensor_value
#include "collector.h"

int button_device[2] = {
	USER_BLUE_BUTTON,
	// BUTTON_2,
	// BUTTON_3,
	// BUTTON_4,
};
/*
int led_device[2] = {
	LED_RED_PIN,
	LED_BLUE_PIN,
	// LED_3,
	// LED_4,
};
*/

static struct os_event button_event;
static TX_REASON reason;

static void gpio_irq_handler(void *arg)
{
	os_eventq_put(os_eventq_dflt_get(), &button_event);
}

void app_gpio_init(void)
{
	/* Buttons configiuratin & setting */

	button_event.ev_cb = send_datacollector;
	button_event.ev_arg = &reason;
	reason = START;

	hal_gpio_irq_init(button_device[0], gpio_irq_handler, NULL,
			  HAL_GPIO_TRIG_RISING, HAL_GPIO_PULL_NONE);
	
	if(start_datacollector()==0){
		hal_gpio_irq_enable(button_device[0]);
	}

}

