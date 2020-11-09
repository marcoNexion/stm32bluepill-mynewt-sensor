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

#ifndef __SEMIHOSTING_CONSOLE_PRIV_H__
#define __SEMIHOSTING_CONSOLE_PRIV_H__

#ifdef __cplusplus
extern "C" {
#endif

int rtt_console_is_init(void);
int rtt_console_init(void);
void enable_buffer(void);   //  Enable buffering.
void disable_buffer(void);  //  Disable buffering.

#ifdef __cplusplus
}
#endif

#endif /* __SEMIHOSTING_CONSOLE_PRIV_H__ */