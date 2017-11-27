/* Copyright (c) 2016-2017. The SimGrid Team. All rights reserved.          */

/* This program is free software; you can redistribute it and/or modify it
 * under the terms of the license (GNU LGPL) which comes with this package. */

#ifndef SIMGRID_PLUGINS_TEMPERATURE_H_
#define SIMGRID_PLUGINS_TEMPERATURE_H_

#include <xbt/base.h>
#include <simgrid/forward.h>

SG_BEGIN_DECL()

XBT_PUBLIC(void) sg_host_temperature_plugin_init();
XBT_PUBLIC(void) sg_host_temperature_update_all();
XBT_PUBLIC(double) sg_host_get_temperature(sg_host_t host);
XBT_PUBLIC(double) sg_host_get_ambient_temperature(sg_host_t host);


#define MSG_host_temperature_plugin_init() sg_host_temperature_plugin_init()
#define MSG_host_get_temperature(host) sg_host_get_temperature(host)
#define MSG_host_get_ambient_temperature(host) sg_host_get_ambient_temperature(host)

SG_END_DECL()

#endif
