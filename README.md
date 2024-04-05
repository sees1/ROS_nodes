multimaster_node
====================================================

[![Build Status](https://travis-ci.org/Dtananaev/ROS_nodes.svg?branch=master)](https://travis-ci.org/Dtananaev/ROS_nodes)
[![BSD2 License](http://img.shields.io/badge/license-BSD2-brightgreen.svg)](https://github.com/Dtananaev/ROS_nodes/blob/master/LICENSE.md) 

## Overview

multimaster_node is a C++ library that implement proxy for inter-master communication. It is designed specifically for cases where standard behavior for communication between individual ROS masters is insufficient to implement the required tasks.

Features:
- Supports broadcast the list of topics from `host pc` to `foreign pc` and vice verse.
- Supports broadcast the tf transforms from `host pc` to `foreign pc` and vice verse.
- Released under the [**BSD 2-Clause license**].
- Support for **ROS1** (see instructions below).

It contains:

* host2foreign - node implement broadcast from `host pc` to `foreign pc`. List of topic and transform to broadcast specified in the `host2foreign_config.yaml` file in `config` directory. Node start and parameter load in `multimaster.launch` in `launch` directory. 
 [![multimaster](https://github.com/Dtananaev/ROS_nodes/blob/master/pictures/mm.JPG)](https://www.youtube.com/watch?v=VnkDEB2HQ4E&feature=youtu.be)
     * Parameters:
         * `local pubs` - list of nedeed topics.
         * `local_tf` - list of nedeed broadcast.
         * `foreign_ip` - ip of foreign PC.
         * `foreign_port` - port of foreign PC (where ROS master hosts).
         * `msgs_pub_freq` - frequency of publishing broadcast and topics.
         * `add_topic_prefix` - add prefix ns to publishing topics(optional).
         * `strip_topic_prefix` - strip prefix ns of publishing topics(optional).
         * `add_topic_suffix` - add suffix ns to publishing topics(optional).
         * `strip_topic_suffix` - strip suffix ns of publishing topics(optional).
     * To run: 
         * open launch file `../multimaster_node/config/host2foreign_config.yaml` and add the `ip` and port of `foreign PC`, and list of nedeed transform and topics. 
         * run roscore on both PC (not necessary to run roscore on `foreign PC` before launching `multimaster` because it has autoconnect).
         * run multimaster: `roslaunch multimaster_node host2foreign.launch`.
     * Troubleshooting: 
         * Problem with sending messages between PCs, type on both PC (add to ~/.bashrc): export ROS_HOSTNAME = ip_adress_of_pc (e.g.export ROS_HOSTNAME=192.168.0.10).
         * Problem with topic loop dependency on host ROS master (when on host ROS master topic stop publishing msgs, but node on foreign PC continue publishing), main idea is not using the same publishing and subscribe topic name, instead use one of namespace parameter.
