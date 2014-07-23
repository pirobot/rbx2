#!/bin/bash

kill `ps -ef | grep dyna | awk '{print $2}'`

