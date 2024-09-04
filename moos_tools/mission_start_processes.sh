#!/bin/bash

##########################################################
# just starts all the moos apps
##########################################################

pAntler coug.moos > /dev/null 2>&1 &
echo $! > pAntler.pid