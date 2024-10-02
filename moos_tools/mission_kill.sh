#!/bin/bash
# Created by Matthew McMurray, Sep 2024
#
# ADD HERE

uPokeDB coug.moos DEPLOY=false, MOOS_MANUAL_OVERIDE=false
pkill -P $(cat pAntler.pid)