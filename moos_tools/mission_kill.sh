#!/bin/bash

uPokeDB coug.moos DEPLOY=false, MOOS_MANUAL_OVERIDE=false
pkill -P $(cat pAntler.pid)