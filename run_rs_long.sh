#!/bin/sh

DATE=`date +%Y_%m_%d_%H_%M_%S`

./GUI/build/ElasticFusion -d 20 -rsr "record_$DATE.rs" "$@"

