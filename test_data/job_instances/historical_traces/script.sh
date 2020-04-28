#!/bin/sh
files="./*/*.log_crypt"
for i in $files
do
    sed '/^$/d' $i > $i.out 
    mv  $i.out $i
    done
